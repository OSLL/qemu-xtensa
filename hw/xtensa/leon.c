/*
 * Copyright (c) 2011, Max Filippov, Open Source and Linux Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source and Linux Lab nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "cpu.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "chardev/char.h"
#include "sysemu/device_tree.h"
#include "qemu/error-report.h"
#include "qemu/option.h"
#include "bootparam.h"
#include "xtensa_memory.h"
#include "hw/char/pl011.h"
#include "sysemu/reset.h"

typedef struct LeonBoardDesc {
    size_t sram_size;
} LeonBoardDesc;

static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

static void xtfpga_reset(void *opaque)
{
    XtensaCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static void leon_board_init(const LeonBoardDesc *board, MachineState *machine)
{
#ifdef TARGET_WORDS_BIGENDIAN
    int be = 1;
#else
    int be = 0;
#endif
    MemoryRegion *system_memory = get_system_memory();
    XtensaCPU *cpu = NULL;
    CPUXtensaState *env = NULL;
    QemuOpts *machine_opts = qemu_get_machine_opts();
    const char *kernel_filename = qemu_opt_get(machine_opts, "kernel");
    const char *kernel_cmdline = qemu_opt_get(machine_opts, "append");
    const char *dtb_filename = qemu_opt_get(machine_opts, "dtb");
    const char *initrd_filename = qemu_opt_get(machine_opts, "initrd");
    int n;

    for (n = 0; n < machine->smp.cpus; n++) {
        cpu = XTENSA_CPU(cpu_create(machine->cpu_type));
        env = &cpu->env;

        env->sregs[PRID] = n;
        qemu_register_reset(xtfpga_reset, cpu);
        /* Need MMU initialized prior to ELF loading,
         * so that ELF gets loaded into virtual addresses
         */
        cpu_reset(CPU(cpu));
    }

    if (env) {
        XtensaMemory sysram = env->config->sysram;

        sysram.location[0].size = machine->ram_size;
        xtensa_create_memory_regions(&env->config->instrom, "xtensa.instrom",
                                     system_memory);
        xtensa_create_memory_regions(&env->config->instram, "xtensa.instram",
                                     system_memory);
        xtensa_create_memory_regions(&env->config->datarom, "xtensa.datarom",
                                     system_memory);
        xtensa_create_memory_regions(&env->config->dataram, "xtensa.dataram",
                                     system_memory);
        xtensa_create_memory_regions(&sysram, "xtensa.sysram",
                                     system_memory);
    }

    pl011_create(0xe0f20000, xtensa_get_extints(env)[0], serial_hd(0));

    /* Use presence of kernel file name as 'boot from SRAM' switch. */
    if (kernel_filename) {
        uint32_t entry_point = env->pc;
        size_t bp_size = 3 * get_tag_size(0); /* first/last and memory tags */
        uint32_t tagptr;
        uint32_t cur_tagptr;
        BpMemInfo memory_location = {
            .type = tswap32(MEMORY_TYPE_CONVENTIONAL),
            .start = tswap32(env->config->sysram.location[0].addr),
            .end = tswap32(env->config->sysram.location[0].addr +
                           machine->ram_size),
        };
        uint32_t lowmem_end = machine->ram_size < 0x08000000 ?
            machine->ram_size : 0x08000000;
        uint32_t cur_lowmem = QEMU_ALIGN_UP(lowmem_end / 2, 4096);

        lowmem_end += env->config->sysram.location[0].addr;
        cur_lowmem += env->config->sysram.location[0].addr;

        xtensa_create_memory_regions(&env->config->sysrom, "xtensa.sysrom",
                                     system_memory);

        if (kernel_cmdline) {
            bp_size += get_tag_size(strlen(kernel_cmdline) + 1);
        }
        if (dtb_filename) {
            bp_size += get_tag_size(sizeof(uint32_t));
        }
        if (initrd_filename) {
            bp_size += get_tag_size(sizeof(BpMemInfo));
        }

        /* Put kernel bootparameters to the end of that low memory */
        tagptr = (lowmem_end - bp_size) & ~0xff;
	lowmem_end = tagptr;
        cur_tagptr = put_tag(tagptr, BP_TAG_FIRST, 0, NULL);
        cur_tagptr = put_tag(cur_tagptr, BP_TAG_MEMORY,
                             sizeof(memory_location), &memory_location);

        if (kernel_cmdline) {
            cur_tagptr = put_tag(cur_tagptr, BP_TAG_COMMAND_LINE,
                                 strlen(kernel_cmdline) + 1, kernel_cmdline);
        }
#ifdef CONFIG_FDT
        if (dtb_filename) {
            int fdt_size;
            void *fdt = load_device_tree(dtb_filename, &fdt_size);
            uint32_t dtb_addr = tswap32(cur_lowmem);

            if (!fdt) {
                error_report("could not load DTB '%s'", dtb_filename);
                exit(EXIT_FAILURE);
            }

            cpu_physical_memory_write(cur_lowmem, fdt, fdt_size);
            cur_tagptr = put_tag(cur_tagptr, BP_TAG_FDT,
                                 sizeof(dtb_addr), &dtb_addr);
            cur_lowmem = QEMU_ALIGN_UP(cur_lowmem + fdt_size, 4 * KiB);
        }
#else
        if (dtb_filename) {
            error_report("could not load DTB '%s': "
                         "FDT support is not configured in QEMU",
                         dtb_filename);
            exit(EXIT_FAILURE);
        }
#endif
        if (initrd_filename) {
            BpMemInfo initrd_location = { 0 };
            int initrd_size = load_ramdisk(initrd_filename, cur_lowmem,
                                           lowmem_end - cur_lowmem);

            if (initrd_size < 0) {
                initrd_size = load_image_targphys(initrd_filename,
                                                  cur_lowmem,
                                                  lowmem_end - cur_lowmem);
            }
            if (initrd_size < 0) {
                error_report("could not load initrd '%s'", initrd_filename);
                exit(EXIT_FAILURE);
            }
            initrd_location.start = tswap32(cur_lowmem);
            initrd_location.end = tswap32(cur_lowmem + initrd_size);
            cur_tagptr = put_tag(cur_tagptr, BP_TAG_INITRD,
                                 sizeof(initrd_location), &initrd_location);
            cur_lowmem = QEMU_ALIGN_UP(cur_lowmem + initrd_size, 4 * KiB);
        }
        cur_tagptr = put_tag(cur_tagptr, BP_TAG_LAST, 0, NULL);
        env->regs[2] = tagptr;

        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(kernel_filename, NULL,
                               translate_phys_addr, cpu,
                               &elf_entry, &elf_lowaddr, NULL,
                               NULL, be, EM_XTENSA, 0, 0);
        if (success > 0) {
            entry_point = elf_entry;
        } else {
            hwaddr ep;
            int is_linux;
            success = load_uimage(kernel_filename, &ep, NULL, &is_linux,
                                  translate_phys_addr, cpu);
            if (success > 0 && is_linux) {
                entry_point = ep;
            } else {
                error_report("could not load kernel '%s'",
                             kernel_filename);
                exit(EXIT_FAILURE);
            }
        }
        if (entry_point != env->pc) {
            uint8_t boot[] = {
#ifdef TARGET_WORDS_BIGENDIAN
                0x60, 0x00, 0x08,       /* j    1f */
                0x00,                   /* .literal_position */
                0x00, 0x00, 0x00, 0x00, /* .literal entry_pc */
                0x00, 0x00, 0x00, 0x00, /* .literal entry_a2 */
                                        /* 1: */
                0x10, 0xff, 0xfe,       /* l32r a0, entry_pc */
                0x12, 0xff, 0xfe,       /* l32r a2, entry_a2 */
                0x0a, 0x00, 0x00,       /* jx   a0 */
#else
                0x06, 0x02, 0x00,       /* j    1f */
                0x00,                   /* .literal_position */
                0x00, 0x00, 0x00, 0x00, /* .literal entry_pc */
                0x00, 0x00, 0x00, 0x00, /* .literal entry_a2 */
                                        /* 1: */
                0x01, 0xfe, 0xff,       /* l32r a0, entry_pc */
                0x21, 0xfe, 0xff,       /* l32r a2, entry_a2 */
                0xa0, 0x00, 0x00,       /* jx   a0 */
#endif
            };
            uint32_t entry_pc = tswap32(entry_point);
            uint32_t entry_a2 = tswap32(tagptr);

            memcpy(boot + 4, &entry_pc, sizeof(entry_pc));
            memcpy(boot + 8, &entry_a2, sizeof(entry_a2));
            cpu_physical_memory_write(env->pc, boot, sizeof(boot));
        }
    } else {
        xtensa_create_memory_regions(&env->config->sysrom, "xtensa.sysrom",
                                     system_memory);
    }
}

static void leon_init(MachineState *machine)
{
    static const LeonBoardDesc leon_board = {
        .sram_size = 0x2000000,
    };
    leon_board_init(&leon_board, machine);
}

static void leon_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "leon (cp0_latest)";
    mc->init = leon_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = XTENSA_CPU_TYPE_NAME("cp0_latest");
    mc->default_ram_size = 0x20000000;
}

static const TypeInfo leon_type = {
    .name = MACHINE_TYPE_NAME("leon"),
    .parent = TYPE_MACHINE,
    .class_init = leon_class_init,
};

static void leon_machine_init(void)
{
    type_register_static(&leon_type);
}

type_init(leon_machine_init)
