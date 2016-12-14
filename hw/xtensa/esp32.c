/*
 * Copyright (c) 2016, Max Filippov, Open Source and Linux Lab.
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

#ifndef TARGET_WORDS_BIGENDIAN
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"

typedef struct Esp32 {
    XtensaCPU *cpu[2];
} Esp32;

static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

static void esp32_reset(void *opaque)
{
    Esp32 *esp32 = opaque;
    int i;

    for (i = 0; i < 2; ++i) {
        cpu_reset(CPU(esp32->cpu[i]));
        xtensa_runstall(&esp32->cpu[i]->env, i > 0);
    }
}

struct MemoryArea {
    uint32_t base;
    uint32_t size;
    const char *name;
};

static void xtensa_esp32_init(MachineState *machine)
{
    static const struct MemoryArea rom[] = {
        { 0x3ff90000, 0x00010000, "esp32-3ff90000.rom" },
        { 0x40000000, 0x00060000, "esp32-40000000.rom" },
    };
    static const struct MemoryArea ram[] = {
        { 0x3f400000, 0x00400000, "flash.d" },
        { 0x3f800000, 0x00400000, "ext.sram.d" },
        { 0x3ff90000, 0x00010000, "rom1" },
        { 0x3ffae000, 0x00022000, "sram2" },
        { 0x3ffe0000, 0x00020000, "sram1.dram" },
        { 0x40000000, 0x00060000, "rom0" },
        { 0x40070000, 0x00010000, "sram0.cache" },
        { 0x40080000, 0x00020000, "sram0" },
        { 0x400a0000, 0x00010000, "sram1.iram" },
        { 0x400b0000, 0x00010000, "sram1.iram.remap" },
        { 0x400c2000, 0x00b3e000, "flash.i" },
    };
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = machine->kernel_filename;
    Esp32 *esp32 = g_malloc0(sizeof(*esp32));
    int i;

    if (!cpu_model) {
        cpu_model = "esp32";
    }

    for (i = 0; i < 2; ++i) {
        static const uint32_t prid[] = {
            0xcdcd,
            0xabab,
        };
        XtensaCPU *cpu = cpu_xtensa_init(cpu_model);

        if (cpu == NULL) {
            error_report("unable to find CPU definition '%s'",
                         cpu_model);
            exit(EXIT_FAILURE);
        }

        esp32->cpu[i] = cpu;
        cpu->env.sregs[PRID] = prid[i];
    }

    qemu_register_reset(esp32_reset, esp32);

    for (i = 0; i < ARRAY_SIZE(ram); ++i) {
        MemoryRegion *ram_region = g_malloc(sizeof(*ram_region));

        memory_region_init_ram(ram_region, NULL, ram[i].name, ram[i].size,
                               &error_fatal);
        vmstate_register_ram_global(ram_region);
        memory_region_add_subregion(get_system_memory(),
                                    ram[i].base, ram_region);
    }

    if (kernel_filename) {
        XtensaCPU *cpu = esp32->cpu[0];
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success;

        /* Need MMU initialized prior to ELF loading,
         * so that ELF gets loaded into virtual addresses
         */
        cpu_reset(CPU(cpu));

        success = load_elf(kernel_filename, translate_phys_addr, cpu,
                           &elf_entry, &elf_lowaddr, NULL, 0, EM_XTENSA, 0, 0);
        if (success > 0) {
            cpu->env.pc = elf_entry;
        }
    }

    for (i = 0; i < ARRAY_SIZE(rom); ++i) {
        const char *rom_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS,
                                                  rom[i].name);
        if (!rom_filename ||
            load_image_targphys(rom_filename, rom[i].base, rom[i].size) < 0) {
            error_report("unable to load ROM image '%s'\n", rom[i].name);
            exit(EXIT_FAILURE);
        }
    }
}

static void xtensa_esp32_machine_init(MachineClass *mc)
{
    mc->desc = "ESP32 machine (esp32)";
    mc->is_default = false;
    mc->init = xtensa_esp32_init;
    mc->max_cpus = 2;
}

DEFINE_MACHINE("esp32", xtensa_esp32_machine_init)

#endif
