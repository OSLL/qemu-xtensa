/*
 * Copyright (c) 2019, Max Filippov, Open Source and Linux Lab.
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
#include "qapi/error.h"
#include "cpu.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/irq.h"
#include "elf.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "xtensa_memory.h"
#include "xtensa_sim.h"

typedef struct XrpSimpleState {
    MemoryRegion iomem;
    XtensaCPU *cpu;
    int idx;
    uint32_t reg[8];
    qemu_irq irq[3];

} XrpSimpleState;

static void xrp_simple_reset(void *opaque)
{
    XrpSimpleState *s = opaque;

    memset(s->reg, 0, sizeof(s->reg));
    s->reg[2] = (s->idx != 0);
    xtensa_runstall(&s->cpu->env, s->reg[2] & 1);
    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
    qemu_set_irq(s->irq[2], 0);
}

static uint64_t xrp_simple_read(void *opaque, hwaddr addr, unsigned size)
{
    XrpSimpleState *s = opaque;

    if (addr < sizeof(s->reg)) {
        return s->reg[addr / 4];
    } else {
        return 0;
    }
}

static void xrp_simple_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    XrpSimpleState *s = opaque;

    if (addr < sizeof(s->reg)) {
        s->reg[addr / 4] = val;

        switch (addr / 4) {
        case 0:
            qemu_set_irq(s->irq[0], val & 1);
            qemu_set_irq(s->irq[1], val & 2);
            qemu_set_irq(s->irq[2], val & 4);
            break;

        case 1:
            cpu_reset(CPU(s->cpu));
            break;

        case 2:
            xtensa_runstall(&s->cpu->env, val & 1);
            break;
        }
    }
}

static const MemoryRegionOps xrp_simple_ops = {
    .read = xrp_simple_read,
    .write = xrp_simple_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void sim_reset(void *opaque)
{
    XtensaCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static XtensaCPU *xtensa_xrp_simple_common_init(MachineState *machine)
{
    XtensaCPU *cpu0 = NULL;
    int n;

    for (n = 0; n < machine->smp.cpus; n++) {
        CPUState *cs = cpu_create(n ? machine->cpu_type :
                                  XTENSA_CPU_TYPE_NAME("dc233c"));
        XtensaCPU *cpu = XTENSA_CPU(cs);
        CPUXtensaState *env = &cpu->env;
        qemu_irq *extints = xtensa_get_extints(env);
        MemoryRegion *system_alias = g_malloc(sizeof(*system_alias));
        XrpSimpleState *xrp_mmio = g_malloc(sizeof(*xrp_mmio));

        if (n == 0) {
            cpu0 = cpu;
        } else {
            xtensa_runstall(env, true);
        }
        memory_region_init_alias(system_alias, NULL, "system",
                                 get_system_memory(),
                                 0, UINT64_C(0x100000000));
        memory_region_add_subregion_overlap(cs->memory, 0, system_alias, -1);

        env->sregs[PRID] = n - 1;
        qemu_register_reset(sim_reset, cpu);
        /* Need MMU initialized prior to ELF loading,
         * so that ELF gets loaded into virtual addresses
         */
        sim_reset(cpu);

        xtensa_create_memory_regions(&env->config->instrom, "xtensa.instrom",
                                     n, cs->memory);
        xtensa_create_memory_regions(&env->config->instram, "xtensa.instram",
                                     n, cs->memory);
        xtensa_create_memory_regions(&env->config->datarom, "xtensa.datarom",
                                     n, cs->memory);
        xtensa_create_memory_regions(&env->config->dataram, "xtensa.dataram",
                                     n, cs->memory);

        xrp_mmio->cpu = cpu;
        xrp_mmio->idx = n;
        if (n == 0) {
            xrp_mmio->irq[0] = extints[10];
            xrp_mmio->irq[1] = extints[11];
            xrp_mmio->irq[2] = extints[12];
        } else {
            xrp_mmio->irq[0] = extints[4];
            xrp_mmio->irq[1] = extints[5];
            xrp_mmio->irq[2] = extints[12];
        }
        memory_region_init_io(&xrp_mmio->iomem, NULL, &xrp_simple_ops, xrp_mmio,
                              "xrp.mmio", 0x200);
        memory_region_add_subregion(get_system_memory(),
                                    0x30000000 + 0x1000 * n,
                                    &xrp_mmio->iomem);
        qemu_register_reset(xrp_simple_reset, xrp_mmio);
    }

    if (cpu0) {
        XtensaMemory sysram = cpu0->env.config->sysram;
		XtensaMemory shared = {
			.num = 1,
			.location = {
				[0] = {
					.addr = 0xe0000000,
					.size = 0x1e000000,
				},
			},
		};

        sysram.location[0].size = machine->ram_size;
        xtensa_create_memory_regions(&cpu0->env.config->sysrom, "xtensa.sysrom",
                                     -1, get_system_memory());
        xtensa_create_memory_regions(&sysram, "xtensa.sysram",
                                     -1, get_system_memory());
        xtensa_create_memory_regions(&shared, "xtensa.shared",
                                     -1, get_system_memory());
    }

    if (serial_hd(0)) {
        xtensa_sim_open_console(serial_hd(0));
    }
    return cpu0;
}

static void xtensa_xrp_simple_init(MachineState *machine)
{
    XtensaCPU *cpu = xtensa_xrp_simple_common_init(machine);

    xtensa_sim_load_kernel(cpu, machine);
}

static void xtensa_xrp_simple_machine_init(MachineClass *mc)
{
    mc->desc = "xrp simple machine (dc233c + visionp6_ao)";
    mc->init = xtensa_xrp_simple_init;
    mc->max_cpus = 5;
    mc->no_serial = 1;
    mc->default_cpu_type = XTENSA_CPU_TYPE_NAME("visionp6_ao");
    mc->default_ram_size = 512 * MiB;
}

DEFINE_MACHINE("xrp_simple", xtensa_xrp_simple_machine_init)
