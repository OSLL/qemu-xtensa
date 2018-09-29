/*
 * Copyright (c) 2015, Max Filippov, Open Source and Linux Lab.
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
#include "cpu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "qemu/timer.h"

#define PMU_COUNTER_MAX 8

typedef struct XtensaPmu {
    MemoryRegion reg;
    CPUXtensaState *env;

    uint32_t pmg;
    uint32_t pm[PMU_COUNTER_MAX];
    uint32_t pmctrl[PMU_COUNTER_MAX];
    uint32_t pmstat[PMU_COUNTER_MAX];

    void *irq;
    bool irq_active;
    QEMUTimer *timer;
    uint64_t time_base;
} XtensaPmu;

static void xtensa_pmu_update_irq(XtensaPmu *pmu)
{
    bool active = false;

    if (pmu->pmg & 1) {
        unsigned i;

        for (i = 0; i < PMU_COUNTER_MAX; ++i)
            if (pmu->pmstat[i] & 0x1) {
                active = true;
                break;
            }
    }
    if (active != pmu->irq_active) {
        pmu->irq_active = active;
        qemu_set_irq(pmu->irq, active);
    }
}

static void xtensa_pmu_update_count(XtensaPmu *pmu)
{
    bool irq = false;
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (pmu->pmg & 0x1) {
        uint64_t d64 = muldiv64(now - pmu->time_base,
                                pmu->env->config->clock_freq_khz,
                                1000000);
        uint32_t d = (uint32_t)d64;
        bool over = d != d64;
        unsigned i;

        for (i = 0; i < PMU_COUNTER_MAX; ++i)
            if (pmu->pmctrl[i] & 0xffff0000) {
                uint32_t new = pmu->pm[i] + d;

                if ((~new & pmu->pm[i] & 0x80000000) || over) {
                    pmu->pmstat[i] |= 0x1;
                    if ((pmu->pmctrl[i] & 0x1) && !pmu->irq_active) {
                        pmu->pmstat[i] |= 0x10;
                        irq = true;
                    }
                }
                pmu->pm[i] = new;
            }
    }
    pmu->time_base = now;
    if (irq) {
        xtensa_pmu_update_irq(pmu);
    }
}

static void xtensa_pmu_update_timer(XtensaPmu *pmu)
{
    uint64_t dt = 0x100000001ull;

    if (pmu->pmg & 0x1) {
        unsigned i;

        for (i = 0; i < PMU_COUNTER_MAX; ++i)
            if ((pmu->pmctrl[i] & 0xffff0000) &&
                (pmu->pmctrl[i] & 0x1)) {
                uint64_t d = 0x100000000ull - pmu->pm[i];

                if (d < dt)
                    dt = d;
            }
    }
    if (dt != 0x100000001ull) {
        timer_mod(pmu->timer,
                  pmu->time_base + muldiv64(dt, 1000000,
                                            pmu->env->config->clock_freq_khz));
    } else {
        timer_del(pmu->timer);
    }
}

static void xtensa_pmu_timer_cb(void *opaque)
{
    xtensa_pmu_update_count(opaque);
    xtensa_pmu_update_irq(opaque);
}

static uint64_t xtensa_pmu_read(void *opaque, hwaddr offset, unsigned size)
{
    XtensaPmu *pmu = opaque;

    if (offset % 4 || size != 4)
        return 0;

    if (offset == 0) {
        return pmu->pmg;
    } else if (offset >= 0x80 && offset < 0xa0) {
        xtensa_pmu_update_count(pmu);
        return pmu->pm[(offset - 0x80) / 4];
    } else if (offset >= 0x100 && offset < 0x120) {
        return pmu->pmctrl[(offset - 0x100) / 4];
    } else if (offset >= 0x180 && offset < 0x1a0) {
        xtensa_pmu_update_count(pmu);
        return pmu->pmstat[(offset - 0x180) / 4];
    } else {
        return 0;
    }
}

static void xtensa_pmu_write(void *opaque, hwaddr offset, uint64_t v, unsigned size)
{
    XtensaPmu *pmu = opaque;

    if (offset % 4 || size != 4)
        return;

    xtensa_pmu_update_count(pmu);
    if (offset == 0) {
        pmu->pmg = v & 1;
        xtensa_pmu_update_timer(pmu);
        xtensa_pmu_update_irq(pmu);
    } else if (offset >= 0x80 && offset < 0xa0) {
        pmu->pm[(offset - 0x80) / 4] = v;
        xtensa_pmu_update_timer(pmu);
    } else if (offset >= 0x100 && offset < 0x120) {
        pmu->pmctrl[(offset - 0x100) / 4] = v & 0xffff1ff9;
        xtensa_pmu_update_timer(pmu);
        xtensa_pmu_update_irq(pmu);
    } else if (offset >= 0x180 && offset < 0x1a0) {
        pmu->pmstat[(offset - 0x180) / 4] &= ~(v & 0x11);
        xtensa_pmu_update_irq(pmu);
    }
}

static const MemoryRegionOps xtensa_pmu_ops = {
    .read = xtensa_pmu_read,
    .write = xtensa_pmu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .unaligned = true,
    },
};

void xtensa_pmu_init(CPUXtensaState *env)
{
    if (env->config->inttype_mask[INTTYPE_PROFILING]) {
        XtensaPmu *pmu = calloc(1, sizeof(XtensaPmu));
        unsigned irq = ctz32(env->config->inttype_mask[INTTYPE_PROFILING]);

        pmu->env = env;
        pmu->irq = env->irq_inputs[irq];
        pmu->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  &xtensa_pmu_timer_cb, pmu);

        memory_region_init_io(&pmu->reg, NULL, &xtensa_pmu_ops, pmu,
                              "pmu.reg", 0x1a0);
        memory_region_add_subregion(xtensa_get_er_region(env),
                                    0x1000, &pmu->reg);
    }
}
