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

#include "exec.h"
#include "helpers.h"
#include "host-utils.h"

static void do_unaligned_access(target_ulong addr, int is_write, int is_user,
        void *retaddr);

#define ALIGNED_ONLY
#define MMUSUFFIX _mmu

#define SHIFT 0
#include "softmmu_template.h"

#define SHIFT 1
#include "softmmu_template.h"

#define SHIFT 2
#include "softmmu_template.h"

#define SHIFT 3
#include "softmmu_template.h"

static void do_restore_state(void *pc_ptr)
{
    TranslationBlock *tb;
    uint32_t pc = (uint32_t)(intptr_t)pc_ptr;

    tb = tb_find_pc(pc);
    if (tb) {
        if (cpu_restore_state(tb, env, pc)) {
            qemu_log("cpu_restore_state(%08x) failed\n", pc);
        }
    }
}

static void do_unaligned_access(target_ulong addr, int is_write, int is_user,
        void *retaddr)
{
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_UNALIGNED_EXCEPTION) &&
            !xtensa_option_enabled(env->config, XTENSA_OPTION_HW_ALIGNMENT)) {
        do_restore_state(retaddr);
        HELPER(exception_cause_vaddr)(
                env->pc, LOAD_STORE_ALIGNMENT_CAUSE, addr);
    }
}

void tlb_fill(target_ulong addr, int is_write, int mmu_idx, void *retaddr)
{
    CPUState *saved_env = env;

    env = cpu_single_env;

    qemu_log("%s(%08x, %d, %d, %d)\n",
            __func__, addr, is_write, mmu_idx, retaddr != 0);

    if (env->config->options &
            (XTENSA_OPTION_BIT(XTENSA_OPTION_MMU) |
             XTENSA_OPTION_BIT(XTENSA_OPTION_REGION_PROTECTION) |
             XTENSA_OPTION_BIT(XTENSA_OPTION_REGION_TRANSLATION))) {
        int ret = xtensa_handle_mmu_fault(env, addr, is_write, mmu_idx);
        if (ret != 0) {
            do_restore_state(retaddr);
            HELPER(exception_cause_vaddr)(env->pc, ret, addr);
        }
    } else {
        tlb_set_page(env,
                addr & TARGET_PAGE_MASK,
                addr & TARGET_PAGE_MASK,
                PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                mmu_idx, TARGET_PAGE_SIZE);
    }
    env = saved_env;
}

void HELPER(exception)(uint32_t excp)
{
    env->exception_index = excp;
    cpu_loop_exit();
}

void HELPER(exception_cause)(uint32_t pc, uint32_t cause)
{
    uint32_t vector;

    env->pc = pc;
    if (env->sregs[PS] & PS_EXCM) {
        if (env->config->ndepc) {
            env->sregs[DEPC] = pc;
        } else {
            env->sregs[EPC1] = pc;
        }
        vector = EXC_DOUBLE;
    } else {
        env->sregs[EPC1] = pc;
        vector = (env->sregs[PS] & PS_UM) ? EXC_USER : EXC_KERNEL;
    }

    env->sregs[EXCCAUSE] = cause;
    env->sregs[PS] |= PS_EXCM;

    HELPER(exception)(vector);
}

void HELPER(exception_cause_vaddr)(uint32_t pc, uint32_t cause, uint32_t vaddr)
{
    env->sregs[EXCVADDR] = vaddr;
    HELPER(exception_cause)(pc, cause);
}

uint32_t HELPER(nsa)(uint32_t v)
{
    if (v & 0x80000000) {
        v = ~v;
    }
    return v ? clz32(v) - 1 : 31;
}

uint32_t HELPER(nsau)(uint32_t v)
{
    return v ? clz32(v) : 32;
}

static void copy_window_from_phys(CPUState *env,
        uint32_t window, uint32_t phys, uint32_t n)
{
    assert(phys < env->config->nareg);
    if (phys + n <= env->config->nareg) {
        memcpy(env->regs + window, env->phys_regs + phys,
                n * sizeof(uint32_t));
    } else {
        uint32_t n1 = env->config->nareg - phys;
        memcpy(env->regs + window, env->phys_regs + phys,
                n1 * sizeof(uint32_t));
        memcpy(env->regs + window + n1, env->phys_regs,
                (n - n1) * sizeof(uint32_t));
    }
}

static void copy_phys_from_window(CPUState *env,
        uint32_t phys, uint32_t window, uint32_t n)
{
    assert(phys < env->config->nareg);
    if (phys + n <= env->config->nareg) {
        memcpy(env->phys_regs + phys, env->regs + window,
                n * sizeof(uint32_t));
    } else {
        uint32_t n1 = env->config->nareg - phys;
        memcpy(env->phys_regs + phys, env->regs + window,
                n1 * sizeof(uint32_t));
        memcpy(env->phys_regs, env->regs + window + n1,
                (n - n1) * sizeof(uint32_t));
    }
}


#define WINDOWBASE_BOUND(a) ((a) & (env->config->nareg / 4 - 1))
#define WINDOW_BOUND(a) ((a) & (env->config->nareg - 1))
#define WINDOWSTART_BIT(a) (1 << WINDOWBASE_BOUND(a))

void xtensa_sync_window_from_phys(CPUState *env)
{
    copy_window_from_phys(env, 0, env->sregs[WINDOW_BASE] * 4, 16);
}

void xtensa_sync_phys_from_window(CPUState *env)
{
    copy_phys_from_window(env, env->sregs[WINDOW_BASE] * 4, 0, 16);
}

static void rotate_window_abs(uint32_t position)
{
    xtensa_sync_phys_from_window(env);
    env->sregs[WINDOW_BASE] = WINDOWBASE_BOUND(position);
    xtensa_sync_window_from_phys(env);
}

static void rotate_window(uint32_t delta)
{
    rotate_window_abs(env->sregs[WINDOW_BASE] + delta);
}

void HELPER(wsr_windowbase)(uint32_t v)
{
    rotate_window_abs(v);
}

void HELPER(entry)(uint32_t pc, uint32_t s, uint32_t imm)
{
    int callinc = (env->sregs[PS] & PS_CALLINC) >> PS_CALLINC_SHIFT;
    if (s > 3 || ((env->sregs[PS] & (PS_WOE | PS_EXCM)) ^ PS_WOE) != 0) {
        qemu_log("Illegal entry instruction(pc = %08x), PS = %08x\n",
                pc, env->sregs[PS]);
        HELPER(exception_cause)(pc, ILLEGAL_INSTRUCTION_CAUSE);
    } else {
        env->regs[(callinc << 2) | (s & 3)] = env->regs[s] - (imm << 3);
        rotate_window(callinc);
        env->sregs[WINDOW_START] |= WINDOWSTART_BIT(env->sregs[WINDOW_BASE]);
    }
}

void HELPER(window_check)(uint32_t pc, uint32_t w)
{
    uint32_t windowbase = WINDOWBASE_BOUND(env->sregs[WINDOW_BASE]);
    uint32_t windowstart = env->sregs[WINDOW_START];
    uint32_t m, n;

    if ((env->sregs[PS] & (PS_WOE | PS_EXCM)) ^ PS_WOE) {
        return;
    }

    for (n = 1; ; ++n) {
        if (n > w) {
            return;
        }
        if (windowstart & WINDOWSTART_BIT(windowbase + n)) {
            break;
        }
    }

    m = WINDOWBASE_BOUND(windowbase + n);
    rotate_window(n);
    env->sregs[PS] = (env->sregs[PS] & ~PS_OWB) |
        (windowbase << PS_OWB_SHIFT) | PS_EXCM;
    env->sregs[EPC1] = env->pc = pc;

    if (windowstart & WINDOWSTART_BIT(m + 1)) {
        HELPER(exception)(EXC_WINDOW_OVERFLOW4);
    } else if (windowstart & WINDOWSTART_BIT(m + 2)) {
        HELPER(exception)(EXC_WINDOW_OVERFLOW8);
    } else {
        HELPER(exception)(EXC_WINDOW_OVERFLOW12);
    }
}

uint32_t HELPER(retw)(uint32_t pc)
{
    int n = (env->regs[0] >> 30) & 0x3;
    int m = 0;
    uint32_t windowbase = WINDOWBASE_BOUND(env->sregs[WINDOW_BASE]);
    uint32_t windowstart = env->sregs[WINDOW_START];
    uint32_t ret_pc = 0;

    if (windowstart & WINDOWSTART_BIT(windowbase - 1)) {
        m = 1;
    } else if (windowstart & WINDOWSTART_BIT(windowbase - 2)) {
        m = 2;
    } else if (windowstart & WINDOWSTART_BIT(windowbase - 3)) {
        m = 3;
    }

    if (n == 0 || (m != 0 && m != n) ||
            ((env->sregs[PS] & (PS_WOE | PS_EXCM)) ^ PS_WOE) != 0) {
        qemu_log("Illegal retw instruction(pc = %08x), "
                "PS = %08x, m = %d, n = %d\n",
                pc, env->sregs[PS], m, n);
        HELPER(exception_cause)(pc, ILLEGAL_INSTRUCTION_CAUSE);
    } else {
        int owb = windowbase;

        ret_pc = (pc & 0xc0000000) | (env->regs[0] & 0x3fffffff);

        rotate_window(-n);
        if (windowstart & WINDOWSTART_BIT(env->sregs[WINDOW_BASE])) {
            env->sregs[WINDOW_START] &= ~WINDOWSTART_BIT(owb);
        } else {
            /* window underflow */
            env->sregs[PS] = (env->sregs[PS] & ~PS_OWB) |
                (windowbase << PS_OWB_SHIFT) | PS_EXCM;
            env->sregs[EPC1] = env->pc = pc;

            if (n == 1) {
                HELPER(exception)(EXC_WINDOW_UNDERFLOW4);
            } else if (n == 2) {
                HELPER(exception)(EXC_WINDOW_UNDERFLOW8);
            } else if (n == 3) {
                HELPER(exception)(EXC_WINDOW_UNDERFLOW12);
            }
        }
    }
    return ret_pc;
}

void HELPER(rotw)(uint32_t imm4)
{
    rotate_window(imm4);
}

void HELPER(restore_owb)(void)
{
    rotate_window_abs((env->sregs[PS] & PS_OWB) >> PS_OWB_SHIFT);
}

void HELPER(movsp)(uint32_t pc)
{
    if ((env->sregs[WINDOW_START] &
            (WINDOWSTART_BIT(env->sregs[WINDOW_BASE] - 3) |
             WINDOWSTART_BIT(env->sregs[WINDOW_BASE] - 2) |
             WINDOWSTART_BIT(env->sregs[WINDOW_BASE] - 1))) == 0) {
        HELPER(exception_cause)(pc, ALLOCA_CAUSE);
    }
}

void HELPER(wsr_lbeg)(uint32_t v)
{
    if (env->sregs[LBEG] != v) {
        tb_invalidate_phys_page_range(
                env->sregs[LEND] - 1, env->sregs[LEND], 0);
        env->sregs[LBEG] = v;
    }
}

void HELPER(wsr_lend)(uint32_t v)
{
    if (env->sregs[LEND] != v) {
        tb_invalidate_phys_page_range(
                env->sregs[LEND] - 1, env->sregs[LEND], 0);
        env->sregs[LEND] = v;
        tb_invalidate_phys_page_range(
                env->sregs[LEND] - 1, env->sregs[LEND], 0);
    }
}

void HELPER(dump_state)(void)
{
    cpu_dump_state(env, stderr, fprintf, 0);
}

void HELPER(waiti)(uint32_t pc, uint32_t intlevel)
{
    env->pc = pc;
    env->sregs[PS] = (env->sregs[PS] & ~PS_INTLEVEL) |
        (intlevel << PS_INTLEVEL_SHIFT);
    check_interrupts(env);
    if (env->pending_irq_level) {
        cpu_loop_exit();
        return;
    }

    if (xtensa_option_enabled(env->config, XTENSA_OPTION_TIMER_INTERRUPT)) {
        int i;
        uint32_t wake_ccount = env->sregs[CCOUNT] - 1;

        for (i = 0; i < env->config->nccompare; ++i) {
            if (env->sregs[CCOMPARE + i] - env->sregs[CCOUNT] <
                    wake_ccount - env->sregs[CCOUNT]) {
                wake_ccount = env->sregs[CCOMPARE + i];
            }
        }
        env->wake_ccount = wake_ccount;
        qemu_mod_timer(env->ccompare_timer, qemu_get_clock_ns(vm_clock) +
                muldiv64(wake_ccount - env->sregs[CCOUNT],
                    1000000, env->config->clock_freq_khz));
    }
    env->halt_clock = qemu_get_clock_ns(vm_clock);
    env->halted = 1;
    HELPER(exception)(EXCP_HLT);
}

void HELPER(timer_irq)(uint32_t id, uint32_t active)
{
    xtensa_timer_irq(env, id, active);
}

void HELPER(advance_ccount)(uint32_t d)
{
    xtensa_advance_ccount(env, d);
}

void HELPER(wsr_rasid)(uint32_t v)
{
    qemu_log("%s: %08x\n", __func__, v);
    env->sregs[RASID] = (v & 0xffffff00) | 1;
    /*TODO may count number of used translations to changed rings
     * and flush only if there are some
     */
    tlb_flush(env, 1);
}

static uint32_t get_page_size(const CPUState *env, bool dtlb, uint32_t way)
{
    uint32_t tlbcfg = env->sregs[dtlb ? DTLBCFG : ITLBCFG];

    switch (way) {
    case 4:
        return (tlbcfg >> 16) & 3;

    case 5:
        return (tlbcfg >> 20) & 1;

    case 6:
        return (tlbcfg >> 24) & 1;

    default:
        return 0;
    }
}

/*!
 * Get bit mask for the bits that get translated by the specified TLB way
 */
uint32_t xtensa_tlb_get_addr_mask(const CPUState *env, bool dtlb, uint32_t way)
{
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_MMU)) {
        bool varway56 = dtlb ?
            env->config->dtlb.varway56 :
            env->config->itlb.varway56;

        switch (way) {
        case 4:
            return 0xfff00000 << get_page_size(env, dtlb, way) * 2;

        case 5:
            if (varway56) {
                return 0xf8000000 << get_page_size(env, dtlb, way);
            } else {
                return 0xf8000000;
            }

        case 6:
            if (varway56) {
                return 0xf0000000 << (1 - get_page_size(env, dtlb, way));
            } else {
                return 0xf0000000;
            }

        default:
            return 0xfffff000;
        }
    } else {
        return REGION_PAGE_MASK;
    }
}

/*!
 * Get bitmask for the 'VPN without index' field
 */
static uint32_t get_vpn_mask(const CPUState *env, bool dtlb, uint32_t way)
{
    if (way < 4) {
        bool is32 = (dtlb ?
                env->config->dtlb.nrefillentries :
                env->config->itlb.nrefillentries) == 32;
        return is32 ? 0xffff8000 : 0xffffc000;
    } else if (way == 4) {
        return xtensa_tlb_get_addr_mask(env, dtlb, way) << 2;
    } else if (way <= 6) {
        uint32_t mask = xtensa_tlb_get_addr_mask(env, dtlb, way);
        bool varway56 = dtlb ?
            env->config->dtlb.varway56 :
            env->config->itlb.varway56;

        if (varway56) {
            return mask << (way == 5 ? 2 : 3);
        } else {
            return mask << 1;
        }
    } else {
        return 0xfffff000;
    }
}

void split_tlb_entry_spec_way(const CPUState *env, uint32_t v, bool dtlb,
        uint32_t *vpn, uint32_t wi, uint32_t *ei)
{
    bool varway56 = dtlb ?
        env->config->dtlb.varway56 :
        env->config->itlb.varway56;

    if (!dtlb) {
        wi &= 7;
    }

    if (wi < 4) {
        bool is32 = (dtlb ?
                env->config->dtlb.nrefillentries :
                env->config->itlb.nrefillentries) == 32;
        *ei = (v >> 12) & (is32 ? 7 : 3);
    } else {
        switch (wi) {
        case 4:
            {
                uint32_t eibase = 20 + get_page_size(env, dtlb, wi) * 2;
                *ei = (v >> eibase) & 3;
            }
            break;

        case 5:
            if (varway56) {
                uint32_t eibase = 27 + get_page_size(env, dtlb, wi);
                *ei = (v >> eibase) & 3;
            } else {
                *ei = (v >> 27) & 1;
            }
            break;

        case 6:
            if (varway56) {
                uint32_t eibase = 29 - get_page_size(env, dtlb, wi);
                *ei = (v >> eibase) & 7;
            } else {
                *ei = (v >> 28) & 1;
            }
            break;

        default:
            *ei = 0;
            break;
        }
    }
    *vpn = v & xtensa_tlb_get_addr_mask(env, dtlb, wi);
}

static void split_tlb_entry_spec(uint32_t v, bool dtlb,
        uint32_t *vpn, uint32_t *wi, uint32_t *ei)
{
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_MMU)) {
        *wi = v & (dtlb ? 0xf : 0x7);
        split_tlb_entry_spec_way(env, v, dtlb, vpn, *wi, ei);
    } else {
        *vpn = v & REGION_PAGE_MASK;
        *wi = 0;
        *ei = (v >> 29) & 7;
    }
}

static xtensa_tlb_entry_t *get_tlb_entry(uint32_t v, bool dtlb, uint32_t *_wi)
{
    uint32_t vpn;
    uint32_t wi;
    uint32_t ei;

    split_tlb_entry_spec(v, dtlb, &vpn, &wi, &ei);
    if (_wi) {
        *_wi = wi;
    }
    return xtensa_get_tlb_entry(env, dtlb, wi, ei);
}

uint32_t HELPER(rtlb0)(uint32_t v, uint32_t dtlb)
{
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_MMU)) {
        uint32_t wi;
        const xtensa_tlb_entry_t *entry = get_tlb_entry(v, dtlb, &wi);
        return (entry->vaddr & get_vpn_mask(env, dtlb, wi)) | entry->asid;
    } else {
        return v & REGION_PAGE_MASK;
    }
}

uint32_t HELPER(rtlb1)(uint32_t v, uint32_t dtlb)
{
    const xtensa_tlb_entry_t *entry = get_tlb_entry(v, dtlb, NULL);
    return entry->paddr | entry->attr;
}

void HELPER(itlb)(uint32_t v, uint32_t dtlb)
{
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_MMU)) {
        uint32_t wi;
        xtensa_tlb_entry_t *entry = get_tlb_entry(v, dtlb, &wi);
        if (entry->variable && entry->asid) {
            tlb_flush_page(env, entry->vaddr);
            entry->asid = 0;
        }
    }
}

uint32_t HELPER(ptlb)(uint32_t v, uint32_t dtlb)
{
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_MMU)) {
        uint32_t wi;
        uint32_t ei;
        uint8_t ring;
        int res = xtensa_tlb_lookup(env, v, dtlb, &wi, &ei, &ring);

        switch (res) {
        case 0:
            if (ring >= xtensa_get_ring(env)) {
                return (v & 0xfffff000) | wi | (dtlb ? 16 : 8);
            }
            break;

        case INST_TLB_MULTI_HIT_CAUSE:
        case LOAD_STORE_TLB_MULTI_HIT_CAUSE:
            HELPER(exception_cause_vaddr)(env->pc, res, v);
            break;
        }
        return 0;
    } else {
        return (v & REGION_PAGE_MASK) | 1;
    }
}

void xtensa_tlb_set_entry(CPUState *env, bool dtlb,
        unsigned wi, unsigned ei, uint32_t vpn, uint32_t pte)
{
    xtensa_tlb_entry_t *entry = xtensa_get_tlb_entry(env, dtlb, wi, ei);

    if (xtensa_option_enabled(env->config, XTENSA_OPTION_MMU)) {
        if (entry->variable) {
            if (entry->asid) {
                tlb_flush_page(env, entry->vaddr);
            }
            entry->vaddr = vpn;
            entry->paddr = pte & xtensa_tlb_get_addr_mask(env, dtlb, wi);
            entry->asid = (env->sregs[RASID] >> ((pte >> 1) & 24)) & 0xff;
            entry->attr = pte & 0xf;
        } else {
            qemu_log("%s %d, %d, %d trying to set immutable entry\n",
                    __func__, dtlb, wi, ei);
        }
    } else {
        tlb_flush_page(env, entry->vaddr);
        entry->vaddr = vpn;
        if (xtensa_option_enabled(env->config,
                    XTENSA_OPTION_REGION_TRANSLATION)) {
            entry->paddr = pte & REGION_PAGE_MASK;
        } else {
            entry->paddr = vpn;
        }
        entry->attr = pte & 0xf;
    }
}

void HELPER(wtlb)(uint32_t p, uint32_t v, uint32_t dtlb)
{
    uint32_t vpn;
    uint32_t wi;
    uint32_t ei;
    split_tlb_entry_spec(v, dtlb, &vpn, &wi, &ei);
    xtensa_tlb_set_entry(env, dtlb, wi, ei, vpn, p);
}
