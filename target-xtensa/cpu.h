#ifndef CPU_XTENSA_H
#define CPU_XTENSA_H

#define TARGET_LONG_BITS 32
#define ELF_MACHINE EM_XTENSA

#define CPUState struct CPUXtensaState

#include "config.h"
#include "qemu-common.h"
#include "cpu-defs.h"

#define TARGET_HAS_ICE 1

#define NB_MMU_MODES 2

#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 32
#define TARGET_PAGE_BITS 12

typedef struct CPUXtensaState {
    uint32_t regs[16];
    uint32_t pc;
    uint32_t sregs[256];

    CPU_COMMON
} CPUXtensaState;

#define cpu_init cpu_xtensa_init
#define cpu_exec cpu_xtensa_exec
#define cpu_gen_code cpu_xtensa_gen_code
#define cpu_signal_handler cpu_xtensa_signal_handler
#define cpu_list xtensa_cpu_list

CPUXtensaState *cpu_xtensa_init(const char *cpu_model);
void xtensa_translate_init(void);
int cpu_xtensa_exec(CPUXtensaState *s);
void do_interrupt(CPUXtensaState *s);
int cpu_xtensa_signal_handler(int host_signum, void *pinfo, void *puc);
void xtensa_cpu_list(FILE *f, fprintf_function cpu_fprintf);

static inline int cpu_mmu_index(CPUState *env)
{
    return 0;
}

static inline void cpu_get_tb_cpu_state(CPUState *env, target_ulong *pc,
        target_ulong *cs_base, int *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0;
}

#include "cpu-all.h"

#endif
