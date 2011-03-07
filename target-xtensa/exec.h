#include "config.h"
#include "dyngen-exec.h"

register struct CPUXtensaState *env asm(AREG0);

#include "cpu.h"
#include "exec-all.h"

static inline int cpu_has_work(CPUState *env)
{
    return 0;
}

static inline int cpu_halted(CPUState *env)
{
    return 0;
}

#if !defined(CONFIG_USER_ONLY)
#include "softmmu_exec.h"
#endif

void raise_exception(int);

static inline void cpu_pc_from_tb(CPUState *env, TranslationBlock *tb)
{
    env->pc = tb->pc;
}

