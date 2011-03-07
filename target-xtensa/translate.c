#include <stdio.h>

#include "cpu.h"
#include "exec-all.h"
#include "disas.h"
#include "tcg-op.h"
#include "qemu-log.h"


void xtensa_translate_init(void)
{
}

void gen_intermediate_code(CPUState *env, TranslationBlock *tb)
{
}

void gen_intermediate_code_pc(CPUState *env, TranslationBlock *tb)
{
}

void cpu_dump_state(CPUState *env, FILE *f, fprintf_function cpu_fprintf,
        int flags)
{
    int i;

    cpu_fprintf(f, "PC=%08x\n", env->pc);

    for (i = 0; i < 16; ++i)
        cpu_fprintf(f, "AR%02d=%08x%c", i, env->regs[i],
                (i % 4) == 3 ? '\n' : ' ');
}

void gen_pc_load(CPUState *env, TranslationBlock *tb,
        unsigned long searched_pc, int pc_pos, void *puc)
{
}
