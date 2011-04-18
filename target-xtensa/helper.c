/*
 * Copyright (c) 2011, Max Filippov, Motorola Solutions, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Motorola Solutions nor the
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

#include "cpu.h"
#include "exec-all.h"
#include "gdbstub.h"
#include "qemu-common.h"
#include "host-utils.h"
#if !defined(CONFIG_USER_ONLY)
#include "hw/loader.h"
#endif

void cpu_reset(CPUXtensaState *env)
{
    env->exception_taken = 0;
    env->pc = 0;
    env->sregs[PS] = 0x1f;
}

CPUXtensaState *cpu_xtensa_init(const char *cpu_model)
{
    static int tcg_inited;
    CPUXtensaState *env;

    env = qemu_mallocz(sizeof(*env));
    cpu_exec_init(env);

    if (!tcg_inited) {
        tcg_inited = 1;
        xtensa_translate_init();
    }

    cpu_reset(env);
    qemu_init_vcpu(env);
    return env;
}


void xtensa_cpu_list(FILE *f, fprintf_function cpu_fprintf)
{
    cpu_fprintf(f, "Available CPUs:\n"
            "  Xtensa core\n");
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
    return addr;
}

void do_interrupt(CPUState *env)
{
    static const uint32_t vector[] = {
        [EXC_WINDOW_OVERFLOW4] = WINDOW_OVERFLOW4,
        [EXC_WINDOW_UNDERFLOW4] = WINDOW_UNDERFLOW4,
        [EXC_WINDOW_OVERFLOW8] = WINDOW_OVERFLOW8,
        [EXC_WINDOW_UNDERFLOW8] = WINDOW_UNDERFLOW8,
        [EXC_WINDOW_OVERFLOW12] = WINDOW_OVERFLOW12,
        [EXC_WINDOW_UNDERFLOW12] = WINDOW_UNDERFLOW12,
        [EXC_KERNEL] = KERNEL_EXCEPTION_VECTOR,
        [EXC_USER] = USER_EXCEPTION_VECTOR,
        [EXC_DOUBLE] = DOUBLE_EXCEPTION_VECTOR,
    };

    switch (env->exception_index) {
    case EXC_WINDOW_OVERFLOW4:
    case EXC_WINDOW_UNDERFLOW4:
    case EXC_WINDOW_OVERFLOW8:
    case EXC_WINDOW_UNDERFLOW8:
    case EXC_WINDOW_OVERFLOW12:
    case EXC_WINDOW_UNDERFLOW12:
    case EXC_KERNEL:
    case EXC_USER:
    case EXC_DOUBLE:
        env->pc = vector[env->exception_index];
        env->exception_taken = 1;
        break;

    }
    env->interrupt_request |= CPU_INTERRUPT_EXITTB;
}
