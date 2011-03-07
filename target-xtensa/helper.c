#include "cpu.h"
#include "exec-all.h"
#include "gdbstub.h"
//#include "helpers.h"
#include "qemu-common.h"
#include "host-utils.h"
#if !defined(CONFIG_USER_ONLY)
#include "hw/loader.h"
#endif

void cpu_reset(CPUXtensaState *env)
{
    env->pc = 0;
}

CPUXtensaState *cpu_xtensa_init(const char *cpu_model)
{
    static int tcg_inited = 0;
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
    cpu_fprintf(f, "Avilable CPUs:\n"
            "  Xtensa core\n");
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
    return addr;
}

void do_interrupt(CPUState *env)
{
}
