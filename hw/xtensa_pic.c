#include "hw.h"
#include "pc.h"

/* Stub functions for hardware that doesn't exist.  */
void pic_info(Monitor *mon)
{
}

void irq_info(Monitor *mon)
{
}

void check_interrupts(CPUState *env)
{
    int minlevel = xtensa_get_cintlevel(env);
    int level;

    for (level = env->config->nlevel; level > minlevel; --level) {
        if (env->config->level_mask[level] &
                env->sregs[INTSET] &
                env->sregs[INTENABLE]) {
            env->pending_irq_level = level;
            cpu_interrupt(env, CPU_INTERRUPT_HARD);
            return;
        }
    }
    env->pending_irq_level = 0;
    cpu_reset_interrupt(env, CPU_INTERRUPT_HARD);
}

static void xtensa_set_irq(void *opaque, int irq, int active)
{
    CPUState *env = opaque;

    if (irq >= env->config->ninterrupt) {
        printf("%s: bad IRQ %d\n", __func__, irq);
    } else {
        uint32_t irq_bit = 1 << irq;

        if (active) {
            env->sregs[INTSET] |= irq_bit;
        } else {
            env->sregs[INTSET] &= ~irq_bit;
        }

        check_interrupts(env);
    }
}

void xtensa_irq_init(CPUState *env)
{
    env->irq_inputs = (void **)qemu_allocate_irqs(
            xtensa_set_irq, env, env->config->ninterrupt);
}
