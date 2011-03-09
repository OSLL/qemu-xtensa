#include "exec.h"
#include "helpers.h"

#define MMUSUFFIX _mmu

#define SHIFT 0
#include "softmmu_template.h"

#define SHIFT 1
#include "softmmu_template.h"

#define SHIFT 2
#include "softmmu_template.h"

#define SHIFT 3
#include "softmmu_template.h"

void tlb_fill (target_ulong addr, int is_write, int mmu_idx, void *retaddr)
{
    tlb_set_page(cpu_single_env,
            addr & ~(TARGET_PAGE_SIZE - 1),
            addr & ~(TARGET_PAGE_SIZE - 1),
            PAGE_READ | PAGE_WRITE | PAGE_EXEC,
            mmu_idx, TARGET_PAGE_SIZE);
}

void HELPER(exception)(uint32_t excp)
{
    env->exception_index = excp;
    cpu_loop_exit();
}
