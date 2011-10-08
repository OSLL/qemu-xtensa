#include "cpu.h"
#include "exec-all.h"
#include "gdbstub.h"
#include "qemu-common.h"
#include "host-utils.h"

#include "core-fsf/core-isa.h"
#include "overlay_tool.h"

static const XtensaConfig fsf = {
    .name = "fsf",
    .options = XTENSA_OPTIONS,
    .gdb_regmap = {
        .num_regs = 130,
        .num_core_regs = 75,
        .reg = {
#include "core-fsf/gdb-config.c"
        }
    },
    .nareg = XCHAL_NUM_AREGS,
    .ndepc = 1,
    EXCEPTIONS_SECTION,
    INTERRUPTS_SECTION,
    TLB_SECTION,
    .clock_freq_khz = 10000,
};

REGISTER_CORE(fsf)
