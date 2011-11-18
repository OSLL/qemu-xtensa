#include "cpu.h"
#include "exec-all.h"
#include "gdbstub.h"
#include "qemu-common.h"
#include "host-utils.h"

#include "core-cpu3400/core-isa.h"
#include "overlay_tool.h"

static const XtensaConfig cpu3400 = {
    .name = "cpu3400",
    .options = XTENSA_OPTIONS,
    .gdb_regmap = {
        .num_regs = 176,
        .num_core_regs = 117,
        .reg = {
#include "core-cpu3400/gdb-config.c"
        }
    },
    .nareg = XCHAL_NUM_AREGS,
    .ndepc = 1,
    EXCEPTIONS_SECTION,
    INTERRUPTS_SECTION,
    TLB_SECTION,
    .clock_freq_khz = 912000,
};

REGISTER_CORE(cpu3400)
