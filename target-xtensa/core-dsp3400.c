#include "cpu.h"
#include "exec-all.h"
#include "gdbstub.h"
#include "qemu-common.h"
#include "host-utils.h"

#include "core-dsp3400/core-isa.h"
#include "overlay_tool.h"

static const XtensaConfig dsp3400 = {
    .name = "dsp3400",
    .options = XTENSA_OPTIONS,
    .gdb_regmap = {
        .num_regs = 231,
        .num_core_regs = 172,
        .reg = {
#include "core-dsp3400/gdb-config.c"
        }
    },
    .nareg = XCHAL_NUM_AREGS,
    .ndepc = 1,
    EXCEPTIONS_SECTION,
    INTERRUPTS_SECTION,
    TLB_SECTION,
    .clock_freq_khz = 10000,
};

REGISTER_CORE(dsp3400)
