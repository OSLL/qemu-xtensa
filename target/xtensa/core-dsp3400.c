#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-dsp3400/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig dsp3400 __attribute__((unused)) = {
    .name = "dsp3400",
    .gdb_regmap = {
        .num_regs = 231,
        .num_core_regs = 172,
        .reg = {
#include "core-dsp3400/gdb-config.c"
        }
    },
    .clock_freq_khz = 10000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(dsp3400)
