#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-cpu3400/core-isa.h"
#include "overlay_tool.h"

static const XtensaConfig cpu3400 = {
    .name = "cpu3400",
    .gdb_regmap = {
        .num_regs = 176,
        .num_core_regs = 117,
        .reg = {
#include "core-cpu3400/gdb-config.c"
        }
    },
    .clock_freq_khz = 912000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(cpu3400)
