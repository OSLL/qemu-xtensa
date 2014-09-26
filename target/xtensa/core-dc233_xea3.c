#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-dc233_xea3/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig dc233_xea3 __attribute__((unused)) = {
    .name = "dc233_xea3",
    .gdb_regmap = {
        .num_regs = 124,
        .reg = {
#include "core-dc233_xea3/gdb-config.c"
        }
    },
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(dc233_xea3)
