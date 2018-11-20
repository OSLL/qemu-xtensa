#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-cp0_latest/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_cp0_latest
#include "core-cp0_latest/xtensa-modules.inc.c"

static XtensaConfig cp0_latest __attribute__((unused)) = {
    .name = "cp0_latest",
    .gdb_regmap = {
        .reg = {
#include "core-cp0_latest/gdb-config.inc.c"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 600000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(cp0_latest)
