#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-lx106/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig lx106 __attribute__((unused)) = {
    .name = "lx106",
    .gdb_regmap = {
        .num_regs = 50,
        .reg = {
#include "core-lx106/gdb-config.c"
        }
    },
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(lx106)
