#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-de108/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig de108 __attribute__((unused)) = {
    .name = "de108",
    .gdb_regmap = {
        .reg = {
#include "core-de108/gdb-config.c"
        }
    },
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(de108)
