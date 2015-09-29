#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-de212/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig de212 __attribute__((unused)) = {
    .name = "de212",
    .gdb_regmap = {
        .reg = {
#include "core-de212/gdb-config.c"
        }
    },
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(de212)
