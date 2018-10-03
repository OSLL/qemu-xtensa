#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-de570t/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_de570t
#include "core-de570t/xtensa-modules.c.inc"

static XtensaConfig de570t __attribute__((unused)) = {
    .name = "de570t",
    .gdb_regmap = {
        .reg = {
#include "core-de570t/gdb-config.c.inc"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(de570t)
