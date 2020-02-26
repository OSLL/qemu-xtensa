#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-hifi4_bd5/core-isa.h"
#include "core-hifi4_bd5/core-matmap.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_hifi4_bd5
#include "core-hifi4_bd5/xtensa-modules.c.inc"

static XtensaConfig hifi4_bd5 __attribute__((unused)) = {
    .name = "hifi4_bd5",
    .gdb_regmap = {
        .reg = {
#include "core-hifi4_bd5/gdb-config.c.inc"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(hifi4_bd5)
