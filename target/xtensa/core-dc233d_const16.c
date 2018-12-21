#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-dc233d_const16/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_dc233d_const16
#include "core-dc233d_const16/xtensa-modules.inc.c"

static XtensaConfig dc233d_const16 __attribute__((unused)) = {
    .name = "dc233d_const16",
    .gdb_regmap = {
        .reg = {
#include "core-dc233d_const16/gdb-config.inc.c"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(dc233d_const16)
