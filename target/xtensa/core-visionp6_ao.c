#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-visionp6_ao/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_visionp6_ao
#include "core-visionp6_ao/xtensa-modules.inc.c"

static XtensaConfig visionp6_ao __attribute__((unused)) = {
    .name = "visionp6_ao",
    .gdb_regmap = {
        .reg = {
#include "core-visionp6_ao/gdb-config.inc.c"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 10000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(visionp6_ao)
