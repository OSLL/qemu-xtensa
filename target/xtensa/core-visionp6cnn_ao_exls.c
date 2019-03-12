#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-visionp6cnn_ao_exls/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_visionp6cnn_ao_exls
#include "core-visionp6cnn_ao_exls/xtensa-modules.c.inc"

static XtensaConfig visionp6cnn_ao_exls __attribute__((unused)) = {
    .name = "visionp6cnn_ao_exls",
    .gdb_regmap = {
        .reg = {
#include "core-visionp6cnn_ao_exls/gdb-config.c.inc"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 10000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(visionp6cnn_ao_exls)
