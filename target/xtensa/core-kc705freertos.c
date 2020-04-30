#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-kc705freertos/core-isa.h"
#include "core-kc705freertos/core-matmap.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_kc705freertos
#include "core-kc705freertos/xtensa-modules.c.inc"

static XtensaConfig kc705freertos __attribute__((unused)) = {
    .name = "kc705freertos",
    .gdb_regmap = {
        .reg = {
#include "core-kc705freertos/gdb-config.c.inc"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS,
    .sysrom.num = 0,
    .sysram.location[0].addr = 0x90000000,
};

REGISTER_CORE(kc705freertos)
