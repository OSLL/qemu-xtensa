#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-test_kc705_perf/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_test_kc705_perf
#include "core-test_kc705_perf/xtensa-modules.c.inc"

static XtensaConfig test_kc705_perf __attribute__((unused)) = {
    .name = "test_kc705_perf",
    .gdb_regmap = {
        .reg = {
#include "core-test_kc705_perf/gdb-config.c.inc"
        }
    },
    .isa_internal = &xtensa_modules,
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(test_kc705_perf)
