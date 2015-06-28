#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-test_kc705_perf/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig test_kc705_perf __attribute__((unused)) = {
    .name = "test_kc705_perf",
    .gdb_regmap = {
        .num_regs = 148,
        .reg = {
#include "core-test_kc705_perf/gdb-config.c"
        }
    },
    .clock_freq_khz = 50000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(test_kc705_perf)
