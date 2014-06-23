#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-test_kc705_be/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig test_kc705_be __attribute__((unused)) = {
    .name = "test_kc705_be",
    .gdb_regmap = {
        .reg = {
#include "core-test_kc705_be/gdb-config.c"
        }
    },
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(test_kc705_be)
