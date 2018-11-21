#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/gdbstub.h"
#include "qemu-common.h"
#include "qemu/host-utils.h"

#include "core-cp0_latest/core-isa.h"
#include "overlay_tool.h"

#define xtensa_modules xtensa_modules_cp0_latest
#include "core-cp0_latest/xtensa-modules.inc.c"

enum {
    WEIGHT_RED = 8,
    OUTPUT_RATE = 9,
    LOOK_UP_MAX_INDEX = 10,
    MSB_OFFSET = 37,
    DST_RIB_T2_PTR = 49,
    SRC_RIB_T2_PTR = 50,
};

static const XtensaOpcodeOps cp0_ops[] = {
    {
        .name = "rur.dst_rib_t2_ptr",
        .translate = translate_rur,
        .par = (const uint32_t[]){DST_RIB_T2_PTR},
    }, {
        .name = "rur.look_up_max_index",
        .translate = translate_rur,
        .par = (const uint32_t[]){LOOK_UP_MAX_INDEX},
    }, {
        .name = "rur.msb_offset",
        .translate = translate_rur,
        .par = (const uint32_t[]){MSB_OFFSET},
    }, {
        .name = "rur.output_rate",
        .translate = translate_rur,
        .par = (const uint32_t[]){OUTPUT_RATE},
    }, {
        .name = "rur.src_rib_t2_ptr",
        .translate = translate_rur,
        .par = (const uint32_t[]){SRC_RIB_T2_PTR},
    }, {
        .name = "rur.weight_red",
        .translate = translate_rur,
        .par = (const uint32_t[]){WEIGHT_RED},
    }, {
        .name = "wur.dst_rib_t2_ptr",
        .translate = translate_wur,
        .par = (const uint32_t[]){DST_RIB_T2_PTR},
    }, {
        .name = "wur.look_up_max_index",
        .translate = translate_wur,
        .par = (const uint32_t[]){LOOK_UP_MAX_INDEX},
    }, {
        .name = "wur.msb_offset",
        .translate = translate_wur,
        .par = (const uint32_t[]){MSB_OFFSET},
    }, {
        .name = "wur.output_rate",
        .translate = translate_wur,
        .par = (const uint32_t[]){OUTPUT_RATE},
    }, {
        .name = "wur.src_rib_t2_ptr",
        .translate = translate_wur,
        .par = (const uint32_t[]){SRC_RIB_T2_PTR},
    }, {
        .name = "wur.weight_red",
        .translate = translate_wur,
        .par = (const uint32_t[]){WEIGHT_RED},
    }
};

static const XtensaOpcodeTranslators cp0_opcodes = {
    .num_opcodes = ARRAY_SIZE(cp0_ops),
    .opcode = cp0_ops,
};

static XtensaConfig cp0_latest __attribute__((unused)) = {
    .name = "cp0_latest",
    .gdb_regmap = {
        .reg = {
#include "core-cp0_latest/gdb-config.inc.c"
        }
    },
    .isa_internal = &xtensa_modules,
    .opcode_translators = (const XtensaOpcodeTranslators *[]){
        &cp0_opcodes,
        &xtensa_core_opcodes,
        NULL,
    },
    .clock_freq_khz = 600000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(cp0_latest)
