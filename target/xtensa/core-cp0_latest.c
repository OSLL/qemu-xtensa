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
    UR_W = 16,
    UR_MY = 32,
    UR_ACCUM = 40,
    UR_RESULT = 42,
    UR_SS = 57,
    MSB_OFFSET = 37,
    DST_RIB_T2_PTR = 49,
    SRC_RIB_T2_PTR = 50,
};

static const XtensaOpcodeOps cp0_ops[] = {
    {
        .name = "rur.accum_0",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_ACCUM + 0},
        .coprocessor = 0x10,
    }, {
        .name = "rur.accum_1",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_ACCUM + 1},
        .coprocessor = 0x10,
    }, {
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
        .name = "rur.mya",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_MY + 0},
        .coprocessor = 0x10,
    }, {
        .name = "rur.myb",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_MY + 1},
        .coprocessor = 0x10,
    }, {
        .name = "rur.myc",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_MY + 2},
        .coprocessor = 0x10,
    }, {
        .name = "rur.myd",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_MY + 3},
        .coprocessor = 0x10,
    }, {
        .name = "rur.mye",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_MY + 4},
        .coprocessor = 0x10,
    }, {
        .name = "rur.output_rate",
        .translate = translate_rur,
        .par = (const uint32_t[]){OUTPUT_RATE},
    }, {
        .name = "rur.result_0",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_RESULT + 0},
        .coprocessor = 0x10,
    }, {
        .name = "rur.result_1",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_RESULT + 1},
        .coprocessor = 0x10,
    }, {
        .name = "rur.src_rib_t2_ptr",
        .translate = translate_rur,
        .par = (const uint32_t[]){SRC_RIB_T2_PTR},
    }, {
        .name = "rur.ss",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_SS},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w0",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 0},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w1",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 1},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w10",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 10},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w11",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 11},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w12",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 12},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w13",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 13},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w14",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 14},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w15",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 15},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w2",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 2},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w3",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 3},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w4",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 4},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w5",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 5},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w6",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 6},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w7",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 7},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w8",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 8},
        .coprocessor = 0x10,
    }, {
        .name = "rur.w9",
        .translate = translate_rur,
        .par = (const uint32_t[]){UR_W + 9},
        .coprocessor = 0x10,
    }, {
        .name = "rur.weight_red",
        .translate = translate_rur,
        .par = (const uint32_t[]){WEIGHT_RED},
    }, {
        .name = "wur.accum_0",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_ACCUM + 0},
        .coprocessor = 0x10,
    }, {
        .name = "wur.accum_1",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_ACCUM + 1},
        .coprocessor = 0x10,
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
        .name = "wur.mya",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_MY + 0},
        .coprocessor = 0x10,
    }, {
        .name = "wur.myb",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_MY + 1},
        .coprocessor = 0x10,
    }, {
        .name = "wur.myc",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_MY + 2},
        .coprocessor = 0x10,
    }, {
        .name = "wur.myd",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_MY + 3},
        .coprocessor = 0x10,
    }, {
        .name = "wur.mye",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_MY + 4},
        .coprocessor = 0x10,
    }, {
        .name = "wur.output_rate",
        .translate = translate_wur,
        .par = (const uint32_t[]){OUTPUT_RATE},
    }, {
        .name = "wur.result_0",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_RESULT + 0},
        .coprocessor = 0x10,
    }, {
        .name = "wur.result_1",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_RESULT + 1},
        .coprocessor = 0x10,
    }, {
        .name = "wur.src_rib_t2_ptr",
        .translate = translate_wur,
        .par = (const uint32_t[]){SRC_RIB_T2_PTR},
    }, {
        .name = "wur.ss",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_SS},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w0",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 0},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w1",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 1},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w10",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 10},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w11",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 11},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w12",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 12},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w13",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 13},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w14",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 14},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w15",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 15},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w2",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 2},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w3",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 3},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w4",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 4},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w5",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 5},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w6",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 6},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w7",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 7},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w8",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 8},
        .coprocessor = 0x10,
    }, {
        .name = "wur.w9",
        .translate = translate_wur,
        .par = (const uint32_t[]){UR_W + 9},
        .coprocessor = 0x10,
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
    .clock_freq_khz = 10000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(cp0_latest)
