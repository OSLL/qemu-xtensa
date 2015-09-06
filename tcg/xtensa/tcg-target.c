/*
 * Tiny Code Generator for QEMU
 *
 * Copyright (c) 2015 Max Filippov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "core-isa.h"
#include "tcg-be-ldst.h"

#ifndef XCHAL_HAVE_CONST16
#define XCHAL_HAVE_CONST16 0
#endif

#if TARGET_LONG_BITS == 64
#define LO_OFF    (XCHAL_HAVE_BE * 4)
#define HI_OFF    (4 - LO_OFF)
#else
#define LO_OFF    0
#define HI_OFF    0
#endif

static tcg_insn_unit *tb_ret_addr;

#ifndef NDEBUG
static const char * const tcg_target_reg_names[TCG_TARGET_NB_REGS] = {
    "a0",
    "a1",
    "a2",
    "a3",
    "a4",
    "a5",
    "a6",
    "a7",
    "a8",
    "a9",
    "a10",
    "a11",
    "a12",
    "a13",
    "a14",
    "a15",
};
#endif

#ifndef CONFIG_SOFTMMU
# define TCG_GUEST_BASE_REG TCG_REG_I5
#endif

static const int tcg_target_reg_alloc_order[] = {
    TCG_REG_A2,
    TCG_REG_A3,
    TCG_REG_A4,
    TCG_REG_A5,
    TCG_REG_A6,
    TCG_REG_A7,
    TCG_REG_A15,
    TCG_REG_A14,
    TCG_REG_A13,
    TCG_REG_A12,
    TCG_REG_A11,
    TCG_REG_A10,
    TCG_REG_A9,
    TCG_REG_A8,
};

static const int tcg_target_call_iarg_regs[6] = {
    TCG_REG_A10,
    TCG_REG_A11,
    TCG_REG_A12,
    TCG_REG_A13,
    TCG_REG_A14,
    TCG_REG_A15,
};

static const int tcg_target_call_oarg_regs[] = {
    TCG_REG_A10,
    TCG_REG_A11,
    TCG_REG_A12,
    TCG_REG_A13,
};

enum {
    R_XTENSA_PCREL8,
    R_XTENSA_PCREL8_18,
    R_XTENSA_PCREL18,
};

enum {
    ADD_OP0 = 0x0,
    ADD_OP1 = 0x0,
    ADD_OP2 = 0x8,

    ADDI_OP0 = 0x2,
    ADDI_R = 0xc,

    ADDMI_OP0 = 0x2,
    ADDMI_R = 0xd,

    AND_OP0 = 0x0,
    AND_OP1 = 0x0,
    AND_OP2 = 0x1,

    BANY_OP0 = 0x7,
    BANY_R = 0x8,
    BEQ_R = 0x1,
    BGE_R = 0xa,
    BGEU_R = 0xb,
    BLT_R = 0x2,
    BLTU_R = 0x3,
    BNE_R = 0x9,

    CALL8_OP0 = 0x5,
    CALL8_N = 0x2,

    CALLX8_OP0 = 0x0,
    CALLX8_OP1 = 0x0,
    CALLX8_OP2 = 0x0,
    CALLX8_N = 0x2,
    CALLX8_M = 0x3,
    CALLX8_R = 0x0,

    ENTRY_OP0 = 0x6,
    ENTRY_N = 0x3,
    ENTRY_M = 0x0,

    EXTUI_OP0 = 0x0,
    EXTUI_OP1 = 0x4,

    J_OP0 = 0x6,
    J_N = 0,

    JX_OP0 = 0x0,
    JX_OP1 = 0x0,
    JX_OP2 = 0x0,
    JX_N = 0x2,
    JX_M = 0x2,
    JX_R = 0x0,

    L16SI_R = 0x9,
    L16UI_R = 0x1,
    L32I_OP0 = 0x2,
    L32I_R = 0x2,
    L8UI_R = 0x0,

    MOV_OP0 = 0x0,
    MOV_OP1 = 0x0,
    MOV_OP2 = 0x2,

    MOVGEZ_OP0 = 0x0,
    MOVGEZ_OP1 = 0x3,
    MOVGEZ_OP2 = 0xb,

    MOVI_OP0 = 0x2,
    MOVI_R = 0xa,

    MUL16S_OP0 = 0x0,
    MUL16S_OP1 = 0x1,
    MUL16S_OP2 = 0xd,

    MUL16U_OP0 = 0x0,
    MUL16U_OP1 = 0x1,
    MUL16U_OP2 = 0xc,

    MULL_OP0 = 0x0,
    MULL_OP1 = 0x2,
    MULL_OP2 = 0x8,

    MULSH_OP0 = 0x0,
    MULSH_OP1 = 0x2,
    MULSH_OP2 = 0xb,

    MULUH_OP0 = 0x0,
    MULUH_OP1 = 0x2,
    MULUH_OP2 = 0xa,

    NEG_OP0 = 0x0,
    NEG_OP1 = 0x0,
    NEG_OP2 = 0x6,

    NOP_OP0 = 0x0,
    NOP_OP1 = 0x0,
    NOP_OP2 = 0x0,
    NOP_R = 0x2,
    NOP_S = 0x0,
    NOP_T = 0xf,

    OR_OP0 = 0x0,
    OR_OP1 = 0x0,
    OR_OP2 = 0x2,

    QUOS_OP0 = 0x0,
    QUOS_OP1 = 0x2,
    QUOS_OP2 = 0xd,

    QUOU_OP0 = 0x0,
    QUOU_OP1 = 0x2,
    QUOU_OP2 = 0xc,

    REMS_OP0 = 0x0,
    REMS_OP1 = 0x2,
    REMS_OP2 = 0xf,

    REMU_OP0 = 0x0,
    REMU_OP1 = 0x2,
    REMU_OP2 = 0xe,

    RETW_OP0 = 0x0,
    RETW_OP1 = 0x0,
    RETW_OP2 = 0x0,
    RETW_N = 0x1,
    RETW_M = 0x2,
    RETW_R = 0x0,
    RETW_S = 0x0,

    S16I_R = 0x5,
    S32I_R = 0x6,
    S8I_R = 0x4,

    SEXT_OP0 = 0x0,
    SEXT_OP1 = 0x3,
    SEXT_OP2 = 0x2,

    SLL_OP0 = 0x0,
    SLL_OP1 = 0x1,
    SLL_OP2 = 0xa,
    SLL_T = 0x0,

    SLLI_OP0 = 0x0,
    SLLI_OP1 = 0x1,
    SLLI_OP2 = 0x0,

    SRA_OP0 = 0x0,
    SRA_OP1 = 0x1,
    SRA_OP2 = 0xb,
    SRA_S = 0x0,

    SRAI_OP0 = 0x0,
    SRAI_OP1 = 0x1,
    SRAI_OP2 = 0x2,

    SRC_OP0 = 0x0,
    SRC_OP1 = 0x1,
    SRC_OP2 = 0x8,

    SRL_OP0 = 0x0,
    SRL_OP1 = 0x1,
    SRL_OP2 = 0x9,
    SRL_S = 0x0,

    SRLI_OP0 = 0x0,
    SRLI_OP1 = 0x1,
    SRLI_OP2 = 0x4,

    SSAI_OP0 = 0x0,
    SSAI_OP1 = 0x0,
    SSAI_OP2 = 0x4,
    SSAI_R = 0x4,

    SSL_OP0 = 0x0,
    SSL_OP1 = 0x0,
    SSL_OP2 = 0x4,
    SSL_R = 0x1,
    SSL_T = 0x0,

    SSR_OP0 = 0x0,
    SSR_OP1 = 0x0,
    SSR_OP2 = 0x4,
    SSR_R = 0x0,
    SSR_T = 0x0,

    SUB_OP0 = 0x0,
    SUB_OP1 = 0x0,
    SUB_OP2 = 0xc,

    XOR_OP0 = 0x0,
    XOR_OP1 = 0x0,
    XOR_OP2 = 0x3,
};

#if XCHAL_HAVE_BE
#define XTENSA_TCG_BITS(v, width, start, length) \
    (((v) >> ((width) - (start) - (length))) & ((1u << length) - 1))
#define XTENSA_TCG_BYTE(low, high) ((high) | ((low) << 4))
#define XTENSA_TCG_NIBBLE(low, high) ((high) | ((low) << 2))
#else
#define XTENSA_TCG_BITS(v, width, start, length) \
    (((v) >> (start)) & ((1u << length) - 1))
#define XTENSA_TCG_BYTE(low, high) ((low) | ((high) << 4))
#define XTENSA_TCG_NIBBLE(low, high) ((low) | ((high) << 2))
#endif

static void tcg_out_rrr(TCGContext *c, uint8_t op0, uint8_t op1, uint8_t op2,
                        uint8_t r, uint8_t s, uint8_t t)
{
    tcg_out8(c, XTENSA_TCG_BYTE(op0, t));
    tcg_out8(c, XTENSA_TCG_BYTE(s, r));
    tcg_out8(c, XTENSA_TCG_BYTE(op1, op2));
}

static void tcg_out_rri8(TCGContext *c, uint8_t op0,
                         uint8_t r, uint8_t s, uint8_t t, uint8_t imm8)
{
    tcg_out8(c, XTENSA_TCG_BYTE(op0, t));
    tcg_out8(c, XTENSA_TCG_BYTE(s, r));
    tcg_out8(c, imm8);
}

static void tcg_out_bri12(TCGContext *c, uint8_t op0, uint8_t n, uint8_t m,
                          uint8_t s, uint16_t imm12)
{
    tcg_out8(c, XTENSA_TCG_BYTE(op0, XTENSA_TCG_NIBBLE(n, m)));
    tcg_out8(c, XTENSA_TCG_BYTE(s, XTENSA_TCG_BITS(imm12, 12, 0, 4)));
    tcg_out8(c, XTENSA_TCG_BITS(imm12, 12, 4, 8));
}

static void tcg_out_call_fmt(TCGContext *c, uint8_t op0, uint8_t n,
                             uint32_t offset)
{
    tcg_out8(c, XTENSA_TCG_BYTE(op0, XTENSA_TCG_NIBBLE(n, XTENSA_TCG_BITS(offset, 18, 0, 2))));
    tcg_out8(c, XTENSA_TCG_BITS(offset, 18, 2, 8));
    tcg_out8(c, XTENSA_TCG_BITS(offset, 18, 10, 8));
}

static void tcg_out_callx(TCGContext *c, uint8_t op0, uint8_t op1, uint8_t op2,
                          uint8_t n, uint8_t m, uint8_t r, uint8_t s)
{
    tcg_out8(c, XTENSA_TCG_BYTE(op0, XTENSA_TCG_NIBBLE(n, m)));
    tcg_out8(c, XTENSA_TCG_BYTE(s, r));
    tcg_out8(c, XTENSA_TCG_BYTE(op1, op2));
}

static void tcg_out_nop(TCGContext *c)
{
    tcg_out_rrr(c, NOP_OP0, NOP_OP1, NOP_OP2, NOP_R, NOP_S, NOP_T);
}

static void tcg_out_add(TCGContext *c, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(c, ADD_OP0, ADD_OP1, ADD_OP2, ret, arg1, arg2);
}

static void tcg_out_addi(TCGContext *c, TCGReg ret, TCGReg arg1, uint8_t arg2)
{
    if (arg2 || ret != arg1) {
        tcg_out_rri8(c, ADDI_OP0, ADDI_R, arg1, ret, arg2);
    }
}

static void tcg_out_addmi(TCGContext *c, TCGReg ret, TCGReg arg1, uint8_t arg2)
{
    if (arg2 || ret != arg1) {
        tcg_out_rri8(c, ADDMI_OP0, ADDMI_R, arg1, ret, arg2);
    }
}

static void tcg_out_and(TCGContext *c, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(c, AND_OP0, AND_OP1, AND_OP2, ret, arg1, arg2);
}

static void tcg_out_extui(TCGContext *c, TCGReg ret, TCGReg arg1,
                          uint8_t shift, uint8_t mask)
{
    if (mask <= 16) {
        tcg_out_rrr(c, EXTUI_OP0, EXTUI_OP1 | ((shift >> 4) & 1), mask - 1,
                    ret, shift & 0xf, arg1);
    } else {
        tcg_abort();
    }
}

static void tcg_out_j(TCGContext *c, uint32_t offset)
{
    tcg_out_call_fmt(c, J_OP0, J_N, offset);
}

static void tcg_out_jx(TCGContext *c, uint8_t s)
{
    tcg_out_callx(c, JX_OP0, JX_OP1, JX_OP2, JX_N, JX_M, JX_R, s);
}

static void tcg_out_movgez(TCGContext *s, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(s, MOVGEZ_OP0, MOVGEZ_OP1, MOVGEZ_OP2, ret, arg1, arg2);
}

static void tcg_out_neg(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_rrr(s, NEG_OP0, NEG_OP1, NEG_OP2, ret, 0, arg);
}

static void tcg_out_or(TCGContext *c, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(c, OR_OP0, OR_OP1, OR_OP2, ret, arg1, arg2);
}

static void tcg_out_slli(TCGContext *s, TCGReg ret, TCGReg arg1, uint8_t arg2)
{
    uint8_t sa = 32 - (arg2 & 0x1f);

    tcg_out_rrr(s, SLLI_OP0, SLLI_OP1, SLLI_OP2 | (sa >> 4),
                ret, arg1, sa & 0xf);
}

static void tcg_out_srai(TCGContext *s, TCGReg ret, TCGReg arg1, uint8_t arg2)
{
    uint8_t sa = arg2 & 0x1f;

    tcg_out_rrr(s, SRAI_OP0, SRAI_OP1, SRAI_OP2 | (sa >> 4),
                ret, sa & 0xf, arg1);
}

static void tcg_out_src(TCGContext *s, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(s, SRC_OP0, SRC_OP1, SRC_OP2, ret, arg1, arg2);
}

static void tcg_out_srli(TCGContext *s, TCGReg ret, TCGReg arg, uint8_t sa)
{
    if (sa < 16) {
        tcg_out_rrr(s, SRLI_OP0, SRLI_OP1, SRLI_OP2, ret, sa, arg);
    } else {
        tcg_out_extui(s, ret, arg, sa, 32 - sa);
    }
}

static void tcg_out_ssai(TCGContext *s, uint8_t sa)
{
    tcg_out_rrr(s, SSAI_OP0, SSAI_OP1, SSAI_OP2, SSAI_R,
                sa & 0xf, (sa >> 4) & 1);
}

static void tcg_out_ssl(TCGContext *s, TCGReg arg)
{
    tcg_out_rrr(s, SSL_OP0, SSL_OP1, SSL_OP2, SSL_R, arg, SSL_T);
}

static void tcg_out_ssr(TCGContext *s, TCGReg arg)
{
    tcg_out_rrr(s, SSR_OP0, SSR_OP1, SSR_OP2, SSR_R, arg, SSR_T);
}

static void tcg_out_sub(TCGContext *c, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(c, SUB_OP0, SUB_OP1, SUB_OP2, ret, arg1, arg2);
}

static void tcg_out_xor(TCGContext *c, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(c, XOR_OP0, XOR_OP1, XOR_OP2, ret, arg1, arg2);
}

static void tcg_out_bswap16(TCGContext *s, TCGReg ret, TCGReg arg)
{
    TCGReg tmp = ret == arg ? TCG_REG_TMP : ret;

    tcg_out_ssai(s, 8);
    tcg_out_slli(s, tmp, arg, 16);
    tcg_out_src(s, tmp, arg, tmp);
    tcg_out_srli(s, ret, tmp, 16);
}

static void tcg_out_bswap16s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    TCGReg tmp = ret == arg ? TCG_REG_TMP : ret;

    tcg_out_ssai(s, 8);
    tcg_out_slli(s, tmp, arg, 16);
    tcg_out_src(s, tmp, arg, tmp);
    tcg_out_srai(s, ret, tmp, 16);
}

static void tcg_out_bswap32(TCGContext *s, TCGReg ret, TCGReg arg)
{
    TCGReg tmp = ret == arg ? TCG_REG_TMP : ret;

    tcg_out_ssai(s, 8);
    tcg_out_srli(s, tmp, arg, 16);
    tcg_out_src(s, tmp, tmp, arg);
    tcg_out_src(s, tmp, tmp, tmp);
    tcg_out_src(s, ret, arg, tmp);
}

static void tcg_out_movi32(TCGContext *s, TCGReg ret, tcg_target_long arg)
{
    if (arg == ((arg & 0xfff) ^ 0x800) - 0x800) {
        tcg_out_rri8(s, MOVI_OP0, MOVI_R, (arg >> 8) & 0xf, ret, arg & 0xff);
    } else if (arg == (int16_t)arg) {
        tcg_out_rri8(s, MOVI_OP0, MOVI_R, 0, ret, arg & 0xff);
        tcg_out_addmi(s, ret, ret, (arg >> 8) & 0xff);
    } else {
        uint32_t b0 = (int8_t)arg;
        uint32_t b1 = (int8_t)((arg - b0) >> 8);
        uint32_t b23 = (int16_t)((((arg - b0) >> 8) - b1) >>  8);

        tcg_out_movi32(s, ret, b23);
        tcg_out_slli(s, ret, ret, 16);
        tcg_out_addi(s, ret, ret, b0 & 0xff);
        tcg_out_addmi(s, ret, ret, b1 & 0xff);
    }
}

static void tcg_out_not(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_movi32(s, TCG_REG_TMP, -1);
    tcg_out_xor(s, ret, arg, TCG_REG_TMP);
}

static void reloc_pcrel18(tcg_insn_unit *code_ptr, tcg_insn_unit *value)
{
    intptr_t diff = value - code_ptr - 4;

    if (diff == ((diff & 0x3ffff) ^ 0x20000) - 0x20000) {
        code_ptr[0] =
            (code_ptr[0] & XTENSA_TCG_BYTE(0xf, XTENSA_TCG_NIBBLE(0x3, 0))) |
            XTENSA_TCG_BYTE(0, XTENSA_TCG_NIBBLE(0, XTENSA_TCG_BITS(diff, 18, 0, 2)));
        code_ptr[1] = XTENSA_TCG_BITS(diff, 18, 2, 8);
        code_ptr[2] = XTENSA_TCG_BITS(diff, 18, 10, 8);
    } else {
        tcg_abort();
    }
}

static void reloc_pcrel8(tcg_insn_unit *code_ptr, tcg_insn_unit *value)
{
    intptr_t diff = value - code_ptr - 4;

    if (diff == (int8_t)diff) {
        code_ptr[2] = diff;
    } else {
        tcg_abort();
    }
}

static void reloc_pcrel8_18(tcg_insn_unit *code_ptr, tcg_insn_unit *value)
{
    intptr_t diff = value - code_ptr - 4;

    if (diff == (int8_t)diff) {
        code_ptr[2] = diff;
    } else {
        code_ptr[1] ^= XTENSA_TCG_BYTE(0, 0x8);
        code_ptr[2] = 2;

        code_ptr[3] = XTENSA_TCG_BYTE(J_OP0, XTENSA_TCG_NIBBLE(J_N, 0));
        reloc_pcrel18(code_ptr + 3, value);
    }
}

static void patch_reloc(tcg_insn_unit *code_ptr, int type,
                        intptr_t value, intptr_t addend)
{
    switch (type) {
    case R_XTENSA_PCREL18:
        reloc_pcrel18(code_ptr, (tcg_insn_unit *)value);
        break;
    case R_XTENSA_PCREL8:
        reloc_pcrel8(code_ptr, (tcg_insn_unit *)value);
        break;
    case R_XTENSA_PCREL8_18:
        reloc_pcrel8_18(code_ptr, (tcg_insn_unit *)value);
        break;
    default:
        tcg_abort();
    }
}

static void tcg_out_add2(TCGContext *s, TCGReg retlo, TCGReg rethi,
                         TCGReg arg1lo, TCGReg arg1hi,
                         TCGReg arg2lo, TCGReg arg2hi)
{
    TCGReg cmp, tmp;
    tcg_insn_unit *code_ptr;

    if (retlo != arg1lo && rethi != arg1lo) {
        cmp = arg1lo;
    } else if (retlo != arg2lo && rethi != arg2lo) {
        cmp = arg2lo;
    } else {
        cmp = TCG_REG_TMP;
        tcg_out_mov(s, TCG_TYPE_REG, cmp, arg2lo);
    }
    if (retlo != arg1hi && retlo != arg2hi) {
        tmp = retlo;
    } else {
        tmp = TCG_REG_TMP1;
    }
    tcg_out_add(s, tmp, arg1lo, arg2lo);
    tcg_out_add(s, rethi, arg1hi, arg2hi);
    code_ptr = s->code_ptr;
    tcg_out_rri8(s, BANY_OP0, BGEU_R, tmp, cmp, 0);
    tcg_out_addi(s, rethi, rethi, 1);
    reloc_pcrel8(code_ptr, s->code_ptr);
    tcg_out_mov(s, TCG_TYPE_REG, retlo, tmp);
}

static void tcg_out_sub2(TCGContext *s, TCGReg retlo, TCGReg rethi,
                         TCGReg arg1lo, TCGReg arg1hi,
                         TCGReg arg2lo, TCGReg arg2hi)
{
    TCGReg tmp1 = arg1lo, tmp2 = arg2lo;
    tcg_insn_unit *code_ptr;

    if (rethi == arg1lo) {
        tmp1 = TCG_REG_TMP;
        tcg_out_mov(s, TCG_TYPE_REG, tmp1, arg1lo);
    }
    if (rethi == arg2lo) {
        tmp2 = TCG_REG_TMP1;
        tcg_out_mov(s, TCG_TYPE_REG, tmp2, arg2lo);
    }
    tcg_out_sub(s, rethi, arg1hi, arg2hi);
    code_ptr = s->code_ptr;
    tcg_out_rri8(s, BANY_OP0, BGEU_R, tmp1, tmp2, 0);
    tcg_out_addi(s, rethi, rethi, -1);
    reloc_pcrel8(code_ptr, s->code_ptr);
    tcg_out_sub(s, retlo, tmp1, tmp2);
}

static void tcg_out_call8(TCGContext *s, tcg_insn_unit *addr)
{
    intptr_t src = (intptr_t)s->code_ptr;
    intptr_t dst = (intptr_t)addr;
    ptrdiff_t diff = (dst >> 2) - (src >> 2) - 1;

    if (!(dst & 3) &&
        diff == ((diff & 0x3ffff) ^ 0x20000) - 0x20000) {
        tcg_out_call_fmt(s, CALL8_OP0, CALL8_N, diff);
    } else {
        tcg_out_movi32(s, TCG_REG_A8, dst);
        tcg_out_callx(s, CALLX8_OP0, CALLX8_OP1, CALLX8_OP2,
                      CALLX8_N, CALLX8_M, CALLX8_R, TCG_REG_A8);
    }
}

static void tcg_out_jl(TCGContext *s, tcg_insn_unit *addr)
{
    intptr_t addri = (intptr_t)addr;
    ptrdiff_t diff = tcg_pcrel_diff(s, addr) - 4;

    if (diff < 0x00020000 && diff >= 0xfffe0000) {
        tcg_out_j(s, diff);
        return;
    }

    tcg_out_movi32(s, TCG_REG_TMP, addri);
    tcg_out_jx(s, TCG_REG_TMP);
}

static void tcg_out_mul16u(TCGContext *s, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    tcg_out_rrr(s, MUL16U_OP0, MUL16U_OP1, MUL16U_OP2, ret, arg1, arg2);
}

static void tcg_out_muluh(TCGContext *s, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    if (XCHAL_HAVE_MUL32_HIGH) {
        tcg_out_rrr(s, MULUH_OP0, MULUH_OP1, MULUH_OP2, ret, arg1, arg2);
    } else {
        TCGReg tmp = ret == arg1 || ret == arg2 ? TCG_REG_TMP : ret;

        tcg_out_mul16u(s, tmp, arg1, arg2);
        tcg_out_srli(s, tmp, tmp, 16);
        tcg_out_extui(s, TCG_REG_TMP1, arg1, 16, 16);
        tcg_out_mul16u(s, TCG_REG_TMP2, TCG_REG_TMP1, arg2);
        tcg_out_add(s, tmp, tmp, TCG_REG_TMP2);
        tcg_out_extui(s, TCG_REG_TMP2, arg2, 16, 16);
        if (tmp == TCG_REG_TMP) {
            tcg_out_mul16u(s, TCG_REG_TMP1, TCG_REG_TMP2, arg1);
            tcg_out_add(s, tmp, tmp, TCG_REG_TMP1);
            tcg_out_extui(s, TCG_REG_TMP1, arg1, 16, 16);
        } else {
            tcg_out_mul16u(s, TCG_REG_TMP, TCG_REG_TMP2, arg1);
            tcg_out_add(s, tmp, tmp, TCG_REG_TMP);
        }
        tcg_out_srli(s, tmp, tmp, 16);
        tcg_out_mul16u(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_REG_TMP2);
        tcg_out_add(s, ret, tmp, TCG_REG_TMP1);
    }
}

static void tcg_out_mulsh(TCGContext *s, TCGReg ret, TCGReg arg1, TCGReg arg2)
{
    if (XCHAL_HAVE_MUL32_HIGH) {
        tcg_out_rrr(s, MULSH_OP0, MULSH_OP1, MULSH_OP2, ret, arg1, arg2);
    } else {
        tcg_out_neg(s, TCG_REG_TMP1, arg1);
        tcg_out_movgez(s, TCG_REG_TMP1, arg1, arg1);
        tcg_out_neg(s, TCG_REG_TMP2, arg2);
        tcg_out_movgez(s, TCG_REG_TMP2, arg2, arg2);

        tcg_out_mul16u(s, TCG_REG_TMP, TCG_REG_TMP1, TCG_REG_TMP2);
        tcg_out_srli(s, TCG_REG_TMP, TCG_REG_TMP, 16);

        tcg_out_extui(s, TCG_REG_TMP1, TCG_REG_TMP1, 16, 16);
        tcg_out_mul16u(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_REG_TMP2);
        tcg_out_add(s, TCG_REG_TMP, TCG_REG_TMP, TCG_REG_TMP1);

        tcg_out_neg(s, TCG_REG_TMP1, arg1);
        tcg_out_movgez(s, TCG_REG_TMP1, arg1, arg1);

        tcg_out_extui(s, TCG_REG_TMP2, TCG_REG_TMP2, 16, 16);
        tcg_out_mul16u(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_REG_TMP2);
        tcg_out_add(s, TCG_REG_TMP, TCG_REG_TMP, TCG_REG_TMP1);

        tcg_out_srli(s, TCG_REG_TMP, TCG_REG_TMP, 16);

        tcg_out_neg(s, TCG_REG_TMP1, arg1);
        tcg_out_movgez(s, TCG_REG_TMP1, arg1, arg1);
        tcg_out_extui(s, TCG_REG_TMP1, TCG_REG_TMP1, 16, 16);

        tcg_out_mul16u(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_REG_TMP2);
        tcg_out_add(s, TCG_REG_TMP, TCG_REG_TMP, TCG_REG_TMP1);

        tcg_out_xor(s, TCG_REG_TMP1, arg1, arg2);
        tcg_out_neg(s, ret, TCG_REG_TMP);
        tcg_out_movgez(s, ret, TCG_REG_TMP, TCG_REG_TMP1);
    }
}

static void tcg_out_sext(TCGContext *s, TCGReg ret, TCGReg arg, unsigned bits)
{
    if (XCHAL_HAVE_SEXT && bits >= 8 && bits < 24) {
        tcg_out_rrr(s, SEXT_OP0, SEXT_OP1, SEXT_OP2, ret, arg, bits - 8);
    } else {
        tcg_out_slli(s, ret, arg, 32 - bits);
        tcg_out_srai(s, ret, ret, 32 - bits);
    }
}

static void tcg_out_ldst(TCGContext *s, uint8_t r, unsigned order,
                         TCGReg v, TCGReg addr, intptr_t offset)
{
    if (offset & ~(0xff << order)) {
        assert(addr != TCG_REG_TMP2);
        if (offset == (int8_t)offset) {
            tcg_out_addi(s, TCG_REG_TMP2, addr, offset);
        } else {
            tcg_out_movi32(s, TCG_REG_TMP2, offset);
            tcg_out_add(s, TCG_REG_TMP2, TCG_REG_TMP2, addr);
        }
        addr = TCG_REG_TMP2;
        offset = 0;
    }
    /* All generic load/store instructions share OP0 code */
    tcg_out_rri8(s, L32I_OP0, r, addr, v, offset >> order);
}

static void tcg_out_br(TCGContext *s, TCGLabel *l)
{
    tcg_out_j(s, 0);
    if (l->has_value) {
        reloc_pcrel18(s->code_ptr - 3, l->u.value_ptr);
    } else {
        tcg_out_reloc(s, s->code_ptr - 3, R_XTENSA_PCREL18, l, 0);
    }
}

static void tcg_out_brcond_code(TCGContext *s, TCGCond cond,
                                TCGReg arg1, TCGReg arg2)
{
    static const uint8_t tcg_cond_to_xtensa_b_r[] = {
        [TCG_COND_EQ] = BEQ_R,
        [TCG_COND_NE] = BNE_R,
        [TCG_COND_LT] = BLT_R,
        [TCG_COND_GE] = BGE_R,
        [TCG_COND_LE] = BGE_R,
        [TCG_COND_GT] = BLT_R,
        [TCG_COND_LTU] = BLTU_R,
        [TCG_COND_GEU] = BGEU_R,
        [TCG_COND_LEU] = BGEU_R,
        [TCG_COND_GTU] = BLTU_R,
    };
    static const bool tcg_cond_to_xtensa_reorder[] = {
        [TCG_COND_LE] = true,
        [TCG_COND_GT] = true,
        [TCG_COND_LEU] = true,
        [TCG_COND_GTU] = true,
    };

    if (tcg_cond_to_xtensa_reorder[cond]) {
        TCGReg tmp = arg1;

        arg1 = arg2;
        arg2 = tmp;
    }
    /* All RRR branch instructions share the same OP0 code. */
    tcg_out_rri8(s, BANY_OP0, tcg_cond_to_xtensa_b_r[cond], arg1, arg2, 0);
}

static void tcg_out_brcond(TCGContext *s, TCGCond cond, TCGReg arg1,
                           TCGReg arg2, TCGLabel *l)
{
    tcg_out_brcond_code(s, cond, arg1, arg2);
    tcg_out_nop(s);
    if (l->has_value) {
        reloc_pcrel8_18(s->code_ptr - 6, l->u.value_ptr);
    } else {
        tcg_out_reloc(s, s->code_ptr - 6, R_XTENSA_PCREL8_18, l, 0);
    }
}

static void tcg_out_brcondi(TCGContext *s, TCGCond cond, TCGReg arg1,
                            intptr_t arg2, TCGLabel *l)
{
    tcg_out_movi32(s, TCG_REG_TMP, arg2);
    tcg_out_brcond(s, cond, arg1, TCG_REG_TMP, l);
}

static void tcg_out_setcond(TCGContext *s, TCGCond cond, TCGReg ret,
                            TCGReg arg1, TCGReg arg2)
{
    TCGReg tmp = (ret == arg1 || ret == arg2) ? TCG_REG_TMP : ret;
    tcg_insn_unit *code_ptr;

    tcg_out_movi32(s, tmp, 1);
    code_ptr = s->code_ptr;
    tcg_out_brcond_code(s, cond, arg1, arg2);
    tcg_out_movi32(s, tmp, 0);
    reloc_pcrel8(code_ptr, s->code_ptr);
    tcg_out_mov(s, TCG_TYPE_REG, ret, tmp);
}

static void tcg_out_setcond2(TCGContext *s, TCGCond cond, TCGReg ret,
                             TCGReg al, TCGReg ah, TCGReg bl, TCGReg bh)
{
    switch (cond) {
    case TCG_COND_EQ:
        tcg_out_setcond(s, cond, TCG_REG_TMP, ah, bh);
        tcg_out_setcond(s, cond, TCG_REG_TMP1, al, bl);
        tcg_out_and(s, ret, TCG_REG_TMP1, TCG_REG_TMP);
        break;

    case TCG_COND_NE:
        tcg_out_setcond(s, cond, TCG_REG_TMP, ah, bh);
        tcg_out_setcond(s, cond, TCG_REG_TMP1, al, bl);
        tcg_out_or(s, ret, TCG_REG_TMP1, TCG_REG_TMP);
        break;

    default:
        tcg_out_setcond(s, TCG_COND_EQ, TCG_REG_TMP, ah, bh);
        tcg_out_setcond(s, tcg_unsigned_cond(cond), TCG_REG_TMP1, al, bl);
        tcg_out_and(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_REG_TMP);
        tcg_out_setcond(s, tcg_high_cond(cond), TCG_REG_TMP, ah, bh);
        tcg_out_or(s, ret, TCG_REG_TMP, TCG_REG_TMP1);
        break;
    }
}

static void tcg_out_brcond2(TCGContext *s, TCGCond cond, TCGReg al, TCGReg ah,
                            TCGReg bl, TCGReg bh, TCGLabel *l)
{
    tcg_out_setcond2(s, cond, TCG_REG_TMP1, al, ah, bl, bh);
    tcg_out_brcondi(s, TCG_COND_NE, TCG_REG_TMP1, 0, l);
}

#if defined(CONFIG_SOFTMMU)
static void * const qemu_ld_helpers[16] = {
    [MO_UB]   = helper_ret_ldub_mmu,
    [MO_SB]   = helper_ret_ldsb_mmu,
    [MO_LEUW] = helper_le_lduw_mmu,
    [MO_LESW] = helper_le_ldsw_mmu,
    [MO_LEUL] = helper_le_ldul_mmu,
    [MO_LEQ]  = helper_le_ldq_mmu,
    [MO_BEUW] = helper_be_lduw_mmu,
    [MO_BESW] = helper_be_ldsw_mmu,
    [MO_BEUL] = helper_be_ldul_mmu,
    [MO_BEQ]  = helper_be_ldq_mmu,
};

static void * const qemu_st_helpers[16] = {
    [MO_UB]   = helper_ret_stb_mmu,
    [MO_LEUW] = helper_le_stw_mmu,
    [MO_LEUL] = helper_le_stl_mmu,
    [MO_LEQ]  = helper_le_stq_mmu,
    [MO_BEUW] = helper_be_stw_mmu,
    [MO_BEUL] = helper_be_stl_mmu,
    [MO_BEQ]  = helper_be_stq_mmu,
};

static int tcg_out_call_iarg_reg(TCGContext *s, int i, TCGReg arg)
{
    if (i < ARRAY_SIZE(tcg_target_call_iarg_regs)) {
        tcg_out_mov(s, TCG_TYPE_REG, tcg_target_call_iarg_regs[i], arg);
    } else {
        tcg_out_ldst(s, S32I_R, 2, arg, TCG_REG_A1,
                     4 * (i - ARRAY_SIZE(tcg_target_call_iarg_regs)));
    }
    return i + 1;
}

static int tcg_out_call_iarg_reg2(TCGContext *s, int i,
                                  TCGReg arglo, TCGReg arghi)
{
    i = (i + 1) & ~1;
    i = tcg_out_call_iarg_reg(s, i, (XCHAL_HAVE_BE ? arghi : arglo));
    i = tcg_out_call_iarg_reg(s, i, (XCHAL_HAVE_BE ? arglo : arghi));
    return i;
}

static int tcg_out_call_iarg_imm(TCGContext *s, int i, TCGArg arg)
{
    TCGReg tmp;

    if (i < ARRAY_SIZE(tcg_target_call_iarg_regs)) {
        tmp = tcg_target_call_iarg_regs[i];
    } else {
        tmp = TCG_REG_TMP;
    }
    tcg_out_movi32(s, tmp, arg);

    return tcg_out_call_iarg_reg(s, i, tmp);
}

static void tcg_out_tlb_read(TCGContext *s, TCGReg base,
                             TCGReg addrlo, TCGReg addrhi,
                             TCGMemOpIdx oi, tcg_insn_unit *label_ptr[2],
                             bool is_load)
{
    TCGMemOp s_bits = get_memop(oi) & MO_SIZE;
    int mem_index = get_mmuidx(oi);
    int tlb_entry_off = offsetof(CPUArchState, tlb_table[mem_index][0]);
    int cmp_off = is_load
        ? offsetof(CPUTLBEntry, addr_read)
        : offsetof(CPUTLBEntry, addr_write);

    tcg_out_extui(s, TCG_REG_TMP1, addrlo, TARGET_PAGE_BITS, CPU_TLB_BITS);
    tcg_out_slli(s, TCG_REG_TMP1, TCG_REG_TMP1, CPU_TLB_ENTRY_BITS);
    tcg_out_movi32(s, TCG_REG_TMP, tlb_entry_off);
    tcg_out_add(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_REG_TMP);
    tcg_out_add(s, TCG_REG_TMP1, TCG_REG_TMP1, TCG_AREG0);

    tcg_out_ldst(s, L32I_R, 2, TCG_REG_TMP, TCG_REG_TMP1, cmp_off + LO_OFF);
    tcg_out_movi32(s, TCG_REG_TMP2, TARGET_PAGE_MASK | ((1 << s_bits) - 1));
    tcg_out_rrr(s, AND_OP0, AND_OP1, AND_OP2,
                TCG_REG_TMP2, TCG_REG_TMP2, addrlo);

    label_ptr[0] = s->code_ptr;
    tcg_out_rri8(s, BANY_OP0, BNE_R,
                 TCG_REG_TMP, TCG_REG_TMP2, 0);
    tcg_out_nop(s);

    if (TARGET_LONG_BITS == 64) {
        tcg_out_ldst(s, L32I_R, 2, TCG_REG_TMP, TCG_REG_TMP1, cmp_off + HI_OFF);
        label_ptr[1] = s->code_ptr;
        tcg_out_rri8(s, BANY_OP0, BNE_R, TCG_REG_TMP, addrhi, 0);
        tcg_out_nop(s);
    }

    tcg_out_ldst(s, L32I_R, 2, TCG_REG_TMP, TCG_REG_TMP1,
                 offsetof(CPUTLBEntry, addend));
    tcg_out_add(s, base, TCG_REG_TMP, addrlo);
}

static void add_qemu_ldst_label(TCGContext *s, int is_ld, TCGMemOpIdx oi,
                                TCGReg datalo, TCGReg datahi,
                                TCGReg addrlo, TCGReg addrhi,
                                void *raddr, tcg_insn_unit *label_ptr[2])
{
    TCGLabelQemuLdst *label = new_ldst_label(s);

    label->is_ld = is_ld;
    label->oi = oi;
    label->datalo_reg = datalo;
    label->datahi_reg = datahi;
    label->addrlo_reg = addrlo;
    label->addrhi_reg = addrhi;
    label->raddr = raddr;
    label->label_ptr[0] = label_ptr[0];
    if (TARGET_LONG_BITS == 64) {
        label->label_ptr[1] = label_ptr[1];
    }
}

static void tcg_out_qemu_ld_slow_path(TCGContext *s, TCGLabelQemuLdst *l)
{
    TCGMemOpIdx oi = l->oi;
    TCGMemOp opc = get_memop(oi);
    TCGReg ret;
    int i = 1;

    /* resolve label address */
    reloc_pcrel8_18(l->label_ptr[0], s->code_ptr);
    if (TARGET_LONG_BITS == 64) {
        reloc_pcrel8_18(l->label_ptr[1], s->code_ptr);
    }

    if (TARGET_LONG_BITS == 64) {
        i = tcg_out_call_iarg_reg2(s, i, l->addrlo_reg, l->addrhi_reg);
    } else {
        i = tcg_out_call_iarg_reg(s, i, l->addrlo_reg);
    }
    i = tcg_out_call_iarg_imm(s, i, oi);
    i = tcg_out_call_iarg_imm(s, i, (intptr_t)l->raddr);
    tcg_out_call_iarg_reg(s, 0, TCG_AREG0);
    tcg_out_call8(s, qemu_ld_helpers[opc & (MO_BSWAP | MO_SSIZE)]);

    ret = l->datalo_reg;
    if ((opc & MO_SIZE) == MO_64) {
        if (XCHAL_HAVE_BE) {
            tcg_out_mov(s, TCG_TYPE_I32, ret, TCG_REG_A11);
            ret = l->datahi_reg;
        } else {
            tcg_out_mov(s, TCG_TYPE_I32, l->datahi_reg, TCG_REG_A11);
        }
    }
    tcg_out_mov(s, TCG_TYPE_REG, ret, TCG_REG_A10);
    tcg_out_jl(s, l->raddr);
}

static void tcg_out_qemu_st_slow_path(TCGContext *s, TCGLabelQemuLdst *l)
{
    TCGMemOpIdx oi = l->oi;
    TCGMemOp opc = get_memop(oi);
    TCGMemOp s_bits = opc & MO_SIZE;
    int i = 1;

    /* resolve label address */
    reloc_pcrel8_18(l->label_ptr[0], s->code_ptr);
    if (TARGET_LONG_BITS == 64) {
        reloc_pcrel8_18(l->label_ptr[1], s->code_ptr);
    }

    if (TARGET_LONG_BITS == 64) {
        i = tcg_out_call_iarg_reg2(s, i, l->addrlo_reg, l->addrhi_reg);
    } else {
        i = tcg_out_call_iarg_reg(s, i, l->addrlo_reg);
    }

    switch (s_bits) {
    case MO_8:
    case MO_16:
    case MO_32:
        i = tcg_out_call_iarg_reg(s, i, l->datalo_reg);
        break;
    case MO_64:
        i = tcg_out_call_iarg_reg2(s, i, l->datalo_reg, l->datahi_reg);
        break;
    default:
        tcg_abort();
    }

    i = tcg_out_call_iarg_imm(s, i, oi);
    i = tcg_out_call_iarg_imm(s, i, (intptr_t)l->raddr);
    tcg_out_call_iarg_reg(s, 0, TCG_AREG0);
    tcg_out_call8(s, qemu_st_helpers[opc & (MO_BSWAP | MO_SSIZE)]);
    tcg_out_jl(s, l->raddr);
}
#endif

static void tcg_out_qemu_ld_direct(TCGContext *s, TCGReg datalo, TCGReg datahi,
                                   TCGReg base, TCGMemOp opc)
{
    switch (opc & (MO_SSIZE | MO_BSWAP)) {
    case MO_UB:
        tcg_out_ldst(s, L8UI_R, 0, datalo, base, 0);
        break;
    case MO_SB:
        tcg_out_ldst(s, L8UI_R, 0, datalo, base, 0);
        tcg_out_sext(s, datalo, datalo, 8);
        break;
    case MO_UW | MO_BSWAP:
        tcg_out_ldst(s, L16UI_R, 1, TCG_REG_TMP1, base, 0);
        tcg_out_bswap16(s, datalo, TCG_REG_TMP1);
        break;
    case MO_UW:
        tcg_out_ldst(s, L16UI_R, 1, datalo, base, 0);
        break;
    case MO_SW | MO_BSWAP:
        tcg_out_ldst(s, L16UI_R, 1, TCG_REG_TMP1, base, 0);
        tcg_out_bswap16s(s, datalo, TCG_REG_TMP1);
        break;
    case MO_SW:
        tcg_out_ldst(s, L16SI_R, 1, datalo, base, 0);
        break;
    case MO_UL | MO_BSWAP:
        tcg_out_ldst(s, L32I_R, 2, TCG_REG_TMP1, base, 0);
        tcg_out_bswap32(s, datalo, TCG_REG_TMP1);
        break;
    case MO_UL:
        tcg_out_ldst(s, L32I_R, 2, datalo, base, 0);
        break;
    case MO_Q | MO_BSWAP:
        tcg_out_ldst(s, L32I_R, 2, TCG_REG_TMP1, base, HI_OFF);
        tcg_out_bswap32(s, datalo, TCG_REG_TMP1);
        tcg_out_ldst(s, L32I_R, 2, TCG_REG_TMP1, base, LO_OFF);
        tcg_out_bswap32(s, datahi, TCG_REG_TMP1);
        break;
    case MO_Q:
        tcg_out_ldst(s, L32I_R, 2, datalo, base, LO_OFF);
        tcg_out_ldst(s, L32I_R, 2, datahi, base, HI_OFF);
        break;
    default:
        tcg_abort();
    }
}

static void tcg_out_qemu_ld(TCGContext *s, const TCGArg *args, bool is_64)
{
    TCGReg addrlo, datalo, datahi, addrhi __attribute__((unused));
    TCGMemOpIdx oi;
    TCGMemOp opc;
#ifdef CONFIG_SOFTMMU
    tcg_insn_unit *label_ptr[2];
#endif
    TCGReg base = TCG_REG_TMP;

    datalo = *args++;
    datahi = (is_64 ? *args++ : 0);
    addrlo = *args++;
    addrhi = (TARGET_LONG_BITS == 64 ? *args++ : 0);
    oi = *args++;
    opc = get_memop(oi);

#ifdef CONFIG_SOFTMMU
    tcg_out_tlb_read(s, base, addrlo, addrhi, oi, label_ptr, 1);
    tcg_out_qemu_ld_direct(s, datalo, datahi, base, opc);
    add_qemu_ldst_label(s, 1, oi, datalo, datahi, addrlo, addrhi,
                        s->code_ptr, label_ptr);
#else /* !CONFIG_SOFTMMU */
    if (guest_base == 0) {
        if (addrlo != datalo && addrlo != datahi) {
            base = addrlo;
        } else {
            tcg_out_mov(s, TCG_TYPE_REG, base, addrlo);
        }
    } else {
        tcg_out_movi32(s, base, guest_base);
        tcg_out_add(s, base, base, addrlo);
    }
    tcg_out_qemu_ld_direct(s, datalo, datahi, base, opc);
#endif
}

static void tcg_out_qemu_st_direct(TCGContext *s, TCGReg datalo, TCGReg datahi,
                                   TCGReg base, TCGMemOp opc)
{
    switch (opc & (MO_SSIZE | MO_BSWAP)) {
    case MO_8:
        tcg_out_ldst(s, S8I_R, 0, datalo, base, 0);
        break;

    case MO_16 | MO_BSWAP:
        tcg_out_bswap16(s, TCG_REG_TMP1, datalo);
        datalo = TCG_REG_TMP1;
        /* fallthrough */
    case MO_16:
        tcg_out_ldst(s, S16I_R, 1, datalo, base, 0);
        break;

    case MO_32 | MO_BSWAP:
        tcg_out_bswap32(s, TCG_REG_TMP1, datalo);
        datalo = TCG_REG_TMP1;
        /* fallthrough */
    case MO_32:
        tcg_out_ldst(s, S32I_R, 2, datalo, base, 0);
        break;

    case MO_64 | MO_BSWAP:
        tcg_out_bswap32(s, TCG_REG_TMP1, datalo);
        tcg_out_ldst(s, S32I_R, 2, TCG_REG_TMP1, base, HI_OFF);
        tcg_out_bswap32(s, TCG_REG_TMP1, datahi);
        tcg_out_ldst(s, S32I_R, 2, TCG_REG_TMP1, base, LO_OFF);
        break;
    case MO_64:
        tcg_out_ldst(s, S32I_R, 2, datalo, base, LO_OFF);
        tcg_out_ldst(s, S32I_R, 2, datahi, base, HI_OFF);
        break;
    default:
        tcg_abort();
    }
}

static void tcg_out_qemu_st(TCGContext *s, const TCGArg *args, bool is_64)
{
    TCGReg addrlo, datalo, datahi, addrhi __attribute__((unused));
    TCGMemOpIdx oi;
    TCGMemOp opc;
#ifdef CONFIG_SOFTMMU
    tcg_insn_unit *label_ptr[2];
#endif
    TCGReg base = TCG_REG_TMP;

    datalo = *args++;
    datahi = (is_64 ? *args++ : 0);
    addrlo = *args++;
    addrhi = (TARGET_LONG_BITS == 64 ? *args++ : 0);
    oi = *args++;
    opc = get_memop(oi);

#ifdef CONFIG_SOFTMMU
    tcg_out_tlb_read(s, base, addrlo, addrhi, oi, label_ptr, 0);
    tcg_out_qemu_st_direct(s, datalo, datahi, base, opc);
    add_qemu_ldst_label(s, 0, oi, datalo, datahi, addrlo, addrhi,
                        s->code_ptr, label_ptr);
#else /* !CONFIG_SOFTMMU */
    if (guest_base == 0) {
        base = addrlo;
    } else {
        tcg_out_movi32(s, base, guest_base);
        tcg_out_add(s, base, base, addrlo);
    }
    tcg_out_qemu_st_direct(s, datalo, datahi, base, opc);
#endif
}

#define TCG_CT_CONST_S8   0x100
#define TCG_CT_CONST_B4   0x200
#define TCG_CT_CONST_B4U  0x400

/* parse target specific constraints */
static int target_parse_constraint(TCGArgConstraint *ct, const char **pct_str)
{
    const char *ct_str;

    ct_str = *pct_str;
    switch (ct_str[0]) {
    case 'r':
        ct->ct |= TCG_CT_REG;
        tcg_regset_set32(ct->u.regs, 0, 0xffff);
        break;
    case 'q': /* qemu_ld output arg constraint */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set32(ct->u.regs, 0, 0xffff);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A10);
        break;
    case 'l': /* qemu_ld input arg constraint */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set32(ct->u.regs, 0, 0xffff);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A11);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A12);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A13);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A14);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A15);
        break;
    case 's': /* qemu_st constraint */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set32(ct->u.regs, 0, 0xffff);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A11);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A12);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A13);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A14);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A15);
        break;
    case 'J':
        ct->ct |= TCG_CT_CONST_S8;
        break;
    case 'K':
        ct->ct |= TCG_CT_CONST_B4;
        break;
    case 'L':
        ct->ct |= TCG_CT_CONST_B4U;
        break;
    default:
        return -1;
    }
    ct_str++;
    *pct_str = ct_str;

    return 0;
}

static int xtensa_simm8(tcg_target_long val)
{
    return val == (int8_t)val;
}

static int xtensa_b4(tcg_target_long val)
{
    static const tcg_target_long b4const[] = {
        -1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256,
    };
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(b4const); ++i)
        if (val == b4const[i])
            return 1;
    return 0;
}

static int xtensa_b4u(tcg_target_long val)
{
    static const tcg_target_long b4constu[] = {
        32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256,
    };
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(b4constu); ++i)
        if (val == b4constu[i])
            return 1;
    return 0;
}

/* test if a constant matches the constraint */
static inline int tcg_target_const_match(tcg_target_long val, TCGType type,
                                         const TCGArgConstraint *arg_ct)
{
    int ct = arg_ct->ct;

    if (ct & TCG_CT_CONST) {
        return 1;
    } else if ((ct & TCG_CT_CONST_S8) && xtensa_simm8(val)) {
        return 1;
    } else if ((ct & TCG_CT_CONST_B4) && (val == 0 || xtensa_b4(val))) {
        return 1;
    } else if ((ct & TCG_CT_CONST_B4U) && xtensa_b4u(val)) {
        return 1;
    } else {
        return 0;
    }
}

#define XTENSA_SPILL_SIZE 32
#define FRAME_SIZE ((XTENSA_SPILL_SIZE + \
                     CPU_TEMP_BUF_NLONGS * sizeof(long) + \
                     TCG_STATIC_CALL_ARGS_SIZE + \
                     TCG_TARGET_STACK_ALIGN - 1) & \
                    ~(TCG_TARGET_STACK_ALIGN - 1))

/* Generate global QEMU prologue and epilogue code */
static void tcg_target_qemu_prologue(TCGContext *s)
{
    /* Make stack frame.  */
    tcg_out_bri12(s, ENTRY_OP0, ENTRY_N, ENTRY_M, TCG_REG_SP, FRAME_SIZE / 8);
    tcg_set_frame(s, TCG_REG_SP, TCG_STATIC_CALL_ARGS_SIZE,
                  CPU_TEMP_BUF_NLONGS * sizeof(long));

    /* Call generated code.  */
    tcg_out_mov(s, TCG_TYPE_PTR, TCG_AREG0, TCG_REG_A2);
    tcg_out_jx(s, TCG_REG_A3);
    tb_ret_addr = s->code_ptr;

    /* Epilogue.  */
    tcg_out_callx(s, RETW_OP0, RETW_OP1, RETW_OP2, RETW_N, RETW_M, RETW_R,
                  RETW_S);
}

static void tcg_out_op(TCGContext *s, TCGOpcode opc,
                       const TCGArg args[TCG_MAX_OP_ARGS],
                       const int const_args[TCG_MAX_OP_ARGS])
{
    switch (opc) {
    case INDEX_op_exit_tb:
        tcg_out_movi32(s, TCG_REG_A2, args[0]);
        tcg_out_jl(s, tb_ret_addr);
        break;
    case INDEX_op_goto_tb:
        if (s->tb_jmp_offset) {
            /* Direct jump method */
            s->tb_jmp_offset[args[0]] = tcg_current_code_size(s);
            tcg_out8(s, XTENSA_TCG_BYTE(J_OP0, XTENSA_TCG_NIBBLE(J_N, XTENSA_TCG_BITS(*s->code_ptr, 8, 6, 2))));
            tcg_out8(s, *s->code_ptr);
            tcg_out8(s, *s->code_ptr);
        } else {
            /* Indirect jump method */
            intptr_t ptr = (intptr_t)(s->tb_next + args[0]);

            tcg_out_movi32(s, TCG_REG_TMP, ptr);
            tcg_out_ld(s, TCG_TYPE_PTR, TCG_REG_TMP, TCG_REG_TMP, 0);
            tcg_out_jx(s, TCG_REG_TMP);
        }
        s->tb_next_offset[args[0]] = tcg_current_code_size(s);
        break;
    case INDEX_op_br:
        tcg_out_br(s, arg_label(args[0]));
        break;

    case INDEX_op_ld8u_i32:
        tcg_out_ldst(s, L8UI_R, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld8s_i32:
        tcg_out_ldst(s, L8UI_R, 0, args[0], args[1], args[2]);
        tcg_out_sext(s, args[0], args[0], 8);
        break;
    case INDEX_op_ld16u_i32:
        tcg_out_ldst(s, L16UI_R, 1, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld16s_i32:
        tcg_out_ldst(s, L16SI_R, 1, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld_i32:
        tcg_out_ldst(s, L32I_R, 2, args[0], args[1], args[2]);
        break;
    case INDEX_op_st8_i32:
        tcg_out_ldst(s, S8I_R, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_st16_i32:
        tcg_out_ldst(s, S16I_R, 1, args[0], args[1], args[2]);
        break;
    case INDEX_op_st_i32:
        tcg_out_ldst(s, S32I_R, 2, args[0], args[1], args[2]);
        break;

    case INDEX_op_add_i32:
        if (const_args[2]) {
            tcg_out_addi(s, args[0], args[1], args[2]);
        } else {
            tcg_out_add(s, args[0], args[1], args[2]);
        }
        break;
    case INDEX_op_sub_i32:
        tcg_out_sub(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_mul_i32:
        tcg_out_rrr(s, MULL_OP0, MULL_OP1, MULL_OP2,
                    args[0], args[1], args[2]);
        break;
    case INDEX_op_muluh_i32:
        tcg_out_muluh(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_mulsh_i32:
        tcg_out_mulsh(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_div_i32:
        tcg_out_rrr(s, QUOS_OP0, QUOS_OP1, QUOS_OP2,
                    args[0], args[1], args[2]);
        break;
    case INDEX_op_divu_i32:
        tcg_out_rrr(s, QUOU_OP0, QUOU_OP1, QUOU_OP2,
                    args[0], args[1], args[2]);
        break;
    case INDEX_op_rem_i32:
        tcg_out_rrr(s, REMS_OP0, REMS_OP1, REMS_OP2,
                    args[0], args[1], args[2]);
        break;
    case INDEX_op_remu_i32:
        tcg_out_rrr(s, REMU_OP0, REMU_OP1, REMU_OP2,
                    args[0], args[1], args[2]);
        break;
    case INDEX_op_neg_i32:
        tcg_out_neg(s, args[0], args[1]);
        break;
    case INDEX_op_not_i32:
        tcg_out_not(s, args[0], args[1]);
        break;
    case INDEX_op_and_i32:
        tcg_out_and(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_or_i32:
        tcg_out_or(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_xor_i32:
        tcg_out_xor(s, args[0], args[1], args[2]);
        break;

    case INDEX_op_shl_i32:
        if (const_args[2]) {
            tcg_out_slli(s, args[0], args[1], args[2]);
        } else {
            tcg_out_ssl(s, args[2]);
            tcg_out_rrr(s, SLL_OP0, SLL_OP1, SLL_OP2,
                        args[0], args[1], SLL_T);
        }
        break;
    case INDEX_op_shr_i32:
        if (const_args[2]) {
            tcg_out_srli(s, args[0], args[1], args[2] & 0x1f);
        } else {
            tcg_out_ssr(s, args[2]);
            tcg_out_rrr(s, SRL_OP0, SRL_OP1, SRL_OP2,
                        args[0], SRL_S, args[1]);
        }
        break;
    case INDEX_op_sar_i32:
        if (const_args[2]) {
            tcg_out_srai(s, args[0], args[1], args[2]);
        } else {
            tcg_out_ssr(s, args[2]);
            tcg_out_rrr(s, SRA_OP0, SRA_OP1, SRA_OP2,
                        args[0], SRA_S, args[1]);
        }
        break;
    case INDEX_op_rotl_i32:
        if (const_args[2]) {
            tcg_out_ssai(s, 32 - args[2]);
        } else {
            tcg_out_ssl(s, args[2]);
        }
        tcg_out_src(s, args[0], args[1], args[1]);
        break;
    case INDEX_op_rotr_i32:
        if (const_args[2]) {
            tcg_out_ssai(s, args[2]);
        } else {
            tcg_out_ssr(s, args[2]);
        }
        tcg_out_src(s, args[0], args[1], args[1]);
        break;

    case INDEX_op_brcond_i32:
        if (const_args[1]) {
            tcg_out_brcondi(s, args[2], args[0], args[1], arg_label(args[3]));
        } else {
            tcg_out_brcond(s, args[2], args[0], args[1], arg_label(args[3]));
        }
        break;
    case INDEX_op_setcond_i32:
        tcg_out_setcond(s, args[3], args[0], args[1], args[2]);
        break;

    case INDEX_op_qemu_ld_i32:
        tcg_out_qemu_ld(s, args, false);
        break;
    case INDEX_op_qemu_ld_i64:
        tcg_out_qemu_ld(s, args, true);
        break;
    case INDEX_op_qemu_st_i32:
        tcg_out_qemu_st(s, args, false);
        break;
    case INDEX_op_qemu_st_i64:
        tcg_out_qemu_st(s, args, true);
        break;

    case INDEX_op_ext8s_i32:
        tcg_out_sext(s, args[0], args[1], 8);
        break;
    case INDEX_op_ext16s_i32:
        tcg_out_sext(s, args[0], args[1], 16);
        break;
    case INDEX_op_ext8u_i32:
        tcg_out_extui(s, args[0], args[1], 0, 8);
        break;
    case INDEX_op_ext16u_i32:
        tcg_out_extui(s, args[0], args[1], 0, 16);
        break;

    case INDEX_op_add2_i32:
        tcg_out_add2(s, args[0], args[1], args[2], args[3], args[4], args[5]);
        break;
    case INDEX_op_sub2_i32:
        tcg_out_sub2(s, args[0], args[1], args[2], args[3], args[4], args[5]);
        break;

    case INDEX_op_brcond2_i32:
        tcg_out_brcond2(s, args[4], args[0], args[1], args[2], args[3],
                        arg_label(args[5]));
        break;
    case INDEX_op_setcond2_i32:
        tcg_out_setcond2(s, args[5], args[0],
                         args[1], args[2], args[3], args[4]);
        break;

    default:
        fprintf(stderr, "%s: opc: %d\n", __func__, opc);
        tcg_abort();
    }
}

static const TCGTargetOpDef xtensa_op_defs[] = {
    { INDEX_op_exit_tb, { } },
    { INDEX_op_goto_tb, { } },
    { INDEX_op_br, { } },

    { INDEX_op_ld8u_i32, { "r", "r" } },
    { INDEX_op_ld8s_i32, { "r", "r" } },
    { INDEX_op_ld16u_i32, { "r", "r" } },
    { INDEX_op_ld16s_i32, { "r", "r" } },
    { INDEX_op_ld_i32, { "r", "r" } },
    { INDEX_op_st8_i32, { "r", "r" } },
    { INDEX_op_st16_i32, { "r", "r" } },
    { INDEX_op_st_i32, { "r", "r" } },

    { INDEX_op_add_i32, { "r", "r", "rJ" } },
    { INDEX_op_sub_i32, { "r", "r", "r" } },
    { INDEX_op_mul_i32, { "r", "r", "r" } },
    { INDEX_op_muluh_i32, { "r", "r", "r" } },
    { INDEX_op_mulsh_i32, { "r", "r", "r" } },
    { INDEX_op_div_i32, { "r", "r", "r" } },
    { INDEX_op_divu_i32, { "r", "r", "r" } },
    { INDEX_op_rem_i32, { "r", "r", "r" } },
    { INDEX_op_remu_i32, { "r", "r", "r" } },
    { INDEX_op_neg_i32, { "r", "r" } },
    { INDEX_op_not_i32, { "r", "r" } },

    { INDEX_op_and_i32, { "r", "r", "r" } },
    { INDEX_op_or_i32, { "r", "r", "r" } },
    { INDEX_op_xor_i32, { "r", "r", "r" } },

    { INDEX_op_shl_i32, { "r", "r", "rJ" } },
    { INDEX_op_shr_i32, { "r", "r", "rJ" } },
    { INDEX_op_sar_i32, { "r", "r", "rJ" } },
    { INDEX_op_rotl_i32, { "r", "r", "rJ" } },
    { INDEX_op_rotr_i32, { "r", "r", "rJ" } },

    { INDEX_op_brcond_i32, { "r", "r" } }, //"rKL"
    { INDEX_op_setcond_i32, { "r", "r", "r" } },

#if TARGET_LONG_BITS == 32
    { INDEX_op_qemu_ld_i32, { "q", "l" } },
    { INDEX_op_qemu_ld_i64, { "q", "q", "l" } },
    { INDEX_op_qemu_st_i32, { "s", "s" } },
    { INDEX_op_qemu_st_i64, { "s", "s", "s" } },
#else
    { INDEX_op_qemu_ld_i32, { "q", "q", "l" } },
    { INDEX_op_qemu_ld_i64, { "q", "q", "l", "l" } },
    { INDEX_op_qemu_st_i32, { "s", "s", "s" } },
    { INDEX_op_qemu_st_i64, { "s", "s", "s", "s" } },
#endif

    { INDEX_op_ext8s_i32, { "r", "r" } },
    { INDEX_op_ext16s_i32, { "r", "r" } },
    { INDEX_op_ext8u_i32, { "r", "r" } },
    { INDEX_op_ext16u_i32, { "r", "r" } },

    { INDEX_op_add2_i32, { "r", "r", "r", "r", "r", "r" } },
    { INDEX_op_sub2_i32, { "r", "r", "r", "r", "r", "r" } },
    { INDEX_op_brcond2_i32, { "r", "r", "r", "r" } },
    { INDEX_op_setcond2_i32, { "r", "r", "r", "r", "r" } },

    { -1 },
};

static void tcg_target_init(TCGContext *s)
{
    tcg_regset_set(tcg_target_available_regs[TCG_TYPE_I32], 0xffff);

    tcg_regset_set(tcg_target_call_clobber_regs,
                   (1 << TCG_REG_A8)  |
                   (1 << TCG_REG_A9)  |
                   (1 << TCG_REG_A10) |
                   (1 << TCG_REG_A11) |
                   (1 << TCG_REG_A12) |
                   (1 << TCG_REG_A13) |
                   (1 << TCG_REG_A14) |
                   (1 << TCG_REG_A15));

    tcg_regset_clear(s->reserved_regs);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_A0); /* return address */
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_A1); /* stack pointer */
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TMP); /* temporary register */
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TMP1); /* temporary register */
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TMP2); /* temporary register */

    tcg_add_target_add_op_defs(xtensa_op_defs);
}

static void tcg_out_call(TCGContext *s, tcg_insn_unit *addr)
{
    tcg_out_call8(s, addr);
}

static inline void tcg_out_ld(TCGContext *s, TCGType type, TCGReg arg,
                              TCGReg arg1, intptr_t arg2)
{
    tcg_out_ldst(s, L32I_R, 2, arg, arg1, arg2);
}

static inline void tcg_out_st(TCGContext *s, TCGType type, TCGReg arg,
                              TCGReg arg1, intptr_t arg2)
{
    tcg_out_ldst(s, S32I_R, 2, arg, arg1, arg2);
}

static inline void tcg_out_mov(TCGContext *s, TCGType type,
                               TCGReg ret, TCGReg arg)
{
    if (ret != arg) {
        tcg_out_rrr(s, MOV_OP0, MOV_OP1, MOV_OP2, ret, arg, arg);
    }
}

static inline void tcg_out_movi(TCGContext *s, TCGType type,
                                TCGReg ret, tcg_target_long arg)
{
    tcg_out_movi32(s, ret, arg);
}

# define ELF_HOST_MACHINE  EM_XTENSA

typedef struct {
    DebugFrameHeader h;
    uint8_t fde_def_cfa[2];
    uint8_t fde_win_save;
    uint8_t fde_ret_save[3];
} DebugFrame;

static const DebugFrame debug_frame = {
    .h.cie.len = sizeof(DebugFrameCIE)-4, /* length after .len member */
    .h.cie.id = -1,
    .h.cie.version = 1,
    .h.cie.code_align = 1,
    .h.cie.data_align = -sizeof(void *) & 0x7f,
    .h.cie.return_column = 15,            /* o7 */

    /* Total FDE size does not include the "len" member.  */
    .h.fde.len = sizeof(DebugFrame) - offsetof(DebugFrame, h.fde.cie_offset),

    .fde_def_cfa = {
        13, 30                          /* DW_CFA_def_cfa_register i6 */
    },
    .fde_win_save = 0x2d,               /* DW_CFA_GNU_window_save */
    .fde_ret_save = { 9, 15, 31 },      /* DW_CFA_register o7, i7 */
};

void tcg_register_jit(void *buf, size_t buf_size)
{
    tcg_register_jit_int(buf, buf_size, &debug_frame, sizeof(debug_frame));
}

void tb_set_jmp_target1(uintptr_t jmp_addr, uintptr_t addr)
{
    reloc_pcrel18((tcg_insn_unit *)jmp_addr, (tcg_insn_unit *)addr);
}

void flush_icache_range(uintptr_t start, uintptr_t stop)
{
    uintptr_t addr;

#if XCHAL_DCACHE_SIZE && XCHAL_DCACHE_IS_WRITEBACK
    start &= -XCHAL_DCACHE_LINESIZE;

    for (addr = start; addr < stop; addr += XCHAL_DCACHE_LINESIZE) {
        asm volatile ("dhwb %0, 0" :: "r"(addr));
    }
#endif
#if XCHAL_ICACHE_SIZE
    start &= -XCHAL_ICACHE_LINESIZE;

    for (addr = start; addr < stop; addr += XCHAL_ICACHE_LINESIZE) {
        asm volatile ("ihi %0, 0" :: "r"(addr));
    }
#endif
    asm volatile ("isync");
}
