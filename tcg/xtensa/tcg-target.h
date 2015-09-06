/*
 * Tiny Code Generator for QEMU
 *
 * Copyright (c) 2008 Fabrice Bellard
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

#ifndef TCG_TARGET_XTENSA
#define TCG_TARGET_XTENSA 1

#define TCG_TARGET_REG_BITS 32

#define TCG_TARGET_INSN_UNIT_SIZE 1
#define TCG_TARGET_TLB_DISPLACEMENT_BITS 32
#define TCG_TARGET_NB_REGS 16

typedef enum {
    TCG_REG_A0 = 0,
    TCG_REG_A1,
    TCG_REG_A2,
    TCG_REG_A3,
    TCG_REG_A4,
    TCG_REG_A5,
    TCG_REG_A6,
    TCG_REG_A7,
    TCG_REG_A8,
    TCG_REG_A9,
    TCG_REG_A10,
    TCG_REG_A11,
    TCG_REG_A12,
    TCG_REG_A13,
    TCG_REG_A14,
    TCG_REG_A15,

    TCG_REG_SP = TCG_REG_A1,
    TCG_REG_CALL_STACK = TCG_REG_SP,
    TCG_AREG0 = TCG_REG_A2,

    TCG_REG_TMP = TCG_REG_A8,
    TCG_REG_TMP1 = TCG_REG_A9,
    TCG_REG_TMP2 = TCG_REG_A12,
} TCGReg;

#define TCG_TARGET_CALL_ALIGN_ARGS      1
#define TCG_TARGET_STACK_ALIGN          16
#define TCG_TARGET_CALL_STACK_OFFSET    0

/* optional instructions */
#define TCG_TARGET_HAS_div_i32          0//XCHAL_HAVE_DIV32
#define TCG_TARGET_HAS_rem_i32          0//XCHAL_HAVE_DIV32
#define TCG_TARGET_HAS_rot_i32          1
#define TCG_TARGET_HAS_ext8s_i32        1
#define TCG_TARGET_HAS_ext16s_i32       1
#define TCG_TARGET_HAS_ext8u_i32        1
#define TCG_TARGET_HAS_ext16u_i32       1
#define TCG_TARGET_HAS_bswap16_i32      0
#define TCG_TARGET_HAS_bswap32_i32      0
#define TCG_TARGET_HAS_neg_i32          1
#define TCG_TARGET_HAS_not_i32          1
#define TCG_TARGET_HAS_andc_i32         0
#define TCG_TARGET_HAS_orc_i32          0
#define TCG_TARGET_HAS_eqv_i32          0
#define TCG_TARGET_HAS_nand_i32         0
#define TCG_TARGET_HAS_nor_i32          0
#define TCG_TARGET_HAS_deposit_i32      0
#define TCG_TARGET_HAS_movcond_i32      0
#define TCG_TARGET_HAS_add2_i32         1
#define TCG_TARGET_HAS_sub2_i32         1
#define TCG_TARGET_HAS_mulu2_i32        0
#define TCG_TARGET_HAS_muls2_i32        0
#define TCG_TARGET_HAS_muluh_i32        1
#define TCG_TARGET_HAS_mulsh_i32        1

void flush_icache_range(uintptr_t start, uintptr_t stop);

#endif
