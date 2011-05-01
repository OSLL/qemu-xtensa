/*
 * Copyright (c) 2011, Max Filippov, Motorola Solutions, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Motorola Solutions nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include "cpu.h"
#include "exec-all.h"
#include "disas.h"
#include "tcg-op.h"
#include "qemu-log.h"

#include "helpers.h"
#define GEN_HELPER 1
#include "helpers.h"

typedef struct DisasContext {
    CPUState *env;
    TranslationBlock *tb;
    uint32_t pc;
    int is_jmp;
} DisasContext;

static TCGv_ptr cpu_env;
static TCGv_i32 cpu_pc;
static TCGv_i32 cpu_R[16];
static TCGv_i32 cpu_SR[256];
static TCGv_i32 cpu_UR[256];

#include "gen-icount.h"

static const char * const sregnames[256] = {
};

static const char * const uregnames[256] = {
    [THREADPTR] = "THREADPTR",
    [FCR] = "FCR",
    [FSR] = "FSR",
};

void xtensa_translate_init(void)
{
    static const char * const regnames[] = {
        "ar0", "ar1", "ar2", "ar3",
        "ar4", "ar5", "ar6", "ar7",
        "ar8", "ar9", "ar10", "ar11",
        "ar12", "ar13", "ar14", "ar15",
    };
    int i;

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");
    cpu_pc = tcg_global_mem_new_i32(TCG_AREG0,
            offsetof(CPUState, pc), "pc");

    for (i = 0; i < 16; i++) {
        cpu_R[i] = tcg_global_mem_new_i32(TCG_AREG0,
                offsetof(CPUState, regs[i]),
                regnames[i]);
    }

    for (i = 0; i < 256; ++i) {
        if (sregnames[i]) {
            cpu_SR[i] = tcg_global_mem_new_i32(TCG_AREG0,
                    offsetof(CPUState, sregs[i]),
                    sregnames[i]);
        }
    }

    for (i = 0; i < 256; ++i) {
        if (uregnames[i]) {
            cpu_UR[i] = tcg_global_mem_new_i32(TCG_AREG0,
                    offsetof(CPUState, uregs[i]),
                    uregnames[i]);
        }
    }
}

static void gen_rsr(TCGv_i32 d, int sr)
{
    if (sregnames[sr]) {
        tcg_gen_mov_i32(d, cpu_SR[sr]);
    } else {
        printf("SR %d not implemented, ", sr);
    }
}

static void gen_wsr(DisasContext *dc, uint32_t sr, TCGv_i32 s)
{
    static void (* const wsr_handler[256])(DisasContext *dc,
            uint32_t sr, TCGv_i32 v) = {
    };

    if (sregnames[sr]) {
        if (wsr_handler[sr]) {
            wsr_handler[sr](dc, sr, s);
        } else {
            tcg_gen_mov_i32(cpu_SR[sr], s);
        }
    } else {
        printf("SR %d not implemented, ", sr);
    }
}

static void gen_exception(int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    gen_helper_exception(tmp);
    tcg_temp_free(tmp);
}

static void gen_jump(DisasContext *dc, TCGv dest)
{
    tcg_gen_mov_i32(cpu_pc, dest);
    if (dc->env->singlestep_enabled) {
        gen_exception(EXCP_DEBUG);
    }
    tcg_gen_exit_tb(0);
    dc->is_jmp = DISAS_UPDATE;
}

static void gen_jumpi(DisasContext *dc, uint32_t dest)
{
    TCGv_i32 tmp = tcg_const_i32(dest);
    gen_jump(dc, tmp);
    tcg_temp_free(tmp);
}

static void disas_xtensa_insn(CPUState *env, DisasContext *dc)
{
#define HAS_OPTION(opt) do { \
        if (!(env->options & (((uint64_t)1) << (opt)))) { \
            goto invalid_opcode; \
        } \
    } while (0)

#define _OP0 (((_b0) & 0xf0) >> 4)
#define _OP1 (((_b2) & 0xf0) >> 4)
#define _OP2 ((_b2) & 0xf)
#define RRR_R ((_b1) & 0xf)
#define RRR_S (((_b1) & 0xf0) >> 4)
#define RRR_T ((_b0) & 0xf)

#define RRRN_R RRR_R
#define RRRN_S RRR_S
#define RRRN_T RRR_T

#define RRI8_R RRR_R
#define RRI8_S RRR_S
#define RRI8_T RRR_T
#define RRI8_IMM8 (_b2)
#define RRI8_IMM8_SE ((((_b2) & 0x80) ? 0xffffff00 : 0) | RRI8_IMM8)

#define RI16_IMM16 (((_b1) << 8) | (_b2))

#define CALL_N (((_b0) & 0xc) >> 2)
#define CALL_OFFSET ((((_b0) & 0x3) << 16) | ((_b1) << 8) | (_b2))
#define CALL_OFFSET_SE (((_b0 & 0x2) ? 0xfffc0000 : 0) | CALL_OFFSET)

#define CALLX_N CALL_N
#define CALLX_M ((_b0) & 0x3)
#define CALLX_S RRR_S

#define BRI12_M CALLX_M
#define BRI12_S RRR_S
#define BRI12_IMM12 ((((_b1) & 0xf) << 8) | (_b2))
#define BRI12_IMM12_SE ((((_b1) & 0x8) ? 0xfffff000 : 0) | BRI12_IMM12)

#define BRI8_M BRI12_M
#define BRI8_R RRI8_R
#define BRI8_S RRI8_S
#define BRI8_IMM8 RRI8_IMM8
#define BRI8_IMM8_SE RRI8_IMM8_SE

#define RSR_SR (_b1)

    uint8_t _b0 = ldub_code(dc->pc);
    uint8_t _b1 = ldub_code(dc->pc + 1);
    uint8_t _b2 = ldub_code(dc->pc + 2);

    static const uint32_t B4CONST[] = {
        0xffffffff, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
    };

    static const uint32_t B4CONSTU[] = {
        32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
    };

    switch (_OP0) {
    case 0: /*QRST*/
        switch (_OP1) {
        case 0: /*RST0*/
            switch (_OP2) {
            case 0: /*ST0*/
                if ((RRR_R & 0xc) == 0x8) {
                    HAS_OPTION(XTENSA_OPTION_BOOLEAN);
                }

                switch (RRR_R) {
                case 0: /*SNM0*/
                    switch (CALLX_M) {
                    case 0: /*ILL*/
                        break;

                    case 1: /*reserved*/
                        break;

                    case 2: /*JR*/
                        switch (CALLX_N) {
                        case 0: /*RET*/
                        case 2: /*JX*/
                            gen_jump(dc, cpu_R[CALLX_S]);
                            break;

                        case 1: /*RETWw*/
                            HAS_OPTION(XTENSA_OPTION_WINDOWED_REGISTER);
                            break;

                        case 3: /*reserved*/
                            break;
                        }
                        break;

                    case 3: /*CALLX*/
                        switch (CALLX_N) {
                        case 0: /*CALLX0*/
                            {
                                TCGv_i32 tmp = tcg_temp_new_i32();
                                tcg_gen_mov_i32(tmp, cpu_R[CALLX_S]);
                                tcg_gen_movi_i32(cpu_R[0], dc->pc + 3);
                                gen_jump(dc, tmp);
                                tcg_temp_free(tmp);
                            }
                            break;

                        case 1: /*CALLX4w*/
                        case 2: /*CALLX8w*/
                        case 3: /*CALLX12w*/
                            HAS_OPTION(XTENSA_OPTION_WINDOWED_REGISTER);
                            break;
                        }
                        break;
                    }
                    break;

                case 1: /*MOVSPw*/
                    HAS_OPTION(XTENSA_OPTION_WINDOWED_REGISTER);
                    break;

                case 2: /*SYNC*/
                    break;

                case 3:
                    break;

                }
                break;

            case 1: /*AND*/
                tcg_gen_and_i32(cpu_R[RRR_R], cpu_R[RRR_S], cpu_R[RRR_T]);
                break;

            case 2: /*OR*/
                tcg_gen_or_i32(cpu_R[RRR_R], cpu_R[RRR_S], cpu_R[RRR_T]);
                break;

            case 3: /*XOR*/
                tcg_gen_xor_i32(cpu_R[RRR_R], cpu_R[RRR_S], cpu_R[RRR_T]);
                break;

            case 4: /*ST1*/
                break;

            case 5: /*TLB*/
                break;

            case 6: /*RT0*/
                switch (RRR_S) {
                case 0: /*NEG*/
                    tcg_gen_neg_i32(cpu_R[RRR_R], cpu_R[RRR_T]);
                    break;

                case 1: /*ABS*/
                    {
                        int label = gen_new_label();
                        tcg_gen_mov_i32(cpu_R[RRR_R], cpu_R[RRR_T]);
                        tcg_gen_brcondi_i32(
                                TCG_COND_GE, cpu_R[RRR_R], 0, label);
                        tcg_gen_neg_i32(cpu_R[RRR_R], cpu_R[RRR_T]);
                        gen_set_label(label);
                    }
                    break;

                default: /*reserved*/
                    break;
                }
                break;

            case 7: /*reserved*/
                break;

            case 8: /*ADD*/
                tcg_gen_add_i32(cpu_R[RRR_R], cpu_R[RRR_S], cpu_R[RRR_T]);
                break;

            case 9: /*ADD**/
            case 10:
            case 11:
                {
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_shli_i32(tmp, cpu_R[RRR_S], _OP2 - 8);
                    tcg_gen_add_i32(cpu_R[RRR_R], tmp, cpu_R[RRR_T]);
                    tcg_temp_free(tmp);
                }
                break;

            case 12: /*SUB*/
                tcg_gen_sub_i32(cpu_R[RRR_R], cpu_R[RRR_S], cpu_R[RRR_T]);
                break;

            case 13: /*SUB**/
            case 14:
            case 15:
                {
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_shli_i32(tmp, cpu_R[RRR_S], _OP2 - 12);
                    tcg_gen_sub_i32(cpu_R[RRR_R], tmp, cpu_R[RRR_T]);
                    tcg_temp_free(tmp);
                }
                break;
            }
            break;

        case 1: /*RST1*/
            break;

        case 2: /*RST2*/
            break;

        case 3: /*RST3*/
            switch (_OP2) {
            case 0: /*RSR*/
                gen_rsr(cpu_R[RRR_T], RSR_SR);
                break;

            case 1: /*WSR*/
                gen_wsr(dc, RSR_SR, cpu_R[RRR_T]);
                break;

            case 2: /*SEXTu*/
                HAS_OPTION(XTENSA_OPTION_MISC_OP);
                {
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_shli_i32(tmp, cpu_R[RRR_S], 24 - RRR_T);
                    tcg_gen_sari_i32(cpu_R[RRR_R], tmp, 24 - RRR_T);
                    tcg_temp_free(tmp);
                }
                break;

            case 3: /*CLAMPSu*/
                HAS_OPTION(XTENSA_OPTION_MISC_OP);
                {
                    TCGv_i32 tmp1 = tcg_temp_new_i32();
                    TCGv_i32 tmp2 = tcg_temp_new_i32();
                    int label = gen_new_label();

                    tcg_gen_sari_i32(tmp1, cpu_R[RRR_S], 24 - RRR_T);
                    tcg_gen_xor_i32(tmp2, tmp1, cpu_R[RRR_S]);
                    tcg_gen_andi_i32(tmp2, tmp2, 0xffffffff << (RRR_T + 7));
                    tcg_gen_mov_i32(cpu_R[RRR_R], cpu_R[RRR_S]);
                    tcg_gen_brcondi_i32(TCG_COND_EQ, tmp2, 0, label);

                    tcg_gen_sari_i32(tmp1, cpu_R[RRR_S], 31);
                    tcg_gen_xori_i32(cpu_R[RRR_R], tmp1,
                            0xffffffff >> (25 - RRR_T));

                    gen_set_label(label);

                    tcg_temp_free(tmp1);
                    tcg_temp_free(tmp2);
                }
                break;

            case 4: /*MINu*/
            case 5: /*MAXu*/
            case 6: /*MINUu*/
            case 7: /*MAXUu*/
                HAS_OPTION(XTENSA_OPTION_MISC_OP);
                {
                    static const TCGCond cond[] = {
                        TCG_COND_LE,
                        TCG_COND_GE,
                        TCG_COND_LEU,
                        TCG_COND_GEU
                    };
                    int label = gen_new_label();

                    if (RRR_R != RRR_T) {
                        tcg_gen_mov_i32(cpu_R[RRR_R], cpu_R[RRR_S]);
                        tcg_gen_brcond_i32(cond[_OP2 - 4],
                                cpu_R[RRR_S], cpu_R[RRR_T], label);
                        tcg_gen_mov_i32(cpu_R[RRR_R], cpu_R[RRR_T]);
                    } else {
                        tcg_gen_brcond_i32(cond[_OP2 - 4],
                                cpu_R[RRR_T], cpu_R[RRR_S], label);
                        tcg_gen_mov_i32(cpu_R[RRR_R], cpu_R[RRR_S]);
                    }
                    gen_set_label(label);
                }
                break;

            case 8: /*MOVEQZ*/
            case 9: /*MOVNEZ*/
            case 10: /*MOVLTZ*/
            case 11: /*MOVGEZ*/
                {
                    static const TCGCond cond[] = {
                        TCG_COND_NE,
                        TCG_COND_EQ,
                        TCG_COND_GE,
                        TCG_COND_LT
                    };
                    int label = gen_new_label();
                    tcg_gen_brcondi_i32(cond[_OP2 - 8], cpu_R[RRR_T], 0, label);
                    tcg_gen_mov_i32(cpu_R[RRR_R], cpu_R[RRR_S]);
                    gen_set_label(label);
                }
                break;

            case 12: /*MOVFp*/
                HAS_OPTION(XTENSA_OPTION_BOOLEAN);
                break;

            case 13: /*MOVTp*/
                HAS_OPTION(XTENSA_OPTION_BOOLEAN);
                break;

            case 14: /*RUR*/
                {
                    int st = (RRR_S << 4) + RRR_T;
                    if (uregnames[st]) {
                        tcg_gen_mov_i32(cpu_R[RRR_R], cpu_UR[st]);
                    } else {
                        printf("rur %d not implemented, ", st);
                    }
                }
                break;

            case 15: /*WUR*/
                {
                    if (uregnames[RSR_SR]) {
                        tcg_gen_mov_i32(cpu_UR[RSR_SR], cpu_R[RRR_T]);
                    } else {
                        printf("wur %d not implemented, ", RSR_SR);
                    }
                }
                break;

            }
            break;

        case 4: /*EXTUI*/
        case 5:
            break;

        case 6: /*CUST0*/
            break;

        case 7: /*CUST1*/
            break;

        case 8: /*LSCXp*/
            HAS_OPTION(XTENSA_OPTION_COPROCESSOR);
            break;

        case 9: /*LSC4*/
            break;

        case 10: /*FP0*/
            HAS_OPTION(XTENSA_OPTION_FP_COPROCESSOR);
            break;

        case 11: /*FP1*/
            HAS_OPTION(XTENSA_OPTION_FP_COPROCESSOR);
            break;

        default: /*reserved*/
            break;
        }
        break;

    case 1: /*L32R*/
        {
            TCGv_i32 tmp = tcg_const_i32(
                    (0xfffc0000 | (RI16_IMM16 << 2)) +
                    ((dc->pc + 3) & ~3));

            /* no ext L32R */

            tcg_gen_qemu_ld32u(cpu_R[RRR_T], tmp, 0);
            tcg_temp_free(tmp);
        }
        break;

    case 2: /*LSAI*/
        break;

    case 3: /*LSCIp*/
        HAS_OPTION(XTENSA_OPTION_COPROCESSOR);
        break;

    case 4: /*MAC16d*/
        HAS_OPTION(XTENSA_OPTION_MAC16);
        break;

    case 5: /*CALLN*/
        switch (CALL_N) {
        case 0: /*CALL0*/
            tcg_gen_movi_i32(cpu_R[0], dc->pc + 3);
            gen_jumpi(dc, (dc->pc & ~3) + (CALL_OFFSET_SE << 2) + 4);
            break;

        case 1: /*CALL4w*/
        case 2: /*CALL8w*/
        case 3: /*CALL12w*/
            HAS_OPTION(XTENSA_OPTION_WINDOWED_REGISTER);
            break;
        }
        break;

    case 6: /*SI*/
        switch (CALL_N) {
        case 0: /*J*/
            gen_jumpi(dc, dc->pc + 4 + CALL_OFFSET_SE);
            break;

        case 1: /*BZ*/
            {
                int label = gen_new_label();
                int inv = BRI12_M & 1;

                switch (BRI12_M & 2) {
                case 0: /*BEQZ*/
                    tcg_gen_brcondi_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                            cpu_R[BRI12_S], 0, label);
                    break;

                case 2: /*BLTZ*/
                    tcg_gen_brcondi_i32(inv ? TCG_COND_LT : TCG_COND_GE,
                            cpu_R[BRI12_S], 0, label);
                    break;
                }
                gen_jumpi(dc, dc->pc + 4 + BRI12_IMM12_SE);
                gen_set_label(label);
                gen_jumpi(dc, dc->pc + 3);
            }
            break;

        case 2: /*BI0*/
            {
                int label = gen_new_label();
                int inv = BRI8_M & 1;

                switch (BRI8_M & 2) {
                case 0: /*BEQI*/
                    tcg_gen_brcondi_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                            cpu_R[BRI8_S], B4CONST[BRI8_R], label);
                    break;

                case 2: /*BLTI*/
                    tcg_gen_brcondi_i32(inv ? TCG_COND_LT : TCG_COND_GE,
                            cpu_R[BRI8_S], B4CONST[BRI8_R], label);
                    break;
                }
                gen_jumpi(dc, dc->pc + 4 + BRI8_IMM8_SE);
                gen_set_label(label);
                gen_jumpi(dc, dc->pc + 3);
            }
            break;

        case 3: /*BI1*/
            switch (BRI8_M) {
            case 0: /*ENTRYw*/
                HAS_OPTION(XTENSA_OPTION_WINDOWED_REGISTER);
                break;

            case 1: /*B1*/
                switch (BRI8_R) {
                case 0: /*BFp*/
                    HAS_OPTION(XTENSA_OPTION_BOOLEAN);
                    break;

                case 1: /*BTp*/
                    HAS_OPTION(XTENSA_OPTION_BOOLEAN);
                    break;

                case 8: /*LOOP*/
                    break;

                case 9: /*LOOPNEZ*/
                    break;

                case 10: /*LOOPGTZ*/
                    break;

                default: /*reserved*/
                    break;

                }
                break;

            case 2: /*BLTUI*/
            case 3: /*BGEUI*/
                {
                    int label = gen_new_label();
                    int inv = BRI8_M & 1;

                    tcg_gen_brcondi_i32(inv ? TCG_COND_LTU : TCG_COND_GEU,
                            cpu_R[BRI8_S], B4CONSTU[BRI8_R], label);

                    gen_jumpi(dc, dc->pc + 4 + BRI8_IMM8_SE);
                    gen_set_label(label);
                    gen_jumpi(dc, dc->pc + 3);
                }
                break;
            }
            break;

        }
        break;

    case 7: /*B*/
        {
            int label = gen_new_label();
            int inv = RRI8_R & 8;

            switch (RRI8_R & 7) {
            case 0: /*BNONE*/
                {
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_and_i32(tmp, cpu_R[RRI8_S], cpu_R[RRI8_T]);
                    tcg_gen_brcondi_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                            tmp, 0, label);
                    tcg_temp_free(tmp);
                }
                break;

            case 1: /*BEQ*/
                tcg_gen_brcond_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                        cpu_R[RRI8_S], cpu_R[RRI8_T], label);
                break;

            case 2: /*BLT*/
                tcg_gen_brcond_i32(inv ? TCG_COND_LT : TCG_COND_GE,
                        cpu_R[RRI8_S], cpu_R[RRI8_T], label);
                break;

            case 3: /*BLTU*/
                tcg_gen_brcond_i32(inv ? TCG_COND_LTU : TCG_COND_GEU,
                        cpu_R[RRI8_S], cpu_R[RRI8_T], label);
                break;

            case 4: /*BALL*/
                {
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_and_i32(tmp, cpu_R[RRI8_S], cpu_R[RRI8_T]);
                    tcg_gen_brcond_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                            tmp, cpu_R[RRI8_T], label);
                    tcg_temp_free(tmp);
                }
                break;

            case 5: /*BBC*/
                {
                    TCGv_i32 bit = tcg_const_i32(1);
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_andi_i32(tmp, cpu_R[RRI8_T], 0x1f);
                    tcg_gen_shl_i32(bit, bit, tmp);
                    tcg_gen_and_i32(tmp, cpu_R[RRI8_S], bit);
                    tcg_gen_brcondi_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                            tmp, 0, label);
                    tcg_temp_free(tmp);
                    tcg_temp_free(bit);
                }
                break;

            case 6: /*BBCI*/
            case 7:
                {
                    TCGv_i32 tmp = tcg_temp_new_i32();
                    tcg_gen_andi_i32(tmp, cpu_R[RRI8_S],
                            1 << (((RRI8_R & 1) << 4) | RRI8_T));
                    tcg_gen_brcondi_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                            tmp, 0, label);
                    tcg_temp_free(tmp);
                }
                break;

            }
            gen_jumpi(dc, dc->pc + 4 + RRI8_IMM8_SE);
            gen_set_label(label);
            gen_jumpi(dc, dc->pc + 3);
        }
        break;

#define gen_narrow_load_store(type) \
        { \
            TCGv_i32 addr = tcg_temp_new_i32(); \
            tcg_gen_addi_i32(addr, cpu_R[RRRN_S], RRRN_R << 2); \
            tcg_gen_qemu_##type(cpu_R[RRRN_T], addr, 0); \
            tcg_temp_free(addr); \
        }
    case 8: /*L32I.Nn*/
        gen_narrow_load_store(ld32u);
        break;

    case 9: /*S32I.Nn*/
        gen_narrow_load_store(st32);
        break;
#undef gen_narrow_load_store

    case 10: /*ADD.Nn*/
        tcg_gen_add_i32(cpu_R[RRRN_R], cpu_R[RRRN_S], cpu_R[RRRN_T]);
        break;

    case 11: /*ADDI.Nn*/
        tcg_gen_addi_i32(cpu_R[RRRN_R], cpu_R[RRRN_S], RRRN_T ? RRRN_T : -1);
        break;

    case 12: /*ST2n*/
        if (RRRN_T < 8) { /*MOVI.Nn*/
            tcg_gen_movi_i32(cpu_R[RRRN_S],
                    RRRN_R | (RRRN_T << 4) |
                    ((RRRN_T & 6) == 6 ? 0xffffff80 : 0));
        } else { /*BEQZ.Nn*/ /*BNEZ.Nn*/
            int label = gen_new_label();
            int inv = RRRN_T & 4;

            tcg_gen_brcondi_i32(inv ? TCG_COND_EQ : TCG_COND_NE,
                    cpu_R[RRRN_S], 0, label);
            gen_jumpi(dc, dc->pc + 4 + (RRRN_R | ((RRRN_T & 3) << 4)));
            gen_set_label(label);
            gen_jumpi(dc, dc->pc + 2);
        }
        break;

    case 13: /*ST3n*/
        switch (RRRN_R) {
        case 0: /*MOV.Nn*/
            tcg_gen_mov_i32(cpu_R[RRRN_T], cpu_R[RRRN_S]);
            break;

        case 15: /*S3*/
            switch (RRRN_T) {
            case 0: /*RET.Nn*/
                gen_jump(dc, cpu_R[0]);
                break;

            case 1: /*RETW.Nn*/
                break;

            case 2: /*BREAK.Nn*/
                break;

            case 3: /*NOP.Nn*/
                break;

            case 6: /*ILL.Nn*/
                break;

            default: /*reserved*/
                break;
            }
            break;

        default: /*reserved*/
            break;
        }
        break;

    default: /*reserved*/
        break;
    }

    if (_OP0 >= 8) {
        dc->pc += 2;
        HAS_OPTION(XTENSA_OPTION_CODE_DENSITY);
    } else {
        dc->pc += 3;
    }
    return;

invalid_opcode:
    printf("INVALID(pc = %08x): %s:%d\n", dc->pc, __FILE__, __LINE__);
    dc->pc += (_OP0 >= 8) ? 2 : 3;
#undef HAS_OPTION
}

static void gen_intermediate_code_internal(
        CPUState *env, TranslationBlock *tb, int search_pc)
{
    DisasContext dc;
    int insn_count = 0;
    int j, lj = -1;
    uint16_t *gen_opc_end = gen_opc_buf + OPC_MAX_SIZE;
    int max_insns = tb->cflags & CF_COUNT_MASK;

    if (max_insns == 0) {
        max_insns = CF_COUNT_MASK;
    }

    dc.env = env;
    dc.tb = tb;
    dc.pc = env->pc;
    dc.is_jmp = DISAS_NEXT;

    gen_icount_start();

    do {
        if (search_pc) {
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j) {
                    gen_opc_instr_start[lj++] = 0;
                }
            }
            gen_opc_pc[lj] = dc.pc;
            gen_opc_instr_start[lj] = 1;
            gen_opc_icount[lj] = insn_count;
        }

        if (unlikely(qemu_loglevel_mask(CPU_LOG_TB_OP))) {
            tcg_gen_debug_insn_start(dc.pc);
        }

        disas_xtensa_insn(env, &dc);
        ++insn_count;
        if (env->singlestep_enabled) {
            tcg_gen_movi_i32(cpu_pc, dc.pc);
            gen_exception(EXCP_DEBUG);
            break;
        }
    } while (dc.is_jmp == DISAS_NEXT &&
            insn_count < max_insns &&
            gen_opc_ptr < gen_opc_end);

    if (dc.is_jmp == DISAS_NEXT) {
        tcg_gen_movi_i32(cpu_pc, dc.pc);
        tcg_gen_exit_tb(0);
    }
    gen_icount_end(tb, insn_count);
    *gen_opc_ptr = INDEX_op_end;

}

void gen_intermediate_code(CPUState *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 0);
}

void gen_intermediate_code_pc(CPUState *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 1);
}

void cpu_dump_state(CPUState *env, FILE *f, fprintf_function cpu_fprintf,
        int flags)
{
    int i, j;

    cpu_fprintf(f, "PC=%08x\n\n", env->pc);

    for (i = j = 0; i < 256; ++i)
        if (sregnames[i]) {
            cpu_fprintf(f, "%s=%08x%c", sregnames[i], env->sregs[i],
                    (j++ % 4) == 3 ? '\n' : ' ');
        }

    cpu_fprintf(f, (j % 4) == 0 ? "\n" : "\n\n");

    for (i = j = 0; i < 256; ++i)
        if (uregnames[i]) {
            cpu_fprintf(f, "%s=%08x%c", uregnames[i], env->uregs[i],
                    (j++ % 4) == 3 ? '\n' : ' ');
        }

    cpu_fprintf(f, (j % 4) == 0 ? "\n" : "\n\n");

    for (i = 0; i < 16; ++i)
        cpu_fprintf(f, "A%02d=%08x%c", i, env->regs[i],
                (i % 4) == 3 ? '\n' : ' ');
}

void gen_pc_load(CPUState *env, TranslationBlock *tb,
        unsigned long searched_pc, int pc_pos, void *puc)
{
}
