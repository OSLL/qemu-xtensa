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
    TranslationBlock *tb;
    uint32_t pc;
    int is_jmp;
} DisasContext;

static TCGv_ptr cpu_env;
static TCGv_i32 cpu_R[16];
static TCGv_i32 cpu_pc;

#include "gen-icount.h"

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
}

static void gen_exception(int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    gen_helper_exception(tmp);
    tcg_temp_free(tmp);
}

static void disas_xtensa_insn(CPUState *env, DisasContext *dc)
{
#define HAS_OPTION(opt) do { \
        if (!(env->options & (((uint64_t)1) << (opt)))) \
            goto invalid_opcode; \
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

#define RI16_IMM16 (((_b1) << 8) | (_b2))

#define CALL_N (((_b0) & 0xc) >> 2)
#define CALL_OFFSET ((((_b0) & 0x3) << 16) | ((_b1) << 8) | (_b2))
#define CALL_OFFSET_SE (((_b0 & 0x2) ? 0xfffc0000 : 0) | CALL_OFFSET)

    uint8_t _b0 = ldub_code(dc->pc);
    uint8_t _b1 = ldub_code(dc->pc + 1);
    uint8_t _b2 = ldub_code(dc->pc + 2);

    switch (_OP0)
    {
    case 0: /*QRST*/
        switch (_OP1)
        {
        case 0: /*RST0*/
            switch (_OP2)
            {
                case 0: /*ST0*/
                    if ((RRR_R & 0xc) == 0x8)
                        HAS_OPTION(XTENSA_OPTION_BOOLEAN);

                    switch (RRR_R)
                    {
                    case 0: /*SNM0*/
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

            /* much simplified, no ext L32R, no MMU, not clear Va/Pa, no USER/SUPER */

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
        switch (CALL_N)
        {
        case 0: /*CALL0*/
            {
                tcg_gen_movi_i32(cpu_R[0], dc->pc + 3);
                tcg_gen_movi_i32(cpu_pc, (dc->pc & ~3) + (CALL_OFFSET_SE << 2) + 4);
                tcg_gen_exit_tb(0);
                dc->is_jmp = DISAS_TB_JUMP;
            }
            break;

        }
        break;

    case 6: /*SI*/
        switch (CALL_N)
        {
        case 0: /*J*/
            {
                uint32_t dest = dc->pc + 4 + CALL_OFFSET_SE;

                if ((dc->tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK)) {
                    tcg_gen_goto_tb(0);
                    tcg_gen_movi_i32(cpu_pc, dest);
                    tcg_gen_exit_tb((long)dc->tb);
                } else {
                    tcg_gen_movi_i32(cpu_pc, dest);
                    tcg_gen_exit_tb(0);
                }
                dc->is_jmp = DISAS_TB_JUMP;
            }
            break;

        }
        break;

    case 7: /*B*/
        break;

    case 8: /*L32I.Nn*/
        break;

    case 9: /*S32I.Nn*/
        break;

    case 10: /*ADD.Nn*/
        tcg_gen_add_i32(cpu_R[RRRN_R], cpu_R[RRRN_S], cpu_R[RRRN_T]);
        break;

    case 11: /*ADDI.Nn*/
        tcg_gen_addi_i32(cpu_R[RRRN_R], cpu_R[RRRN_S], RRRN_T ? RRRN_T : -1);
        break;

    case 12: /*ST2n*/
        break;

    case 13: /*ST3n*/
        break;

    default: /*reserved*/
        break;
    }

    if (_OP0 >= 8)
    {
        dc->pc += 2;
        HAS_OPTION(XTENSA_OPTION_CODE_DENSITY);
    }
    else
    {
        dc->pc += 3;
    }
    return;

invalid_opcode:
    ;
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

    if (max_insns == 0)
        max_insns = CF_COUNT_MASK;

    dc.tb = tb;
    dc.pc = env->pc;
    dc.is_jmp = DISAS_NEXT;

    gen_icount_start();

    do
    {
        if (search_pc) {
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j)
                    gen_opc_instr_start[lj++] = 0;
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
        if (env->singlestep_enabled)
        {
            tcg_gen_movi_i32(cpu_pc, dc.pc);
            gen_exception(EXCP_DEBUG);
            break;
        }
    }
    while(dc.is_jmp == DISAS_NEXT &&
            insn_count < max_insns &&
            gen_opc_ptr < gen_opc_end);

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
    int i;

    cpu_fprintf(f, "PC=%08x\n", env->pc);

    for (i = 0; i < 16; ++i)
        cpu_fprintf(f, "AR%02d=%08x%c", i, env->regs[i],
                (i % 4) == 3 ? '\n' : ' ');
}

void gen_pc_load(CPUState *env, TranslationBlock *tb,
        unsigned long searched_pc, int pc_pos, void *puc)
{
}
