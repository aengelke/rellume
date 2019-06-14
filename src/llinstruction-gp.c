/**
 * This file is part of Rellume.
 *
 * (c) 2016-2019, Alexis Engelke <alexis.engelke@googlemail.com>
 *
 * Rellume is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL)
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * Rellume is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Rellume.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file
 **/

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <llvm-c/Core.h>

#include <llinstruction-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llflags-internal.h>
#include <llinstr-internal.h>
#include <lloperand-internal.h>
#include <llregfile-internal.h>
#include <llstate-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLInstructionGP General Purpose Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

void
ll_instruction_movgp(LLInstr* instr, LLState* state)
{
    if (instr->dst.type == LL_OP_REG && instr->src.type == LL_OP_REG && instr->dst.size == 8 && instr->src.size == 8)
        ll_regfile_rename(state->regfile, instr->dst.reg, instr->src.reg);
    else
    {
        LLVMTypeRef targetType = LLVMIntTypeInContext(state->context, instr->dst.size * 8);
        LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);

        if (instr->type == LL_INS_MOVZX)
            operand1 = LLVMBuildZExtOrBitCast(state->builder, operand1, targetType, "");
        else if (instr->type == LL_INS_MOVSX) // There was a case when MOV was sign-extending, too...
            operand1 = LLVMBuildSExtOrBitCast(state->builder, operand1, targetType, "");

        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, operand1, state);
    }
}

void
ll_instruction_add(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
    operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

    LLVMValueRef result = LLVMBuildAdd(state->builder, operand1, operand2, "");

    if (LLVMGetIntTypeWidth(LLVMTypeOf(operand1)) == 64 && instr->dst.type == LL_OP_REG)
    {
        LLVMValueRef ptr = ll_get_register(instr->dst.reg, FACET_PTR, state);
        LLVMValueRef gep = LLVMBuildGEP(state->builder, ptr, &operand2, 1, "");

        ll_set_register(instr->dst.reg, FACET_I64, result, true, state);
        ll_set_register(instr->dst.reg, FACET_PTR, gep, false, state);
    }
    else
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);

    ll_flags_set_add(result, operand1, operand2, state);
}

void
ll_instruction_sub(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
    operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

    LLVMValueRef result = LLVMBuildSub(state->builder, operand1, operand2, "");

    if (LLVMGetIntTypeWidth(LLVMTypeOf(operand1)) == 64 && instr->dst.type == LL_OP_REG)
    {
        LLVMValueRef sub = LLVMBuildNeg(state->builder, operand2, "");
        LLVMValueRef ptr = ll_get_register(instr->dst.reg, FACET_PTR, state);
        LLVMValueRef gep = LLVMBuildGEP(state->builder, ptr, &sub, 1, "");

        ll_set_register(instr->dst.reg, FACET_I64, result, true, state);
        ll_set_register(instr->dst.reg, FACET_PTR, gep, false, state);
    }
    else
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);

    ll_flags_set_sub(result, operand1, operand2, state);
}

void
ll_instruction_cmp(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
    operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

    LLVMValueRef result = LLVMBuildSub(state->builder, operand1, operand2, "");

    ll_flags_set_sub(result, operand1, operand2, state);

    if (LLVMGetIntTypeWidth(LLVMTypeOf(operand1)) == 64 &&
        instr->dst.type == LL_OP_REG &&instr->src.type == LL_OP_REG)
    {
        LLVMValueRef ptr1 = ll_get_register(instr->dst.reg, FACET_PTR, state);
        LLVMValueRef ptr2 = ll_get_register(instr->src.reg, FACET_PTR, state);
        ll_set_flag(RFLAG_ZF, LLVMBuildICmp(state->builder, LLVMIntEQ, ptr1, ptr2, ""), state);
    }
}

void
ll_instruction_logical(LLInstr* instr, LLState* state, LLVMOpcode opcode)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
    LLVMValueRef result = LLVMBuildBinOp(state->builder, opcode, operand1, operand2, "");
    ll_flags_set_bit(state, result, operand1, operand2);
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
}

void
ll_instruction_test(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
    operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

    LLVMValueRef result = LLVMBuildAnd(state->builder, operand1, operand2, "");

    ll_flags_set_bit(state, result, NULL, NULL);
}

void
ll_instruction_notneg(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef result = NULL;

    if (instr->type == LL_INS_NEG)
    {
        result = LLVMBuildNeg(state->builder, operand1, "");

        LLVMValueRef zero = LLVMConstNull(LLVMTypeOf(operand1));
        LLVMValueRef cf = LLVMBuildICmp(state->builder, LLVMIntNE, operand1, zero, "");
        ll_flags_invalidate(state);
        ll_set_flag(RFLAG_CF, cf, state);
        ll_flags_set_pf(result, state);
        ll_flags_set_zf(result, state);
        ll_flags_set_sf(result, state);
        ll_flags_set_af(result, zero, operand1, state);
        ll_flags_set_of_sub(result, zero, operand1, state);
    }
    else // LL_INS_NOT
        result = LLVMBuildNot(state->builder, operand1, "");

    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
}

void
ll_instruction_incdec(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = LLVMConstInt(LLVMTypeOf(operand1), 1, false);
    LLVMValueRef result = NULL;

    if (instr->type == LL_INS_INC)
    {
        result = LLVMBuildAdd(state->builder, operand1, operand2, "");
        ll_flags_set_inc(result, operand1, state);
    }
    else // LL_INS_DEC
    {
        result = LLVMBuildSub(state->builder, operand1, operand2, "");
        ll_flags_set_dec(result, operand1, state);
    }

    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
}

void
ll_instruction_shift(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2;
    LLVMValueRef result = NULL;

    if (instr->operand_count == 1)
    {
        // DBrew decodes shifts with a implicit shift of 1 with only one operand
        operand2 = LLVMConstInt(LLVMTypeOf(operand1), 1, false);
    }
    else
    {
        operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
        // In x86, the second operand is always one byte, but in LLVM it must
        // have the same type as the first operand.
        operand2 = LLVMBuildZExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

        // x86 also masks the shift operands, depending on operand size. Note
        // that for 8/16-bit operands the mask is 0x1f as well.
        uint64_t mask_value = 0x3f;
        if (LLVMGetIntTypeWidth(LLVMTypeOf(operand1)) != 64)
        {
            mask_value = 0x1f;
        }

        LLVMValueRef mask = LLVMConstInt(LLVMTypeOf(operand1), mask_value, false);
        operand2 = LLVMBuildAnd(state->builder, operand2, mask, "");
    }

    if (instr->type == LL_INS_SHL)
    {
        result = LLVMBuildShl(state->builder, operand1, operand2, "");
        ll_flags_set_shl(state, result, operand1, operand2);
    }
    else if (instr->type == LL_INS_SHR)
    {
        result = LLVMBuildLShr(state->builder, operand1, operand2, "");
        ll_flags_set_shr(state, result, operand1, operand2);
    }
    else if (instr->type == LL_INS_SAR)
    {
        result = LLVMBuildAShr(state->builder, operand1, operand2, "");
        ll_flags_set_sar(state, result, operand1, operand2);
    }

    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
}

void
ll_instruction_mul(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1;
    LLVMValueRef operand2;
    LLVMValueRef result = NULL;

    ll_flags_invalidate(state);

    if (instr->operand_count == 1) // This covers LL_INS_MUL as well
    {
        LLVMOpcode ext = instr->type == LL_INS_IMUL ? LLVMSExt : LLVMZExt;
        LLVMOpcode shift = instr->type == LL_INS_IMUL ? LLVMAShr : LLVMLShr;
        LLVMTypeRef targetHalfType = LLVMIntTypeInContext(state->context, instr->dst.size * 8);
        LLVMTypeRef targetType = LLVMIntTypeInContext(state->context, instr->dst.size * 16);
        LLInstrOp regOp = getRegOp(ll_reg_gp(instr->dst.size, false, LL_RI_A));

        operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
        operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &regOp, state);

        LLVMValueRef largeOperand1 = LLVMBuildCast(state->builder, ext, operand1, targetType, "");
        LLVMValueRef largeOperand2 = LLVMBuildCast(state->builder, ext, operand2, targetType, "");

        result = LLVMBuildMul(state->builder, largeOperand1, largeOperand2, "");

        LLVMValueRef resultA = LLVMBuildTrunc(state->builder, result, targetHalfType, "");
        LLVMValueRef resultD = LLVMBuildBinOp(state->builder, shift, result, LLVMConstInt(targetType, instr->dst.size * 8, false), "");
        resultD = LLVMBuildTrunc(state->builder, resultD, targetHalfType, "");

        if (instr->dst.size == 1)
        {
            regOp = getRegOp(ll_reg(LL_RT_GP16, LL_RI_A));
            ll_operand_store(OP_SI, ALIGN_MAXIMUM, &regOp, REG_DEFAULT, result, state);
        }
        else
        {
            regOp = getRegOp(ll_reg_gp(instr->dst.size, false, LL_RI_A));
            ll_operand_store(OP_SI, ALIGN_MAXIMUM, &regOp, REG_DEFAULT, resultA, state);

            regOp = getRegOp(ll_reg_gp(instr->dst.size, false, LL_RI_D));
            ll_operand_store(OP_SI, ALIGN_MAXIMUM, &regOp, REG_DEFAULT, resultD, state);
        }

        if (instr->type == LL_INS_MUL)
        {
            LLVMValueRef of = LLVMBuildICmp(state->builder, LLVMIntNE, resultD, LLVMConstNull(targetHalfType), "");
            ll_set_flag(RFLAG_OF, of, state);
            ll_set_flag(RFLAG_CF, of, state);
        }
        else // LL_INS_IMUL
        {
            ll_flags_set_sf(resultA, state);
            ll_flags_set_of_imul(result, operand1, operand2, state);
        }
    }
    else if (instr->operand_count == 2)
    {
        operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
        operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
        operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");
        result = LLVMBuildMul(state->builder, operand1, operand2, "");
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
        ll_flags_set_sf(result, state);
        ll_flags_set_of_imul(result, operand1, operand2, state);
    }
    else if (instr->operand_count == 3)
    {
        operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
        operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src2, state);
        operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");
        result = LLVMBuildMul(state->builder, operand1, operand2, "");
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
        ll_flags_set_sf(result, state);
        ll_flags_set_of_imul(result, operand1, operand2, state);
    }
    else
    {
        warn_if_reached();
    }
}

void
ll_instruction_lea(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
    LLVMTypeRef targetType = LLVMIntTypeInContext(state->context, instr->dst.size * 8);
    LLVMTypeRef pi8 = LLVMPointerType(i8, 0);

    if (instr->src.type != LL_OP_MEM)
        warn_if_reached();
    if (instr->dst.type != LL_OP_REG)
        warn_if_reached();

    LLVMValueRef result = ll_operand_get_address(OP_SI, &instr->src, state);
    result = LLVMBuildPointerCast(state->builder, result, pi8, "");

    LLVMValueRef base = LLVMConstInt(i64, instr->src.val, false);

    if (instr->src.reg.rt != LL_RT_None)
        base = LLVMBuildAdd(state->builder, base, ll_get_register(instr->src.reg, FACET_I64, state), "");

    if (instr->src.scale != 0)
    {
        LLVMValueRef offset = ll_get_register(instr->src.ireg, FACET_I64, state);
        offset = LLVMBuildMul(state->builder, offset, LLVMConstInt(i64, instr->src.scale, false), "");
        base = LLVMBuildAdd(state->builder, base, offset, "");
    }

    base = LLVMBuildTruncOrBitCast(state->builder, base, targetType, "");
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, base, state);

    if (instr->dst.reg.rt == LL_RT_GP64)
        ll_set_register(instr->dst.reg, FACET_PTR, result, false, state);
}

void
ll_instruction_cmov(LLInstr* instr, LLState* state)
{
    LLVMValueRef cond = ll_flags_condition(instr->type, LL_INS_CMOVO, state);
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef result = LLVMBuildSelect(state->builder, cond, operand1, operand2, "");
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
}

void
ll_instruction_setcc(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMValueRef cond = ll_flags_condition(instr->type, LL_INS_SETO, state);
    LLVMValueRef result = LLVMBuildZExtOrBitCast(state->builder, cond, i8, "");
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
}

void
ll_instruction_cdqe(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &getRegOp(ll_reg(LL_RT_GP32, LL_RI_A)), state);
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &getRegOp(ll_reg(LL_RT_GP64, LL_RI_A)), REG_DEFAULT, operand1, state);

    (void) instr;
}

/**
 * @}
 **/
