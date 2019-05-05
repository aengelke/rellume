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
#include <llsupport-internal.h>

/**
 * \defgroup LLInstructionSSE SSE Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

void
ll_instruction_movq(LLInstr* instr, LLState* state)
{
    OperandDataType type = instr->type == LL_INS_MOVQ ? OP_SI64 : OP_SI32;
    LLVMValueRef operand1 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->src, state);

    if (instr->dst.type == LL_OP_REG && regIsV(instr->dst.reg))
        ll_operand_store(type, ALIGN_MAXIMUM, &instr->dst, REG_ZERO_UPPER_SSE, operand1, state);
    else
        ll_operand_store(type, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, operand1, state);
}

void
ll_instruction_movs(LLInstr* instr, LLState* state)
{
    OperandDataType type = instr->type == LL_INS_MOVSS ? OP_SF32 : OP_SF64;
    LLVMValueRef operand1 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->src, state);

    if (instr->src.type == LL_OP_MEM)
        ll_operand_store(type, ALIGN_MAXIMUM, &instr->dst, REG_ZERO_UPPER_SSE, operand1, state);
    else
        ll_operand_store(type, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, operand1, state);
}

void
ll_instruction_movp(LLInstr* instr, LLState* state)
{
    Alignment alignment = instr->type == LL_INS_MOVAPS || instr->type == LL_INS_MOVAPD ? ALIGN_MAXIMUM : ALIGN_8;
    OperandDataType type = instr->type == LL_INS_MOVAPS || instr->type == LL_INS_MOVUPS ? OP_VF32 : OP_VF64;

    LLVMValueRef operand1 = ll_operand_load(type, alignment, &instr->src, state);
    ll_operand_store(type, alignment, &instr->dst, REG_KEEP_UPPER, operand1, state);
}

void
ll_instruction_movdq(LLInstr* instr, LLState* state)
{
    Alignment alignment = instr->type == LL_INS_MOVDQA ? ALIGN_MAXIMUM : ALIGN_8;

    LLVMValueRef operand1 = ll_operand_load(OP_VI64, alignment, &instr->src, state);
    ll_operand_store(OP_VI64, alignment, &instr->dst, REG_KEEP_UPPER, operand1, state);
}

void
ll_instruction_movlp(LLInstr* instr, LLState* state)
{
    OperandDataType type = instr->type == LL_INS_MOVLPS ? OP_VF32 : OP_SF64;

    // XXX: Hack for XED. Should use OP_V2F32.
    instr->dst.size = 8;
    if (instr->src.type == LL_OP_REG)
        instr->src.size = 8;

    LLVMValueRef operand1 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->src, state);
    ll_operand_store(type, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, operand1, state);
}

void
ll_instruction_movhps(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);

    if (instr->dst.type == LL_OP_REG)
    {
        // XXX: Hack for XED. Even though only 64 bits are written, they are in
        // the upper half of the register.
        instr->dst.size = 16;
        instr->dst.reg.rt = LL_RT_XMM;
        // opOverwriteType(&instr->dst, VT_128);

        // XXX: Hack to make life more simple... this is actually illegal.
        instr->src.size = 16;
        // opOverwriteType(&instr->src, VT_128);

        LLVMValueRef maskElements[4];
        maskElements[0] = LLVMConstInt(i32, 0, false);
        maskElements[1] = LLVMConstInt(i32, 1, false);
        maskElements[2] = LLVMConstInt(i32, 4, false);
        maskElements[3] = LLVMConstInt(i32, 5, false);
        LLVMValueRef mask = LLVMConstVector(maskElements, 4);

        LLVMValueRef operand1 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->dst, state);
        LLVMValueRef operand2 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->src, state);
        LLVMValueRef result = LLVMBuildShuffleVector(state->builder, operand1, operand2, mask, "");
        ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, result, state);
    }
    else
    {
        // XXX: Hack for DBrew. Ensure that the destination receives <2 x float>.
        instr->dst.size = 8;
        //opOverwriteType(&instr->dst, VT_64);

        LLVMValueRef maskElements[2];
        maskElements[0] = LLVMConstInt(i32, 2, false);
        maskElements[1] = LLVMConstInt(i32, 3, false);
        LLVMValueRef mask = LLVMConstVector(maskElements, 2);

        LLVMValueRef operand1 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->src, state);
        LLVMValueRef result = LLVMBuildShuffleVector(state->builder, operand1, LLVMGetUndef(LLVMTypeOf(operand1)), mask, "");
        ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, result, state);
    }
}

void
ll_instruction_movhpd(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);

    if (instr->dst.type == LL_OP_REG)
    {
        // XXX: Hack for XED. Even though only 64 bits are written, they are in
        // the upper half of the register.
        instr->dst.size = 16;
        instr->dst.reg.rt = LL_RT_XMM;
        // opOverwriteType(&instr->dst, VT_128);

        LLVMValueRef operand1 = ll_operand_load(OP_VF64, ALIGN_MAXIMUM, &instr->dst, state);
        LLVMValueRef operand2 = ll_operand_load(OP_SF64, ALIGN_MAXIMUM, &instr->src, state);
        LLVMValueRef result = LLVMBuildInsertElement(state->builder, operand1, operand2, LLVMConstInt(i32, 1, false), "");
        ll_operand_store(OP_VF64, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, result, state);
    }
    else
    {
        // XXX: Hack for XED. Even though only 64 bits are written, they are in
        // the upper half of the register.
        instr->dst.size = 16;
        // opOverwriteType(&instr->src, VT_128);

        LLVMValueRef operand1 = ll_operand_load(OP_VF64, ALIGN_MAXIMUM, &instr->src, state);
        LLVMValueRef result = LLVMBuildExtractElement(state->builder, operand1, LLVMConstInt(i32, 1, false), "");
        ll_operand_store(OP_SF64, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, result, state);
    }
}

void
ll_instruction_unpckl(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);
    OperandDataType type = instr->type == LL_INS_UNPCKLPS ? OP_VF32 : OP_VF64;

    LLVMValueRef maskElements[4];
    LLVMValueRef mask;

    // This is actually legal as an implementation "MAY only fetch 64-bit".
    // See Intel SDM Vol. 2B 4-696 (Dec. 2016).
    instr->src.size = 16;
    if (instr->src.type == LL_OP_REG)
        instr->src.reg.rt = LL_RT_XMM;
    // opOverwriteType(&instr->src, VT_128);

    if (instr->type == LL_INS_UNPCKLPS)
    {
        maskElements[0] = LLVMConstInt(i32, 0, false);
        maskElements[1] = LLVMConstInt(i32, 4, false);
        maskElements[2] = LLVMConstInt(i32, 1, false);
        maskElements[3] = LLVMConstInt(i32, 5, false);
        mask = LLVMConstVector(maskElements, 4);
    }
    else // LL_INS_UNPCKLPD
    {
        maskElements[0] = LLVMConstInt(i32, 0, false);
        maskElements[1] = LLVMConstInt(i32, 2, false);
        mask = LLVMConstVector(maskElements, 2);
    }

    LLVMValueRef operand1 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->dst, state);
    LLVMValueRef operand2 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->src, state);
    LLVMValueRef result = LLVMBuildShuffleVector(state->builder, operand1, operand2, mask, "");
    ll_operand_store(type, ALIGN_MAXIMUM, &instr->dst, REG_KEEP_UPPER, result, state);
}

/**
 * @}
 **/
