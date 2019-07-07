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

#include <cstdbool>
#include <cstdint>
#include <llvm-c/Core.h>

#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>

#include <llinstruction-internal.h>
#include <llstate-internal.h>

#include <llcommon-internal.h>
#include <llinstr-internal.h>

/**
 * \defgroup LLInstructionSSE SSE Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

void LLState::LiftSseMovq(const LLInstr& inst, Facet::Value type)
{
    llvm::Value* op1 = OpLoad(inst.ops[1], type);
    if (inst.ops[0].type == LL_OP_REG && inst.ops[0].reg.IsVec())
    {
        llvm::Type* el_ty = op1->getType();
        llvm::Type* vector_ty = llvm::VectorType::get(el_ty, 128 / el_ty->getPrimitiveSizeInBits());
        llvm::Value* zero = llvm::Constant::getNullValue(vector_ty);
        llvm::Value* zext = irb.CreateInsertElement(zero, op1, 0ul);
        OpStoreVec(inst.ops[0], zext);
    }
    else
    {
        OpStoreGp(inst.ops[0], op1);
    }
}

void
ll_instruction_movs(LLInstr* instr, LLState* state)
{
    OperandDataType type = instr->type == LL_INS_MOVSS ? OP_SF32 : OP_SF64;
    LLVMValueRef operand1 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->ops[1], state);

    if (instr->ops[1].type == LL_OP_MEM)
    {
        llvm::Type* el_ty = llvm::unwrap(operand1)->getType();
        llvm::Type* vector_ty = llvm::VectorType::get(el_ty, 128 / el_ty->getPrimitiveSizeInBits());
        llvm::Value* zero = llvm::Constant::getNullValue(vector_ty);
        llvm::Value* zext = state->irb.CreateInsertElement(zero, llvm::unwrap(operand1), 0ul);
        state->OpStoreVec(instr->ops[0], zext);
    }
    else
    {
        ll_operand_store(type, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, operand1, state);
    }
}

void
ll_instruction_movp(LLInstr* instr, LLState* state)
{
    Alignment alignment = instr->type == LL_INS_MOVAPS || instr->type == LL_INS_MOVAPD ? ALIGN_MAX : ALIGN_NONE;
    OperandDataType type = instr->type == LL_INS_MOVAPS || instr->type == LL_INS_MOVUPS ? OP_VF32 : OP_VF64;

    LLVMValueRef operand1 = ll_operand_load(type, alignment, &instr->ops[1], state);
    ll_operand_store(type, alignment, &instr->ops[0], REG_KEEP_UPPER, operand1, state);
}

void
ll_instruction_movdq(LLInstr* instr, LLState* state)
{
    Alignment alignment = instr->type == LL_INS_MOVDQA ? ALIGN_MAX : ALIGN_NONE;

    LLVMValueRef operand1 = ll_operand_load(OP_VI64, alignment, &instr->ops[1], state);
    ll_operand_store(OP_VI64, alignment, &instr->ops[0], REG_KEEP_UPPER, operand1, state);
}

void
ll_instruction_movlp(LLInstr* instr, LLState* state)
{
    if (instr->ops[0].type == LL_OP_REG && instr->ops[1].type == LL_OP_REG)
    {
        // move high 64-bit from src to low 64-bit from dst
        if (instr->type != LL_INS_MOVLPS)
            warn_if_reached();

        llvm::Value* operand1 = llvm::unwrap(ll_operand_load(OP_V4F32, ALIGN_MAXIMUM, &instr->ops[0], state));
        llvm::Value* operand2 = llvm::unwrap(ll_operand_load(OP_V4F32, ALIGN_MAXIMUM, &instr->ops[1], state));

        llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
        llvm::Value* result = builder->CreateShuffleVector(operand1, operand2, {6, 7, 2, 3});
        ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, llvm::wrap(result), state);
    }
    else
    {
        // move (low) 64-bit from src to (low) 64-bit from dst
        OperandDataType type = instr->type == LL_INS_MOVLPS ? OP_V2F32 : OP_V1F64;
        LLVMValueRef operand1 = ll_operand_load(type, ALIGN_MAXIMUM, &instr->ops[1], state);
        ll_operand_store(type, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, operand1, state);
    }
}

void
ll_instruction_movhps(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);

    if (instr->ops[0].type == LL_OP_REG)
    {
        // XXX: Hack for XED. Even though only 64 bits are written, they are in
        // the upper half of the register.
        instr->ops[0].size = 16;
        instr->ops[0].reg.rt = LL_RT_XMM;
        // opOverwriteType(&instr->ops[0], VT_128);

        // XXX: Hack to make life more simple... this is actually illegal.
        instr->ops[1].size = 16;
        // opOverwriteType(&instr->ops[1], VT_128);

        LLVMValueRef maskElements[4];
        maskElements[0] = LLVMConstInt(i32, 0, false);
        maskElements[1] = LLVMConstInt(i32, 1, false);
        maskElements[2] = LLVMConstInt(i32, 4, false);
        maskElements[3] = LLVMConstInt(i32, 5, false);
        LLVMValueRef mask = LLVMConstVector(maskElements, 4);

        LLVMValueRef operand1 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], state);
        // The source memory operand does not need to be aligned.
        // FIXME: actually remove the hack above...
        LLVMValueRef operand2 = ll_operand_load(OP_VF32, ALIGN_1, &instr->ops[1], state);
        LLVMValueRef result = LLVMBuildShuffleVector(state->builder, operand1, operand2, mask, "");
        ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, result, state);
    }
    else
    {
        // XXX: Hack for DBrew. Ensure that the destination receives <2 x float>.
        instr->ops[0].size = 8;
        //opOverwriteType(&instr->ops[0], VT_64);

        LLVMValueRef maskElements[2];
        maskElements[0] = LLVMConstInt(i32, 2, false);
        maskElements[1] = LLVMConstInt(i32, 3, false);
        LLVMValueRef mask = LLVMConstVector(maskElements, 2);

        LLVMValueRef operand1 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->ops[1], state);
        LLVMValueRef result = LLVMBuildShuffleVector(state->builder, operand1, LLVMGetUndef(LLVMTypeOf(operand1)), mask, "");
        ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, result, state);
    }
}

void
ll_instruction_movhpd(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);

    if (instr->ops[0].type == LL_OP_REG)
    {
        // XXX: Hack for XED. Even though only 64 bits are written, they are in
        // the upper half of the register.
        instr->ops[0].size = 16;
        instr->ops[0].reg.rt = LL_RT_XMM;
        // opOverwriteType(&instr->ops[0], VT_128);

        LLVMValueRef operand1 = ll_operand_load(OP_VF64, ALIGN_MAXIMUM, &instr->ops[0], state);
        LLVMValueRef operand2 = ll_operand_load(OP_SF64, ALIGN_MAXIMUM, &instr->ops[1], state);
        LLVMValueRef result = LLVMBuildInsertElement(state->builder, operand1, operand2, LLVMConstInt(i32, 1, false), "");
        ll_operand_store(OP_VF64, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, result, state);
    }
    else
    {
        // XXX: Hack for XED. Even though only 64 bits are written, they are in
        // the upper half of the register.
        instr->ops[0].size = 16;
        // opOverwriteType(&instr->ops[1], VT_128);

        LLVMValueRef operand1 = ll_operand_load(OP_VF64, ALIGN_MAXIMUM, &instr->ops[1], state);
        LLVMValueRef result = LLVMBuildExtractElement(state->builder, operand1, LLVMConstInt(i32, 1, false), "");
        ll_operand_store(OP_SF64, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, result, state);
    }
}

void
ll_instruction_sse_binary(LLInstr* instr, LLState* state, LLVMOpcode opcode,
                          bool fast_math, OperandDataType data_type)
{
    LLVMValueRef operand1 = ll_operand_load(data_type, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef operand2 = ll_operand_load(data_type, ALIGN_MAXIMUM, &instr->ops[1], state);
    LLVMValueRef result = LLVMBuildBinOp(state->builder, opcode, operand1, operand2, "");
    if (fast_math && state->cfg.enableFastMath)
    {
#if LL_LLVM_MAJOR >= 6
        llvm::unwrap<llvm::Instruction>(result)->setFast(true);
#else
        llvm::unwrap<llvm::Instruction>(result)->setHasUnsafeAlgebra(true);
#endif
    }
    ll_operand_store(data_type, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, result, state);
}

void
ll_instruction_unpck(LLInstr* instr, LLState* state, OperandDataType op_type)
{
    // This is actually legal as an implementation "MAY only fetch 64-bit".
    // See Intel SDM Vol. 2B 4-696 (Dec. 2016).
    instr->ops[1].size = 16;
    if (instr->ops[1].type == LL_OP_REG)
        instr->ops[1].reg.rt = LL_RT_XMM;

    llvm::Value* operand1 = llvm::unwrap(ll_operand_load(op_type, ALIGN_MAXIMUM, &instr->ops[0], state));
    llvm::Value* operand2 = llvm::unwrap(ll_operand_load(op_type, ALIGN_MAXIMUM, &instr->ops[1], state));

    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    llvm::Value* result = NULL;
    if (instr->type == LL_INS_UNPCKLPS)
        result = builder->CreateShuffleVector(operand1, operand2, {0, 4, 1, 5});
    else if (instr->type == LL_INS_UNPCKLPD)
        result = builder->CreateShuffleVector(operand1, operand2, {0, 2});
    else if (instr->type == LL_INS_UNPCKHPS)
        result = builder->CreateShuffleVector(operand1, operand2, {2, 6, 3, 7});
    else if (instr->type == LL_INS_UNPCKHPD)
        result = builder->CreateShuffleVector(operand1, operand2, {1, 3});

    ll_operand_store(op_type, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, llvm::wrap(result), state);
}

void
ll_instruction_shufps(LLInstr* instr, LLState* state)
{
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    uint32_t mask[4];
    for (int i = 0; i < 4; i++)
        mask[i] = (i < 2 ? 0 : 4) + ((instr->ops[2].val >> 2*i) & 3);

    LLVMValueRef operand1 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef operand2 = ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->ops[1], state);
    llvm::Value* result = builder->CreateShuffleVector(llvm::unwrap(operand1), llvm::unwrap(operand2), mask);
    ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, llvm::wrap(result), state);
}

void
ll_instruction_insertps(LLInstr* instr, LLState* state)
{
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    llvm::Value* src;

    int count_s = (instr->ops[2].val >> 6) & 3;
    int count_d = (instr->ops[2].val >> 4) & 3;
    int zmask = instr->ops[2].val & 0xf;

    // If src is a reg, extract element, otherwise load scalar from memory.
    if (instr->ops[1].type == LL_OP_REG)
    {
        src = llvm::unwrap(ll_operand_load(OP_V4F32, ALIGN_MAXIMUM, &instr->ops[1], state));
        src = builder->CreateExtractElement(src, count_s);
    }
    else
    {
        src = llvm::unwrap(ll_operand_load(OP_SF32, ALIGN_MAXIMUM, &instr->ops[1], state));
    }

    llvm::Value* dst = llvm::unwrap(ll_operand_load(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], state));
    dst = builder->CreateInsertElement(dst, src, count_d);

    if (zmask)
    {
        uint32_t mask[4];
        for (int i = 0; i < 4; i++)
            mask[i] = zmask & (1 << i) ? 4 : i;
        llvm::Value* zero = llvm::Constant::getNullValue(dst->getType());
        dst = builder->CreateShuffleVector(dst, zero, mask);
    }

    ll_operand_store(OP_VF32, ALIGN_MAXIMUM, &instr->ops[0], REG_KEEP_UPPER, llvm::wrap(dst), state);
}

/**
 * @}
 **/
