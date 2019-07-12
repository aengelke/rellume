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
#include <llstate-internal.h>

#include <llcommon-internal.h>
#include <rellume/instr.h>

/**
 * \defgroup LLInstructionGP General Purpose Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

void
ll_instruction_movgp(LLInstr* instr, LLState* state)
{
    if (instr->ops[0].type == LL_OP_REG && instr->ops[1].type == LL_OP_REG && instr->ops[0].size == 8 && instr->ops[1].size == 8)
        state->regfile->Rename(instr->ops[0].reg, instr->ops[1].reg);
    else
    {
        LLVMTypeRef targetType = llvm::wrap(state->irb.getIntNTy(instr->ops[0].size * 8));
        LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[1], state);

        if (instr->type == LL_INS_MOVZX)
            operand1 = LLVMBuildZExtOrBitCast(state->builder, operand1, targetType, "");
        else if (instr->type == LL_INS_MOVSX) // There was a case when MOV was sign-extending, too...
            operand1 = LLVMBuildSExtOrBitCast(state->builder, operand1, targetType, "");

        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, operand1, state);
    }
}

void
ll_instruction_add(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[1], state);
    operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

    LLVMValueRef result = LLVMBuildAdd(state->builder, operand1, operand2, "");

    if (LLVMGetIntTypeWidth(LLVMTypeOf(operand1)) == 64 && instr->ops[0].type == LL_OP_REG)
    {
        LLVMValueRef ptr = llvm::wrap(state->GetReg(instr->ops[0].reg, Facet::PTR));
        LLVMValueRef gep = LLVMBuildGEP(state->builder, ptr, &operand2, 1, "");

        state->SetReg(instr->ops[0].reg, Facet::I64, llvm::unwrap(result));
        state->SetRegFacet(instr->ops[0].reg, Facet::PTR, llvm::unwrap(gep));
    }
    else
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, result, state);

    state->FlagCalcZ(llvm::unwrap(result));
    state->FlagCalcS(llvm::unwrap(result));
    state->FlagCalcP(llvm::unwrap(result));
    state->FlagCalcA(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    state->FlagCalcCAdd(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    state->FlagCalcOAdd(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
}

void
ll_instruction_sub(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[1], state);
    operand2 = LLVMBuildSExtOrBitCast(state->builder, operand2, LLVMTypeOf(operand1), "");

    LLVMValueRef result = LLVMBuildSub(state->builder, operand1, operand2, "");

    if (LLVMGetIntTypeWidth(LLVMTypeOf(operand1)) == 64 && instr->ops[0].type == LL_OP_REG)
    {
        LLVMValueRef sub = LLVMBuildNeg(state->builder, operand2, "");
        LLVMValueRef ptr = llvm::wrap(state->GetReg(instr->ops[0].reg, Facet::PTR));
        LLVMValueRef gep = LLVMBuildGEP(state->builder, ptr, &sub, 1, "");

        state->SetReg(instr->ops[0].reg, Facet::I64, llvm::unwrap(result));
        state->SetRegFacet(instr->ops[0].reg, Facet::PTR, llvm::unwrap(gep));
    }
    else
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, result, state);

    state->FlagCalcZ(llvm::unwrap(result));
    state->FlagCalcS(llvm::unwrap(result));
    state->FlagCalcP(llvm::unwrap(result));
    state->FlagCalcA(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    state->FlagCalcCSub(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    state->FlagCalcOSub(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    state->regfile->GetFlagCache().update(llvm::unwrap(operand1), llvm::unwrap(operand2));
}

void LLState::LiftCmp(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateSub(op1, op2);
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, op1, op2);
    FlagCalcCSub(res, op1, op2);
    FlagCalcOSub(res, op1, op2);

    if (cfg.prefer_pointer_cmp && op1->getType()->getIntegerBitWidth() == 64 &&
        inst.ops[0].type == LL_OP_REG && inst.ops[1].type == LL_OP_REG)
    {
        llvm::Value* ptr1 = GetReg(inst.ops[0].reg, Facet::PTR);
        llvm::Value* ptr2 = GetReg(inst.ops[1].reg, Facet::PTR);
        SetFlag(RFLAG_ZF, irb.CreateICmpEQ(ptr1, ptr2));
    }

    regfile->GetFlagCache().update(op1, op2);
}

void LLState::LiftAndOrXor(const LLInstr& inst, llvm::Instruction::BinaryOps op,
                           bool writeback) {
    llvm::Value* res = irb.CreateBinOp(op, OpLoad(inst.ops[0], Facet::I),
                                       OpLoad(inst.ops[1], Facet::I));
    if (writeback)
        OpStoreGp(inst.ops[0], res);

    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlag(RFLAG_AF, llvm::UndefValue::get(irb.getInt1Ty()));
    SetFlag(RFLAG_CF, irb.getFalse());
    SetFlag(RFLAG_OF, irb.getFalse());
}

void LLState::LiftNot(const LLInstr& inst) {
    OpStoreGp(inst.ops[0], irb.CreateNot(OpLoad(inst.ops[0], Facet::I)));
}

void LLState::LiftNeg(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* res = irb.CreateNeg(op1);
    llvm::Value* zero = llvm::Constant::getNullValue(res->getType());
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, zero, op1);
    FlagCalcCSub(res, zero, op1);
    FlagCalcOSub(res, zero, op1);
    OpStoreGp(inst.ops[0], res);
}

void
ll_instruction_incdec(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef operand2 = LLVMConstInt(LLVMTypeOf(operand1), 1, false);
    LLVMValueRef result = NULL;

    if (instr->type == LL_INS_INC)
    {
        result = LLVMBuildAdd(state->builder, operand1, operand2, "");
        state->FlagCalcOAdd(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    }
    else // LL_INS_DEC
    {
        result = LLVMBuildSub(state->builder, operand1, operand2, "");
        state->FlagCalcOSub(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));
    }

    // Carry flag is _not_ updated.
    state->FlagCalcZ(llvm::unwrap(result));
    state->FlagCalcS(llvm::unwrap(result));
    state->FlagCalcP(llvm::unwrap(result));
    state->FlagCalcA(llvm::unwrap(result), llvm::unwrap(operand1), llvm::unwrap(operand2));

    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, result, state);
}

void
ll_instruction_shift(LLInstr* instr, LLState* state)
{
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef operand2;
    LLVMValueRef result = NULL;

    if (instr->operand_count == 1)
    {
        // DBrew decodes shifts with a implicit shift of 1 with only one operand
        operand2 = LLVMConstInt(LLVMTypeOf(operand1), 1, false);
    }
    else
    {
        operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[1], state);
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
    }
    else if (instr->type == LL_INS_SHR)
    {
        result = LLVMBuildLShr(state->builder, operand1, operand2, "");
    }
    else if (instr->type == LL_INS_SAR)
    {
        result = LLVMBuildAShr(state->builder, operand1, operand2, "");
    }

    llvm::Value* undef = llvm::UndefValue::get(state->irb.getInt1Ty());
    state->FlagCalcZ(llvm::unwrap(result));
    state->FlagCalcS(llvm::unwrap(result));
    state->FlagCalcP(llvm::unwrap(result));
    state->SetFlag(RFLAG_AF, undef);
    state->SetFlag(RFLAG_OF, undef);
    state->SetFlag(RFLAG_CF, undef);

    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, result, state);
}

void LLState::LiftMul(const LLInstr& inst) {
    llvm::Value* op1;
    llvm::Value* op2;
    if (inst.operand_count == 1) {
        op1 = OpLoad(inst.ops[0], Facet::I);
        op2 = OpLoad(LLInstrOp::Reg(LLReg::Gp(inst.ops[0].size, LL_RI_A)), Facet::I);
    } else {
        op1 = OpLoad(inst.ops[inst.operand_count - 2], Facet::I);
        op2 = OpLoad(inst.ops[inst.operand_count - 1], Facet::I);
    }

    // Perform "normal" multiplication
    llvm::Value* short_res = irb.CreateMul(op1, op2);

    // Extend operand values and perform extended multiplication
    auto cast_op = inst.type == LL_INS_IMUL ? llvm::Instruction::SExt
                                            : llvm::Instruction::ZExt;
    llvm::Type* double_ty = irb.getIntNTy(inst.ops[0].size*8 * 2);
    llvm::Value* ext_op1 = irb.CreateCast(cast_op, op1, double_ty);
    llvm::Value* ext_op2 = irb.CreateCast(cast_op, op2, double_ty);
    llvm::Value* ext_res = irb.CreateMul(ext_op1, ext_op2);

    if (inst.operand_count == 1) {
        if (inst.ops[0].size == 1) {
            OpStoreGp(LLInstrOp::Reg(LLReg(LL_RT_GP16, LL_RI_A)), ext_res);
        } else {
            // Don't use short_res to avoid having two multiplications.
            // TODO: is this concern still valid?
            llvm::Type* value_ty = irb.getIntNTy(inst.ops[0].size*8);
            llvm::Value* res_a = irb.CreateTrunc(ext_res, value_ty);
            llvm::Value* high = irb.CreateLShr(ext_res, inst.ops[0].size*8);
            llvm::Value* res_d = irb.CreateTrunc(high, value_ty);
            OpStoreGp(LLInstrOp::Reg(LLReg::Gp(inst.ops[0].size, LL_RI_A)), res_a);
            OpStoreGp(LLInstrOp::Reg(LLReg::Gp(inst.ops[0].size, LL_RI_D)), res_d);
        }
    } else {
        OpStoreGp(inst.ops[0], short_res);
    }

    llvm::Value* overflow;
    if (cfg.enableOverflowIntrinsics) {
        llvm::Intrinsic::ID id = llvm::Intrinsic::smul_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, op1, op2);
        overflow = irb.CreateExtractValue(packed, 1);
    } else {
        llvm::Value* ext_short_res = irb.CreateCast(cast_op, short_res, double_ty);
        overflow = irb.CreateICmpNE(ext_res, ext_short_res);
    }

    llvm::Value* undef = llvm::UndefValue::get(irb.getInt1Ty());
    SetFlag(RFLAG_ZF, undef);
    SetFlag(RFLAG_SF, undef);
    SetFlag(RFLAG_PF, undef);
    SetFlag(RFLAG_AF, undef);
    SetFlag(RFLAG_OF, overflow);
    SetFlag(RFLAG_CF, overflow);
}

void
LLState::LiftLea(const LLInstr& inst)
{
    assert(inst.ops[0].type == LL_OP_REG);
    assert(inst.ops[1].type == LL_OP_MEM);

    // Compute pointer before we overwrite any registers.
    llvm::Value* res_ptr = OpAddr(inst.ops[1], irb.getInt8Ty());

    // Compute as integer
    llvm::Value* res = irb.getInt64(inst.ops[1].val);
    if (inst.ops[1].reg.rt != LL_RT_None)
        res = irb.CreateAdd(res, GetReg(inst.ops[1].reg, Facet::I64));
    if (inst.ops[1].scale != 0)
    {
        llvm::Value* offset = GetReg(inst.ops[1].ireg, Facet::I64);
        offset = irb.CreateMul(offset, irb.getInt64(inst.ops[1].scale));
        res = irb.CreateAdd(res, offset);
    }

    OpStoreGp(inst.ops[0], irb.CreateTrunc(res, irb.getIntNTy(inst.ops[0].size*8)));
    if (inst.ops[0].reg.rt == LL_RT_GP64)
        SetRegFacet(inst.ops[0].reg, Facet::PTR, res_ptr);
}

void
ll_instruction_cmov(LLInstr* instr, LLState* state)
{
    LLVMValueRef cond = llvm::wrap(state->FlagCond(instr->type, LL_INS_CMOVO));
    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[1], state);
    LLVMValueRef operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], state);
    LLVMValueRef result = LLVMBuildSelect(state->builder, cond, operand1, operand2, "");
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, result, state);
}

void
ll_instruction_setcc(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i8 = llvm::wrap(state->irb.getInt8Ty());
    LLVMValueRef cond = llvm::wrap(state->FlagCond(instr->type, LL_INS_SETO));
    LLVMValueRef result = LLVMBuildZExtOrBitCast(state->builder, cond, i8, "");
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->ops[0], REG_DEFAULT, result, state);
}

void
ll_instruction_cdqe(LLInstr* instr, LLState* state)
{
    LLInstrOp srcOp = LLInstrOp::Reg(LLReg(LL_RT_GP32, LL_RI_A));
    LLInstrOp dstOp = LLInstrOp::Reg(LLReg(LL_RT_GP64, LL_RI_A));

    LLVMValueRef operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &srcOp, state);
    ll_operand_store(OP_SI, ALIGN_MAXIMUM, &dstOp, REG_DEFAULT, operand1, state);

    (void) instr;
}

/**
 * @}
 **/
