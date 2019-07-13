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

#include "llstate-internal.h"

#include "llcommon-internal.h"
#include "rellume/instr.h"
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>


/**
 * \defgroup LLInstructionGP General Purpose Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

void LLState::LiftMovgp(const LLInstr& inst, llvm::Instruction::CastOps cast) {
    // If the instruction moves the whole register, keep all facets.
    // TODO: implement this for all register-register moves.
    if (inst.ops[0].type == LL_OP_REG && inst.ops[0].size == 8 &&
            inst.ops[1].type == LL_OP_REG && inst.ops[1].size == 8) {
        regfile.Rename(inst.ops[0].reg, inst.ops[1].reg);
        return;
    }

    llvm::Value* val = OpLoad(inst.ops[1], Facet::I);
    llvm::Type* tgt_ty = irb.getIntNTy(inst.ops[0].size*8);
    OpStoreGp(inst.ops[0], irb.CreateCast(cast, val, tgt_ty));
}

void LLState::LiftAdd(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateAdd(op1, op2);

    // Compute pointer facet for 64-bit additions stored in a register.
    // TODO: handle case where the original pointer is the second operand.
    if (inst.ops[0].type == LL_OP_REG && inst.ops[0].size == 8) {
        llvm::Value* op1_ptr = GetReg(inst.ops[0].reg, Facet::PTR);
        SetReg(inst.ops[0].reg, Facet::I64, res);
        SetRegFacet(inst.ops[0].reg, Facet::PTR, irb.CreateGEP(op1_ptr, op2));
    } else {
        // We cannot use this outside of the if-clause, otherwise we would
        // clobber the pointer facet of the source operand.
        OpStoreGp(inst.ops[0], res);
    }

    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, op1, op2);
    FlagCalcCAdd(res, op1, op2);
    FlagCalcOAdd(res, op1, op2);
}

void LLState::LiftSub(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateSub(op1, op2);

    // Compute pointer facet for 64-bit additions stored in a register.
    // TODO: handle case where the original pointer is the second operand.
    if (inst.ops[0].type == LL_OP_REG && inst.ops[0].size == 8) {
        llvm::Value* op1_ptr = GetReg(inst.ops[0].reg, Facet::PTR);
        llvm::Value* res_ptr = irb.CreateGEP(op1_ptr, irb.CreateNeg(op2));
        SetReg(inst.ops[0].reg, Facet::I64, res);
        SetRegFacet(inst.ops[0].reg, Facet::PTR, res_ptr);
    } else {
        // We cannot use this outside of the if-clause, otherwise we would
        // clobber the pointer facet of the source operand.
        OpStoreGp(inst.ops[0], res);
    }

    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, op1, op2);
    FlagCalcCSub(res, op1, op2);
    FlagCalcOSub(res, op1, op2);
    regfile.GetFlagCache().update(op1, op2);
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

    regfile.GetFlagCache().update(op1, op2);
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

void LLState::LiftIncDec(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = irb.getIntN(inst.ops[0].size*8, 1);
    llvm::Value* res = nullptr;
    if (inst.type == LL_INS_INC) {
        res = irb.CreateAdd(op1, op2);
        FlagCalcOAdd(res, op1, op2);
    } else if (inst.type == LL_INS_DEC) {
        res = irb.CreateSub(op1, op2);
        FlagCalcOSub(res, op1, op2);
    }

    // Carry flag is _not_ updated.
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, op1, op2);
    OpStoreGp(inst.ops[0], res);
}

void LLState::LiftShift(const LLInstr& inst, llvm::Instruction::BinaryOps op) {
    llvm::Value* src = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* shift;
    if (inst.operand_count == 1) {
        shift = llvm::ConstantInt::get(src->getType(), 1);
    } else {
        unsigned mask = inst.ops[0].size == 8 ? 0x3f : 0x1f;
        shift = irb.CreateZExt(OpLoad(inst.ops[1], Facet::I), src->getType());
        // TODO: support small shifts with amount > len
        // LLVM sets the result to poison if this occurs.
        shift = irb.CreateAnd(shift, mask);
    }

    llvm::Value* res = irb.CreateBinOp(op, src, shift);
    OpStoreGp(inst.ops[0], res);

    llvm::Value* undef = llvm::UndefValue::get(irb.getInt1Ty());
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    // TODO: calculate flags correctly
    SetFlag(RFLAG_AF, undef);
    SetFlag(RFLAG_OF, undef);
    SetFlag(RFLAG_CF, undef);
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

void LLState::LiftCmovcc(const LLInstr& inst, Condition cond) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    OpStoreGp(inst.ops[0], irb.CreateSelect(FlagCond(cond), op2, op1));
}

void LLState::LiftSetcc(const LLInstr& inst, Condition cond) {
    OpStoreGp(inst.ops[0], irb.CreateZExt(FlagCond(cond), irb.getInt8Ty()));
}

void LLState::LiftCdqe(const LLInstr& inst) {
    LLInstrOp src_op = LLInstrOp::Reg(LLReg::Gp(inst.operand_size/2, LL_RI_A));
    LLInstrOp dst_op = LLInstrOp::Reg(LLReg::Gp(inst.operand_size, LL_RI_A));
    llvm::Type* dst_ty = irb.getIntNTy(inst.operand_size * 8);

    OpStoreGp(dst_op, irb.CreateSExt(OpLoad(src_op, Facet::I), dst_ty));
}

/**
 * @}
 **/
