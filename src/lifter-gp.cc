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

#include "lifter.h"

#include "facet.h"
#include "regfile.h"
#include "rellume/instr.h"
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>


/**
 * \defgroup LLInstructionGP General Purpose Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

namespace rellume {

void Lifter::Lift(const LLInstr& inst) {
    // Set new instruction pointer register
    llvm::Value* ripValue = irb.getInt64(inst.addr + inst.len);
    SetReg(X86Reg::IP, Facet::I64, ripValue);

    // Add separator for debugging.
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    irb.CreateCall(llvm::Intrinsic::getDeclaration(module,
                                                   llvm::Intrinsic::donothing,
                                                   {}));

    // Check overridden implementations first.
    const auto& override = cfg.instr_overrides.find(inst.type);
    if (override != cfg.instr_overrides.end()) {
        LiftOverride(inst, override->second);
        return;
    }

    switch (inst.type)
    {
#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include "rellume/opcodes.inc"
#undef DEF_IT

        default:
    not_implemented:
            fprintf(stderr, "Could not handle instruction at %#zx\n", inst.addr);
            assert(0);
            break;
    }
}

void Lifter::LiftOverride(const LLInstr& inst, llvm::Function* override) {
    if (inst.type == LL_INS_SYSCALL) {
        SetReg(X86Reg::GP(LL_RI_C), Facet::I64,
               GetReg(X86Reg::IP, Facet::I64));
        SetReg(X86Reg::GP(11), Facet::I64, FlagAsReg(64));
    }

    auto call_type = llvm::FunctionType::get(irb.getVoidTy(), {fi.sptr_raw->getType()}, false);

    // Pack all state into the CPU struct.
    CallConv sptr_conv = CallConv::SPTR;
    sptr_conv.Pack(*regfile, fi);
    llvm::CallInst* call = irb.CreateCall(call_type, override, {fi.sptr_raw});
    regfile->Clear(); // Clear all facets before importing register state
    sptr_conv.Unpack(*regfile, fi);

    // Directly inline alwaysinline functions
    if (override->hasFnAttribute(llvm::Attribute::AlwaysInline)) {
        llvm::InlineFunctionInfo ifi;
        llvm::InlineFunction(llvm::CallSite(call), ifi);
    }
}

void Lifter::LiftMovgp(const LLInstr& inst, llvm::Instruction::CastOps cast) {
    // TODO: if the instruction moves the whole register, keep all facets.
    // TODO: implement this for all register-register moves.

    llvm::Value* val = OpLoad(inst.ops[1], Facet::I);
    llvm::Type* tgt_ty = irb.getIntNTy(inst.ops[0].size*8);
    OpStoreGp(inst.ops[0], irb.CreateCast(cast, val, tgt_ty));
}

void Lifter::LiftAdd(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateAdd(op1, op2);

    // Compute pointer facet for 64-bit add with immediate. We can't usefully do
    // this for reg-reg, because we can't identify the base operand.
    if (cfg.use_gep_ptr_arithmetic && inst.ops[0].type == LL_OP_REG &&
            inst.ops[0].size == 8 && inst.ops[1].type == LL_OP_IMM) {
        llvm::Value* op1_ptr = GetReg(MapReg(inst.ops[0].reg), Facet::PTR);
        op1_ptr = irb.CreatePointerCast(op1_ptr, irb.getInt8PtrTy());
        SetReg(MapReg(inst.ops[0].reg), Facet::I64, res);
        SetRegFacet(MapReg(inst.ops[0].reg), Facet::PTR, irb.CreateGEP(op1_ptr, op2));
    } else {
        // We cannot use this outside of the if-clause, otherwise we would
        // clobber the pointer facet of the source operand.
        OpStoreGp(inst.ops[0], res);
    }

    FlagCalcAdd(res, op1, op2);
}

void Lifter::LiftAdc(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    op2 = irb.CreateAdd(op2, irb.CreateZExt(GetFlag(Facet::CF), op2->getType()));
    llvm::Value* res = irb.CreateAdd(op1, op2);

    OpStoreGp(inst.ops[0], res);
    FlagCalcAdd(res, op1, op2);
}

void Lifter::LiftXadd(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateAdd(op1, op2);

    // TODO: generate pointer facets?
    OpStoreGp(inst.ops[0], res);
    OpStoreGp(inst.ops[1], op1);
    FlagCalcAdd(res, op1, op2);
}

void Lifter::LiftSub(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateSub(op1, op2);

    // Compute pointer facet for 64-bit additions stored in a register.
    // TODO: handle case where the original pointer is the second operand.
    if (cfg.use_gep_ptr_arithmetic && inst.ops[0].type == LL_OP_REG &&
            inst.ops[0].size == 8) {
        llvm::Value* op1_ptr = GetReg(MapReg(inst.ops[0].reg), Facet::PTR);
        op1_ptr = irb.CreatePointerCast(op1_ptr, irb.getInt8PtrTy());
        llvm::Value* res_ptr = irb.CreateGEP(op1_ptr, irb.CreateNeg(op2));
        SetReg(MapReg(inst.ops[0].reg), Facet::I64, res);
        SetRegFacet(MapReg(inst.ops[0].reg), Facet::PTR, res_ptr);
    } else {
        // We cannot use this outside of the if-clause, otherwise we would
        // clobber the pointer facet of the source operand.
        OpStoreGp(inst.ops[0], res);
    }

    FlagCalcSub(res, op1, op2);
}

void Lifter::LiftSbb(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    op2 = irb.CreateAdd(op2, irb.CreateZExt(GetFlag(Facet::CF), op2->getType()));
    llvm::Value* res = irb.CreateSub(op1, op2);

    OpStoreGp(inst.ops[0], res);
    FlagCalcSub(res, op1, op2);
}

void Lifter::LiftCmp(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Value* res = irb.CreateSub(op1, op2);
    FlagCalcSub(res, op1, op2);

    if (cfg.prefer_pointer_cmp && op1->getType()->getIntegerBitWidth() == 64 &&
        inst.ops[0].type == LL_OP_REG && inst.ops[1].type == LL_OP_REG)
    {
        llvm::Value* ptr1 = GetReg(MapReg(inst.ops[0].reg), Facet::PTR);
        llvm::Value* ptr2 = GetReg(MapReg(inst.ops[1].reg), Facet::PTR);
        SetFlag(Facet::ZF, irb.CreateICmpEQ(ptr1, ptr2));
    }
}

void Lifter::LiftCmpxchg(const LLInstr& inst) {
    auto acc = OpLoad(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_A)), Facet::I);
    auto dst = OpLoad(inst.ops[0], Facet::I);
    auto src = OpLoad(inst.ops[1], Facet::I);

    // Full compare with acc and dst
    llvm::Value* cmp_res = irb.CreateSub(acc, dst);
    FlagCalcSub(cmp_res, acc, dst);

    // Store SRC if DST=ACC, else store DST again (i.e. don't change memory).
    OpStoreGp(inst.ops[0], irb.CreateSelect(GetFlag(Facet::ZF), src, dst));
    // ACC gets the value from memory.
    OpStoreGp(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_A)), dst);
}

void Lifter::LiftXchg(const LLInstr& inst) {
    // TODO: atomic memory operation
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    OpStoreGp(inst.ops[0], op2);
    OpStoreGp(inst.ops[1], op1);
}

void Lifter::LiftAndOrXor(const LLInstr& inst, llvm::Instruction::BinaryOps op,
                           bool writeback) {
    llvm::Value* res = irb.CreateBinOp(op, OpLoad(inst.ops[0], Facet::I),
                                       OpLoad(inst.ops[1], Facet::I));
    if (writeback)
        OpStoreGp(inst.ops[0], res);

    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlagUndef({Facet::AF});
    SetFlag(Facet::CF, irb.getFalse());
    SetFlag(Facet::OF, irb.getFalse());
}

void Lifter::LiftNot(const LLInstr& inst) {
    OpStoreGp(inst.ops[0], irb.CreateNot(OpLoad(inst.ops[0], Facet::I)));
}

void Lifter::LiftNeg(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* res = irb.CreateNeg(op1);
    llvm::Value* zero = llvm::Constant::getNullValue(res->getType());
    FlagCalcSub(res, zero, op1);
    OpStoreGp(inst.ops[0], res);
}

void Lifter::LiftIncDec(const LLInstr& inst) {
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

void Lifter::LiftShift(const LLInstr& inst, llvm::Instruction::BinaryOps op) {
    llvm::Value* src = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* src_ex;
    if (inst.ops[0].size >= 4)
        src_ex = src;
    else if (inst.type == LL_INS_SAR)
        src_ex = irb.CreateSExt(src, irb.getInt32Ty());
    else // inst.ops[0].size < 4 && (SHR || SHL)
        src_ex = irb.CreateZExt(src, irb.getInt32Ty());

    llvm::Value* shift;
    if (inst.operand_count == 1)
        shift = irb.getInt8(1);
    else
        shift = OpLoad(inst.ops[1], Facet::I);

    unsigned mask = inst.ops[0].size == 8 ? 0x3f : 0x1f;
    shift = irb.CreateAnd(irb.CreateZExt(shift, src_ex->getType()), mask);

    llvm::Value* res_ex = irb.CreateBinOp(op, src_ex, shift);
    llvm::Value* res = irb.CreateTrunc(res_ex, src->getType());
    OpStoreGp(inst.ops[0], res);

    // CF is the last bit shifted out
    llvm::Value* cf_big;
    if (inst.type == LL_INS_SHL) {
        unsigned sz = inst.ops[0].size * 8;
        llvm::Value* max = llvm::ConstantInt::get(src_ex->getType(), sz);
        cf_big = irb.CreateLShr(src_ex, irb.CreateSub(max, shift));
    } else { // SHR/SAR
        llvm::Value* one = llvm::ConstantInt::get(src_ex->getType(), 1);
        cf_big = irb.CreateLShr(src_ex, irb.CreateSub(shift, one));
    }

    // TODO: flags are only affected if shift != 0
    FlagCalcS(res);
    FlagCalcZ(res);
    FlagCalcP(res);
    SetFlag(Facet::CF, irb.CreateTrunc(cf_big, irb.getInt1Ty()));
    llvm::Value* zero = llvm::ConstantInt::get(src->getType(), 0);
    SetFlag(Facet::OF, irb.CreateICmpSLT(irb.CreateXor(src, res), zero));
    SetFlagUndef({Facet::AF});
}

void Lifter::LiftShiftdouble(const LLInstr& inst) {
    llvm::Value* src1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* src2 = OpLoad(inst.ops[1], Facet::I);
    llvm::Type* ty = src1->getType();
    llvm::Value* shift;
    llvm::Value* res;
    if (inst.operand_count == 2) {
        shift = llvm::ConstantInt::get(ty, 1);
    } else {
        unsigned mask = inst.ops[0].size == 8 ? 0x3f : 0x1f;
        shift = irb.CreateZExt(OpLoad(inst.ops[2], Facet::I), ty);
        // TODO: support small shifts with amount > len
        // LLVM sets the result to poison if this occurs.
        shift = irb.CreateAnd(shift, mask);
    }

    auto id = inst.type == LL_INS_SHLD ? llvm::Intrinsic::fshl
                                       : llvm::Intrinsic::fshr;
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    if (inst.type == LL_INS_SHLD)
        res = irb.CreateCall(intrinsic, {src1, src2, shift});
    else if (inst.type == LL_INS_SHRD)
        res = irb.CreateCall(intrinsic, {src2, src1, shift});
    else
        assert(false && "invalid double-shift operation");
    OpStoreGp(inst.ops[0], res);

    // TODO: calculate flags correctly
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlagUndef({Facet::OF, Facet::AF, Facet::CF});
}

void Lifter::LiftRotate(const LLInstr& inst) {
    llvm::Value* src = OpLoad(inst.ops[0], Facet::I);
    llvm::Type* ty = src->getType();
    llvm::Value* shift;
    if (inst.operand_count == 1) {
        shift = llvm::ConstantInt::get(ty, 1);
    } else {
        unsigned mask = inst.ops[0].size == 8 ? 0x3f : 0x1f;
        shift = irb.CreateZExt(OpLoad(inst.ops[1], Facet::I), ty);
        // TODO: support small shifts with amount > len
        // LLVM sets the result to poison if this occurs.
        shift = irb.CreateAnd(shift, mask);
    }

    auto id = inst.type == LL_INS_ROL ? llvm::Intrinsic::fshl
                                      : llvm::Intrinsic::fshr;
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    llvm::Value* res = irb.CreateCall(intrinsic, {src, src, shift});
    OpStoreGp(inst.ops[0], res);

    // SF, ZF, AF, and PF are unaffected.
    // TODO: calculate flags correctly
    // CF is affected only if count > 0
    // OF is affected only if count > 0, but undefined if count > 1
    SetFlagUndef({Facet::OF, Facet::CF});
}

void Lifter::LiftMul(const LLInstr& inst) {
    llvm::Value* op1;
    llvm::Value* op2;
    if (inst.operand_count == 1) {
        op1 = OpLoad(inst.ops[0], Facet::I);
        op2 = OpLoad(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_A)), Facet::I);
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
            OpStoreGp(LLInstrOp({LL_RT_GP16, LL_RI_A}), ext_res);
        } else {
            // Don't use short_res to avoid having two multiplications.
            // TODO: is this concern still valid?
            llvm::Type* value_ty = irb.getIntNTy(inst.ops[0].size*8);
            llvm::Value* res_a = irb.CreateTrunc(ext_res, value_ty);
            llvm::Value* high = irb.CreateLShr(ext_res, inst.ops[0].size*8);
            llvm::Value* res_d = irb.CreateTrunc(high, value_ty);
            OpStoreGp(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_A)), res_a);
            OpStoreGp(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_D)), res_d);
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

    SetFlag(Facet::OF, overflow);
    SetFlag(Facet::CF, overflow);
    SetFlagUndef({Facet::SF, Facet::ZF, Facet::AF, Facet::PF});
}

void Lifter::LiftDiv(const LLInstr& inst) {
    bool sign = inst.type == LL_INS_IDIV;
    auto ext_op = sign ? llvm::Instruction::SExt : llvm::Instruction::ZExt;
    auto div_op = sign ? llvm::Instruction::SDiv : llvm::Instruction::UDiv;
    auto rem_op = sign ? llvm::Instruction::SRem : llvm::Instruction::URem;

    // TODO: raise #DE on division by zero or overflow.

    auto ex_ty = irb.getIntNTy(inst.ops[0].size*8 * 2);

    llvm::Value* dividend;
    if (inst.ops[0].size == 1) {
        // Dividend is AX
        dividend = OpLoad(LLInstrOp({LL_RT_GP16, LL_RI_A}), Facet::I);
    } else {
        // Dividend is DX:AX/EDX:EAX/RDX:RAX
        auto low = OpLoad(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_A)), Facet::I);
        auto high = OpLoad(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_D)), Facet::I);
        high = irb.CreateShl(irb.CreateZExt(high, ex_ty), inst.ops[0].size*8);
        dividend = irb.CreateOr(irb.CreateZExt(low, ex_ty), high);
    }

    // Divisor is the operand
    auto divisor = irb.CreateCast(ext_op, OpLoad(inst.ops[0], Facet::I), ex_ty);

    auto quot = irb.CreateBinOp(div_op, dividend, divisor);
    auto rem = irb.CreateBinOp(rem_op, dividend, divisor);

    auto val_ty = irb.getIntNTy(inst.ops[0].size*8);
    quot = irb.CreateTrunc(quot, val_ty);
    rem = irb.CreateTrunc(rem, val_ty);

    if (inst.ops[0].size == 1) {
        // Quotient is AL, remainder is AH
        OpStoreGp(LLInstrOp({LL_RT_GP8Leg, LL_RI_A}), quot);
        OpStoreGp(LLInstrOp({LL_RT_GP8Leg, LL_RI_AH}), rem);
    } else {
        // Quotient is AX/EAX/RAX, remainer is DX/EDX/RDX
        OpStoreGp(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_A)), quot);
        OpStoreGp(LLInstrOp(LLReg::Gp(inst.ops[0].size, LL_RI_D)), rem);
    }

    SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                  Facet::CF});
}

void
Lifter::LiftLea(const LLInstr& inst)
{
    assert(inst.ops[0].type == LL_OP_REG);
    assert(inst.ops[1].type == LL_OP_MEM);

    // Compute pointer before we overwrite any registers.
    llvm::Value* res_ptr = OpAddr(inst.ops[1], irb.getInt8Ty());

    // Compute as integer
    unsigned addrsz = inst.ops[1].addrsize * 8;
    Facet facet = Facet{Facet::I}.Resolve(addrsz);
    llvm::Value* res = irb.getIntN(addrsz, inst.ops[1].val);
    if (inst.ops[1].reg.rt != LL_RT_None)
        res = irb.CreateAdd(res, GetReg(MapReg(inst.ops[1].reg), facet));
    if (inst.ops[1].scale != 0) {
        llvm::Value* offset = GetReg(MapReg(inst.ops[1].ireg), facet);
        offset = irb.CreateMul(offset, irb.getIntN(addrsz, inst.ops[1].scale));
        res = irb.CreateAdd(res, offset);
    }

    llvm::Type* op_type = irb.getIntNTy(inst.ops[0].size * 8);
    OpStoreGp(inst.ops[0], irb.CreateZExtOrTrunc(res, op_type));

    if (cfg.use_gep_ptr_arithmetic && inst.ops[0].reg.rt == LL_RT_GP64)
        SetRegFacet(MapReg(inst.ops[0].reg), Facet::PTR, res_ptr);
}

void Lifter::LiftXlat(const LLInstr& inst) {
    llvm::Value* al = OpLoad(LLInstrOp({LL_RT_GP8, LL_RI_A}), Facet::I8);
    llvm::Value* bx;
    if (inst.address_size == 8) {
        bx = OpLoad(LLInstrOp({LL_RT_GP64, LL_RI_B}), Facet::PTR);
        bx = irb.CreatePointerCast(bx, irb.getInt8PtrTy());
    } else {
        bx = OpLoad(LLInstrOp({LL_RT_GP32, LL_RI_B}), Facet::I32);
        bx = irb.CreateIntToPtr(bx, irb.getInt8PtrTy());
    }
    assert(bx->getType() == irb.getInt8PtrTy() && "xlat wrong rbx type");

    llvm::Value* ptr = irb.CreateGEP(bx, irb.CreateZExt(al, irb.getInt32Ty()));
    OpStoreGp(LLInstrOp({LL_RT_GP8, LL_RI_A}), irb.CreateLoad(ptr));
}

void Lifter::LiftCmovcc(const LLInstr& inst, Condition cond) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::I);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::I);
    OpStoreGp(inst.ops[0], irb.CreateSelect(FlagCond(cond), op2, op1));
}

void Lifter::LiftSetcc(const LLInstr& inst, Condition cond) {
    OpStoreGp(inst.ops[0], irb.CreateZExt(FlagCond(cond), irb.getInt8Ty()));
}

void Lifter::LiftCext(const LLInstr& inst) {
    LLInstrOp src_op = LLInstrOp(LLReg::Gp(inst.operand_size/2, LL_RI_A));
    LLInstrOp dst_op = LLInstrOp(LLReg::Gp(inst.operand_size, LL_RI_A));
    llvm::Type* dst_ty = irb.getIntNTy(inst.operand_size * 8);

    OpStoreGp(dst_op, irb.CreateSExt(OpLoad(src_op, Facet::I), dst_ty));
}

void Lifter::LiftCsep(const LLInstr& inst) {
    LLInstrOp src_op = LLInstrOp(LLReg::Gp(inst.operand_size, LL_RI_A));
    LLInstrOp dst_op = LLInstrOp(LLReg::Gp(inst.operand_size, LL_RI_D));

    llvm::Value* src = OpLoad(src_op, Facet::I);
    OpStoreGp(dst_op, irb.CreateAShr(src, inst.operand_size * 8 - 1));
}

void Lifter::LiftBitscan(const LLInstr& inst, bool trailing) {
    llvm::Value* src = OpLoad(inst.ops[1], Facet::I);
    auto id = trailing ? llvm::Intrinsic::cttz : llvm::Intrinsic::ctlz;
    llvm::Value* res = irb.CreateBinaryIntrinsic(id, src,
                                                 /*zero_undef=*/irb.getTrue());
    if (!trailing) {
        unsigned sz = inst.ops[1].size*8;
        res = irb.CreateSub(irb.getIntN(sz, sz - 1), res);
    }
    OpStoreGp(inst.ops[0], res);

    FlagCalcZ(src);
    SetFlagUndef({Facet::OF, Facet::SF, Facet::AF, Facet::PF, Facet::CF});
}

void Lifter::LiftBittest(const LLInstr& inst) {
    llvm::Value* index = OpLoad(inst.ops[1], Facet::I);
    unsigned op_size = inst.ops[0].size * 8;
    assert((op_size == 16 || op_size == 32 || op_size == 64) &&
            "invalid bittest operation size");

    llvm::Value* val;
    llvm::Value* addr;
    if (inst.ops[0].type == LL_OP_REG) {
        val = OpLoad(inst.ops[0], Facet::I);
    } else { // LL_OP_MEM
        addr = OpAddr(inst.ops[0], irb.getIntNTy(op_size));
        // Immediate operands are truncated, register operands are sign-extended
        if (inst.ops[1].type == LL_OP_REG) {
            llvm::Value* off = irb.CreateAShr(index, __builtin_ctz(op_size));
            addr = irb.CreateGEP(addr, irb.CreateSExt(off, irb.getInt64Ty()));
        }
        val = irb.CreateLoad(addr);
    }

    // Truncated here because memory operand may need full value.
    index = irb.CreateAnd(index, irb.getIntN(op_size, op_size-1));
    llvm::Value* mask = irb.CreateShl(irb.getIntN(op_size, 1), index);

    llvm::Value* bit = irb.CreateAnd(val, mask);

    if (inst.type == LL_INS_BT) {
        goto skip_writeback;
    } else if (inst.type == LL_INS_BTC) {
        val = irb.CreateXor(val, mask);
    } else if (inst.type == LL_INS_BTR) {
        val = irb.CreateAnd(val, irb.CreateNot(mask));
    } else if (inst.type == LL_INS_BTS) {
        val = irb.CreateOr(val, mask);
    }

    if (inst.ops[0].type == LL_OP_REG)
        OpStoreGp(inst.ops[0], val);
    else // LL_OP_MEM
        irb.CreateStore(val, addr);

skip_writeback:
    // Zero flag is not modified
    SetFlag(Facet::CF, irb.CreateICmpNE(bit, irb.getIntN(op_size, 0)));
    SetFlagUndef({Facet::OF, Facet::SF, Facet::AF, Facet::PF});
}

void Lifter::LiftMovbe(const LLInstr& inst) {
    llvm::Value* src = OpLoad(inst.ops[1], Facet::I);
    OpStoreGp(inst.ops[0], CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftBswap(const LLInstr& inst) {
    assert(inst.ops[0].type == LL_OP_REG && "bswap with non-reg operand");
    llvm::Value* src = OpLoad(inst.ops[0], Facet::I);
    OpStoreGp(inst.ops[0], CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftJmp(const LLInstr& inst) {
    LLInstrOp op = inst.ops[0];
    op.seg = LL_RI_None; // Force default segment, 3e is notrack.
    SetReg(X86Reg::IP, Facet::I64, OpLoad(op, Facet::I64));
}

void Lifter::LiftJcc(const LLInstr& inst, Condition cond) {
    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(FlagCond(cond),
        OpLoad(inst.ops[0], Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftJcxz(const LLInstr& inst) {
    unsigned sz = inst.address_size;
    llvm::Value* cx = OpLoad(LLInstrOp(LLReg::Gp(sz, LL_RI_C)), Facet::I);
    llvm::Value* cond = irb.CreateICmpEQ(cx, irb.getIntN(sz*8, 0));
    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(cond,
        OpLoad(inst.ops[0], Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftLoop(const LLInstr& inst) {
    unsigned sz = inst.address_size;
    LLInstrOp cx_op = LLInstrOp(LLReg::Gp(sz, LL_RI_C));

    // Decrement RCX/ECX
    auto cx = irb.CreateSub(OpLoad(cx_op, Facet::I), irb.getIntN(sz*8, 1));
    OpStoreGp(cx_op, cx);

    // Construct condition
    llvm::Value* cond = irb.CreateICmpNE(cx, irb.getIntN(sz*8, 0));
    if (inst.type == LL_INS_LOOPE)
        cond = irb.CreateAnd(cond, GetFlag(Facet::ZF));
    else if (inst.type == LL_INS_LOOPNE)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(Facet::ZF)));

    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(cond,
        OpLoad(inst.ops[0], Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftCall(const LLInstr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                      Facet::CF});

    LLInstrOp op = inst.ops[0];
    op.seg = LL_RI_None; // Force default segment, 3e is notrack.
    llvm::Value* new_rip = OpLoad(op, Facet::I);
    StackPush(GetReg(X86Reg::IP, Facet::I64));
    SetReg(X86Reg::IP, Facet::I64, new_rip);
}

void Lifter::LiftRet(const LLInstr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                      Facet::CF});

    SetReg(X86Reg::IP, Facet::I64, StackPop());
}

void Lifter::LiftUnreachable(const LLInstr& inst) {
    irb.CreateUnreachable();
    SetReg(X86Reg::IP, Facet::I64, llvm::UndefValue::get(irb.getInt64Ty()));
}

LifterBase::RepInfo LifterBase::RepBegin(const LLInstr& inst) {
    RepInfo info = {};
    bool di = false, si = false;

    switch (inst.type) {
    case LL_INS_LODS:       info.mode=RepInfo::NO_REP; di=false; si=true; break;
    case LL_INS_REP_LODS:   info.mode=RepInfo::REP;    di=false; si=true; break;
    case LL_INS_STOS:       info.mode=RepInfo::NO_REP; di=true; si=false; break;
    case LL_INS_REP_STOS:   info.mode=RepInfo::REP;    di=true; si=false; break;
    case LL_INS_MOVS:       info.mode=RepInfo::NO_REP; di=true; si=true;  break;
    case LL_INS_REP_MOVS:   info.mode=RepInfo::REP;    di=true; si=true;  break;
    case LL_INS_SCAS:       info.mode=RepInfo::NO_REP; di=true; si=false; break;
    case LL_INS_REPZ_SCAS:  info.mode=RepInfo::REPZ;   di=true; si=false; break;
    case LL_INS_REPNZ_SCAS: info.mode=RepInfo::REPNZ;  di=true; si=false; break;
    case LL_INS_CMPS:       info.mode=RepInfo::NO_REP; di=true; si=true;  break;
    case LL_INS_REPZ_CMPS:  info.mode=RepInfo::REPZ;   di=true; si=true;  break;
    case LL_INS_REPNZ_CMPS: info.mode=RepInfo::REPNZ;  di=true; si=true;  break;
    default: assert(false && "non-string instruction in RepBegin");
    }

    // Iff instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode != RepInfo::NO_REP) {
        info.loop_block = ablock.AddBlock();
        info.cont_block = ablock.AddBlock();

        llvm::Value* count = GetReg(X86Reg::GP(LL_RI_C), Facet::I64);
        llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
        llvm::Value* enter_loop = irb.CreateICmpNE(count, zero);
        ablock.GetInsertBlock()->BranchTo(enter_loop, *info.loop_block,
                                          *info.cont_block);

        SetInsertBlock(info.loop_block);
    }

    llvm::Type* op_ty = irb.getIntNTy(inst.operand_size * 8);
    if (di) {
        llvm::Value* ptr = GetReg(X86Reg::GP(LL_RI_DI), Facet::PTR);
        info.di = irb.CreatePointerCast(ptr, op_ty->getPointerTo());
    }
    if (si) {
        llvm::Value* ptr = GetReg(X86Reg::GP(LL_RI_SI), Facet::PTR);
        info.si = irb.CreatePointerCast(ptr, op_ty->getPointerTo());
    }

    return info;
}

void LifterBase::RepEnd(RepInfo info) {
    // First update pointer registers with direction flag
    llvm::Value* df = GetFlag(Facet::DF);
    llvm::Value* adj = irb.CreateSelect(df, irb.getInt64(-1), irb.getInt64(1));

    std::pair<int, llvm::Value*> ptr_regs[] = {
        {LL_RI_DI, info.di}, {LL_RI_SI, info.si}
    };

    for (auto reg : ptr_regs) {
        if (reg.second == nullptr)
            continue;

        llvm::Value* ptr = irb.CreateGEP(reg.second, adj);
        llvm::Value* int_val = irb.CreatePtrToInt(ptr, irb.getInt64Ty());
        SetReg(X86Reg::GP(reg.first), Facet::I64, int_val);
        SetRegFacet(X86Reg::GP(reg.first), Facet::PTR, ptr);
    }

    // If instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode == RepInfo::NO_REP)
        return;

    // Decrement count and check.
    llvm::Value* count = GetReg(X86Reg::GP(LL_RI_C), Facet::I64);
    count = irb.CreateSub(count, irb.getInt64(1));
    SetReg(X86Reg::GP(LL_RI_C), Facet::I64, count);

    llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
    llvm::Value* cond = irb.CreateICmpNE(count, zero);
    if (info.mode == RepInfo::REPZ)
        cond = irb.CreateAnd(cond, GetFlag(Facet::ZF));
    else if (info.mode == RepInfo::REPNZ)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(Facet::ZF)));

    ablock.GetInsertBlock()->BranchTo(cond, *info.loop_block, *info.cont_block);
    SetInsertBlock(info.cont_block);
}

void Lifter::LiftLods(const LLInstr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    unsigned size = inst.operand_size;
    OpStoreGp(LLInstrOp(LLReg::Gp(size, LL_RI_A)), irb.CreateLoad(rep_info.di));

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftStos(const LLInstr& inst) {
    // TODO: optimize REP STOSB and other sizes with constant zero to llvm
    // memset intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    auto ax = OpLoad(LLInstrOp(LLReg::Gp(inst.operand_size, LL_RI_A)), Facet::I);
    irb.CreateStore(ax, rep_info.di);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftMovs(const LLInstr& inst) {
    // TODO: optimize REP MOVSB to use llvm memcpy intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    irb.CreateStore(irb.CreateLoad(rep_info.si), rep_info.di);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftScas(const LLInstr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    auto src = OpLoad(LLInstrOp(LLReg::Gp(inst.operand_size, LL_RI_A)), Facet::I);
    llvm::Value* dst = irb.CreateLoad(rep_info.di);
    // Perform a normal CMP operation.
    FlagCalcSub(irb.CreateSub(src, dst), src, dst);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftCmps(const LLInstr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    llvm::Value* src = irb.CreateLoad(rep_info.si);
    llvm::Value* dst = irb.CreateLoad(rep_info.di);
    // Perform a normal CMP operation.
    FlagCalcSub(irb.CreateSub(src, dst), src, dst);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

} // namespace

/**
 * @}
 **/
