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

#include "x86-64/lifter-private.h"

#include "facet.h"
#include "instr.h"
#include "regfile.h"
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>


/**
 * \defgroup InstructionGP General Purpose Instructions
 * \ingroup Instruction
 *
 * @{
 **/

namespace rellume::x86_64 {

void Lifter::LiftMovgp(const Instr& inst) {
    // TODO: if the instruction moves the whole register, keep all facets.
    // TODO: implement this for all register-register moves.

    llvm::Value* val = OpLoad(inst.op(1), Facet::I);
    llvm::Type* tgt_ty = irb.getIntNTy(inst.op(0).bits());
    OpStoreGp(inst.op(0), irb.CreateSExt(val, tgt_ty));
}

void Lifter::LiftMovzx(const Instr& inst) {
    // TODO: keep smaller facets intact on reg-reg extensions?
    llvm::Value* val = OpLoad(inst.op(1), Facet::I);
    auto dstSize = inst.op(0).bits();
    if (dstSize >= 32)
        SetReg(MapReg(inst.op(0).reg()), val); // Insert into zero
    else
        OpStoreGp(inst.op(0), irb.CreateZExt(val, irb.getIntNTy(dstSize)));
}

// Implementation of ADD, ADC, SUB, SBB, CMP, and XADD
void Lifter::LiftArith(const Instr& inst, bool sub) {
    llvm::Value* op1;
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    if (inst.type() == FDI_ADC || inst.type() == FDI_SBB)
        op2 = irb.CreateAdd(op2, irb.CreateZExt(GetFlag(ArchReg::CF), op2->getType()));

    auto arith_op = sub ? llvm::Instruction::Sub : llvm::Instruction::Add;
    auto atomic_op = sub ? llvm::AtomicRMWInst::Sub : llvm::AtomicRMWInst::Add;
    if (!inst.has_lock()) {
        op1 = OpLoad(inst.op(0), Facet::I);
    } else {
        auto ordering = llvm::AtomicOrdering::SequentiallyConsistent;
        llvm::Value* addr = OpAddr(inst.op(0), op2->getType());
        // Add op2 to *addr and return previous value
        op1 = irb.CreateAtomicRMW(atomic_op, addr, op2, {}, ordering);
    }

    llvm::Value* res = irb.CreateBinOp(arith_op, op1, op2);

    if (inst.type() != FDI_CMP && !inst.has_lock()) // atomicrmw stored already
        OpStoreGp(inst.op(0), res);
    if (inst.type() == FDI_XADD)
        OpStoreGp(inst.op(1), op1);

    if (sub)
        FlagCalcSub(res, op1, op2, /*skip_carry=*/false, /*alt_zf=*/inst.type() == FDI_CMP);
    else
        FlagCalcAdd(res, op1, op2);
}

void Lifter::LiftCmpxchg(const Instr& inst) {
    llvm::Value* acc = GetReg(ArchReg::RAX, Facet::In(inst.op(0).bits()));
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    llvm::Value* dst;

    if (inst.has_lock()) {
        auto ord = llvm::AtomicOrdering::SequentiallyConsistent;
        llvm::Value* ptr = OpAddr(inst.op(0), src->getType());
        // Do an atomic cmpxchg, compare *ptr with acc and set to src if equal
        auto cmpxchg = irb.CreateAtomicCmpXchg(ptr, acc, src, {}, ord, ord);
        dst = irb.CreateExtractValue(cmpxchg, {0});
        FlagCalcSub(irb.CreateSub(acc, dst), acc, dst);
        SetReg(ArchReg::ZF, irb.CreateExtractValue(cmpxchg, {1}));
    } else {
        dst = OpLoad(inst.op(0), Facet::I);
        // Full compare with acc and dst
        llvm::Value* cmp_res = irb.CreateSub(acc, dst);
        FlagCalcSub(cmp_res, acc, dst);

        // Store SRC if DST=ACC, else store DST again (i.e. don't change memory).
        OpStoreGp(inst.op(0), irb.CreateSelect(GetFlag(ArchReg::ZF), src, dst));
    }

    // ACC gets the value from memory.
    StoreGp(ArchReg::RAX, dst);
}

void Lifter::LiftXchg(const Instr& inst) {
    llvm::Value* op1;
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    if (inst.op(0).is_mem()) { // always atomic
        auto ord = llvm::AtomicOrdering::SequentiallyConsistent;
        llvm::Value* addr = OpAddr(inst.op(0), op2->getType());
        op1 = irb.CreateAtomicRMW(llvm::AtomicRMWInst::Xchg, addr, op2, {}, ord);
    } else {
        op1 = OpLoad(inst.op(0), Facet::I);
        OpStoreGp(inst.op(0), op2);
    }
    OpStoreGp(inst.op(1), op1);
}

void Lifter::LiftAndOrXor(const Instr& inst, llvm::Instruction::BinaryOps op,
                          llvm::AtomicRMWInst::BinOp armw_op, bool writeback) {
    if (inst.type() == FDI_XOR && inst.op(0).is_reg() && inst.op(1).is_reg() &&
        inst.op(0).reg().ri == inst.op(1).reg().ri && inst.op(0).size() >= 4) {
        OpStoreGp(inst.op(0), irb.getIntN(8 * inst.op(0).size(), 0));
        FlagCalcSAPLogic(irb.getInt64(0));
        SetReg(ArchReg::ZF, irb.getTrue());
        SetReg(ArchReg::CF, irb.getFalse());
        SetReg(ArchReg::OF, irb.getFalse());
        return;
    }

    llvm::Value* op1;
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    if (!inst.has_lock()) {
        op1 = OpLoad(inst.op(0), Facet::I);
    } else {
        auto ord = llvm::AtomicOrdering::SequentiallyConsistent;
        llvm::Value* addr = OpAddr(inst.op(0), op2->getType());
        op1 = irb.CreateAtomicRMW(armw_op, addr, op2, {}, ord);
    }

    llvm::Value* res = irb.CreateBinOp(op, op1, op2);
    if (writeback && !inst.has_lock())
        OpStoreGp(inst.op(0), res);

    FlagCalcZ(res);
    FlagCalcSAPLogic(res);
    SetReg(ArchReg::CF, irb.getFalse());
    SetReg(ArchReg::OF, irb.getFalse());
}

void Lifter::LiftNot(const Instr& inst) {
    if (!inst.has_lock()) {
        OpStoreGp(inst.op(0), irb.CreateNot(OpLoad(inst.op(0), Facet::I)));
    } else {
        auto ord = llvm::AtomicOrdering::SequentiallyConsistent;
        llvm::Value* mask = irb.getIntN(inst.op(0).bits(), -1);
        llvm::Value* addr = OpAddr(inst.op(0), mask->getType());
        irb.CreateAtomicRMW(llvm::AtomicRMWInst::Xor, addr, mask, {}, ord);
    }
}

void Lifter::LiftNeg(const Instr& inst) {
    assert(!inst.has_lock() && "atomic NEG not implemented");
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* res = irb.CreateNeg(op1);
    llvm::Value* zero = llvm::Constant::getNullValue(res->getType());
    FlagCalcSub(res, zero, op1);
    OpStoreGp(inst.op(0), res);
}

void Lifter::LiftIncDec(const Instr& inst) {
    llvm::Value* op1;
    llvm::Value* op2 = irb.getIntN(inst.op(0).bits(), 1);
    llvm::Value* res = nullptr;

    bool sub = inst.type() == FDI_DEC;
    auto arith_op = sub ? llvm::Instruction::Sub : llvm::Instruction::Add;
    auto atomic_op = sub ? llvm::AtomicRMWInst::Sub : llvm::AtomicRMWInst::Add;
    if (!inst.has_lock()) {
        op1 = OpLoad(inst.op(0), Facet::I);
        res = irb.CreateBinOp(arith_op, op1, op2);
        OpStoreGp(inst.op(0), res);
    } else {
        auto ord = llvm::AtomicOrdering::SequentiallyConsistent;
        llvm::Value* addr = OpAddr(inst.op(0), op2->getType());
        op1 = irb.CreateAtomicRMW(atomic_op, addr, op2, {}, ord);
        res = irb.CreateBinOp(arith_op, op1, op2);
    }
    if (sub)
        FlagCalcSub(res, op1, op2, /*skip_carry=*/true);
    else
        FlagCalcAdd(res, op1, op2, /*skip_carry=*/true);
}

void Lifter::LiftShift(const Instr& inst, llvm::Instruction::BinaryOps op) {
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    llvm::Value* src_ex;
    if (inst.op(0).size() >= 4)
        src_ex = src;
    else if (inst.type() == FDI_SAR)
        src_ex = irb.CreateSExt(src, irb.getInt32Ty());
    else // inst.op(0).size() < 4 && (SHR || SHL)
        src_ex = irb.CreateZExt(src, irb.getInt32Ty());

    assert(inst.op(1) && "shift without second operand");
    llvm::Value* shift = OpLoad(inst.op(1), Facet::I);

    unsigned mask = inst.op(0).size() == 8 ? 0x3f : 0x1f;
    shift = irb.CreateAnd(irb.CreateZExt(shift, src_ex->getType()), mask);

    llvm::Value* res_ex = irb.CreateBinOp(op, src_ex, shift);
    llvm::Value* res = irb.CreateTrunc(res_ex, src->getType());
    OpStoreGp(inst.op(0), res);

    // CF is the last bit shifted out
    llvm::Value* cf_big;
    if (inst.type() == FDI_SHL) {
        unsigned sz = inst.op(0).bits();
        llvm::Value* max = llvm::ConstantInt::get(src_ex->getType(), sz);
        cf_big = irb.CreateLShr(src_ex, irb.CreateSub(max, shift));
    } else { // SHR/SAR
        llvm::Value* one = llvm::ConstantInt::get(src_ex->getType(), 1);
        cf_big = irb.CreateLShr(src_ex, irb.CreateSub(shift, one));
    }

    // TODO: flags are only affected if shift != 0
    FlagCalcZ(res);
    FlagCalcSAPLogic(res);
    SetReg(ArchReg::CF, irb.CreateTrunc(cf_big, irb.getInt1Ty()));
    llvm::Value* zero = llvm::ConstantInt::get(src->getType(), 0);
    SetReg(ArchReg::OF, irb.CreateICmpSLT(irb.CreateXor(src, res), zero));
}

void Lifter::LiftShiftdouble(const Instr& inst) {
    llvm::Value* src1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* src2 = OpLoad(inst.op(1), Facet::I);
    llvm::Type* ty = src1->getType();
    llvm::Value* res;

    assert(inst.op(2) && "shld/shrd without third operand");
    unsigned mask = inst.op(0).size() == 8 ? 0x3f : 0x1f;
    llvm::Value* shift = irb.CreateZExt(OpLoad(inst.op(2), Facet::I), ty);
    // TODO: support small shifts with amount > len
    // LLVM sets the result to poison if this occurs.
    shift = irb.CreateAnd(shift, mask);

    auto id = inst.type() == FDI_SHLD ? llvm::Intrinsic::fshl
                                       : llvm::Intrinsic::fshr;
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    if (inst.type() == FDI_SHLD)
        res = irb.CreateCall(intrinsic, {src1, src2, shift});
    else if (inst.type() == FDI_SHRD)
        res = irb.CreateCall(intrinsic, {src2, src1, shift});
    else
        assert(false && "invalid double-shift operation");
    OpStoreGp(inst.op(0), res);

    // TODO: calculate flags correctly
    FlagCalcZ(res);
    FlagCalcSAPLogic(res);
    SetFlagUndef({ArchReg::OF, ArchReg::CF});
}

void Lifter::LiftRotate(const Instr& inst) {
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    llvm::Type* ty = src->getType();

    assert(inst.op(1) && "rotate without second operand");

    unsigned mask = inst.op(0).size() == 8 ? 0x3f : 0x1f;
    llvm::Value* shift = irb.CreateZExt(OpLoad(inst.op(1), Facet::I), ty);
    // TODO: support small shifts with amount > len
    // LLVM sets the result to poison if this occurs.
    shift = irb.CreateAnd(shift, mask);

    auto id = inst.type() == FDI_ROL ? llvm::Intrinsic::fshl
                                      : llvm::Intrinsic::fshr;
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    llvm::Value* res = irb.CreateCall(intrinsic, {src, src, shift});
    OpStoreGp(inst.op(0), res);

    // SF, ZF, AF, and PF are unaffected.
    // TODO: calculate flags correctly
    // CF is affected only if count > 0
    // OF is affected only if count > 0, but undefined if count > 1
    SetFlagUndef({ArchReg::OF, ArchReg::CF});
}

void Lifter::LiftMul(const Instr& inst) {
    unsigned sz = inst.op(0).bits();
    bool sign = inst.type() == FDI_IMUL;

    unsigned op_cnt = inst.op(2) ? 3 : inst.op(1) ? 2 : 1;

    llvm::Value* op1;
    llvm::Value* op2;
    if (op_cnt == 1) {
        op1 = OpLoad(inst.op(0), Facet::I);
        op2 = GetReg(ArchReg::RAX, Facet::In(sz));
    } else {
        op1 = OpLoad(inst.op(op_cnt - 2), Facet::I);
        op2 = OpLoad(inst.op(op_cnt - 1), Facet::I);
    }

    // Perform "normal" multiplication
    llvm::Value* short_res = irb.CreateMul(op1, op2);

    // Extend operand values and perform extended multiplication
    auto cast_op = sign ? llvm::Instruction::SExt : llvm::Instruction::ZExt;
    llvm::Type* double_ty = irb.getIntNTy(sz * 2);
    llvm::Value* ext_op1 = irb.CreateCast(cast_op, op1, double_ty);
    llvm::Value* ext_op2 = irb.CreateCast(cast_op, op2, double_ty);
    llvm::Value* ext_res = irb.CreateMul(ext_op1, ext_op2);

    if (op_cnt == 1) {
        if (sz == 8) {
            StoreGp(ArchReg::RAX, ext_res);
        } else {
            // Don't use short_res to avoid having two multiplications.
            // TODO: is this concern still valid?
            llvm::Type* value_ty = irb.getIntNTy(sz);
            llvm::Value* res_a = irb.CreateTrunc(ext_res, value_ty);
            llvm::Value* high = irb.CreateLShr(ext_res, sz);
            llvm::Value* res_d = irb.CreateTrunc(high, value_ty);
            StoreGp(ArchReg::RAX, res_a);
            StoreGp(ArchReg::RDX, res_d);
        }
    } else {
        OpStoreGp(inst.op(0), short_res);
    }

    llvm::Value* overflow;
    if (cfg.enableOverflowIntrinsics) {
        llvm::Intrinsic::ID id = sign ? llvm::Intrinsic::smul_with_overflow
                                      : llvm::Intrinsic::umul_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, op1, op2);
        overflow = irb.CreateExtractValue(packed, 1);
    } else {
        llvm::Value* ext_short_res = irb.CreateCast(cast_op, short_res, double_ty);
        overflow = irb.CreateICmpNE(ext_res, ext_short_res);
    }

    SetReg(ArchReg::OF, overflow);
    SetReg(ArchReg::CF, overflow);
    SetFlagUndef({ArchReg::SF, ArchReg::ZF, ArchReg::AF, ArchReg::PF});
}

void Lifter::LiftDiv(const Instr& inst) {
    unsigned sz = inst.op(0).bits();
    bool sign = inst.type() == FDI_IDIV;
    auto ext_op = sign ? llvm::Instruction::SExt : llvm::Instruction::ZExt;
    auto div_op = sign ? llvm::Instruction::SDiv : llvm::Instruction::UDiv;
    auto rem_op = sign ? llvm::Instruction::SRem : llvm::Instruction::URem;

    // TODO: raise #DE on division by zero or overflow.

    llvm::Value* dividend = nullptr;
    if (sz == 8) {
        // Dividend is AX
        dividend = GetReg(ArchReg::RAX, Facet::I16);
    } else {
        // Dividend is DX:AX/EDX:EAX/RDX:RAX
        auto low = GetReg(ArchReg::RAX, Facet::In(sz));
        auto high = GetReg(ArchReg::RDX, Facet::In(sz));
        if (!sign) {
            // div is often preceded by xor edx, edx
            auto cnst = llvm::dyn_cast<llvm::Constant>(high);
            if (cnst && cnst->isNullValue())
                dividend = low;
        } else {
            auto binop = llvm::dyn_cast<llvm::BinaryOperator>(high);
            if (binop && binop->isArithmeticShift() && binop->getOperand(0) == low) {
                auto cnst = llvm::dyn_cast<llvm::ConstantInt>(binop->getOperand(1));
                if (cnst->equalsInt(sz - 1))
                    dividend = low;
            }
        }
        if (!dividend) {
            auto ex_ty = irb.getIntNTy(sz * 2);
            high = irb.CreateShl(irb.CreateZExt(high, ex_ty), sz);
            dividend = irb.CreateOr(irb.CreateZExt(low, ex_ty), high);
        }
    }

    // Divisor is the operand
    auto divisor = OpLoad(inst.op(0), Facet::I);
    divisor = irb.CreateCast(ext_op, divisor, dividend->getType());

    auto quot = irb.CreateBinOp(div_op, dividend, divisor);
    auto rem = irb.CreateBinOp(rem_op, dividend, divisor);

    auto val_ty = irb.getIntNTy(sz);
    quot = irb.CreateTrunc(quot, val_ty);
    rem = irb.CreateTrunc(rem, val_ty);

    if (sz == 8) {
        // Quotient is AL, remainder is AH
        StoreGpFacet(ArchReg::RAX, Facet::I8, quot);
        StoreGpFacet(ArchReg::RAX, Facet::I8H, rem);
    } else {
        // Quotient is AX/EAX/RAX, remainer is DX/EDX/RDX
        StoreGp(ArchReg::RAX, quot);
        StoreGp(ArchReg::RDX, rem);
    }

    SetFlagUndef({ArchReg::OF, ArchReg::SF, ArchReg::ZF, ArchReg::AF, ArchReg::PF,
                  ArchReg::CF});
}

void Lifter::LiftLea(const Instr& inst) {
    assert(inst.op(0).is_reg());
    assert(inst.op(1).is_mem());

    llvm::Value* res = nullptr;
    if (inst.op(1).base().ri == FD_REG_IP) {
        res = AddrIPRel(inst.op(1).off());
    } else {
        // Compute as integer
        unsigned addrsz = inst.op(1).addrsz() * 8;
        Facet facet = Facet{Facet::I}.Resolve(addrsz);
        if (inst.op(1).off())
            res = irb.getIntN(addrsz, inst.op(1).off());
        if (unsigned scale = inst.op(1).scale()) {
            llvm::Value* offset = GetReg(MapReg(inst.op(1).index()), facet);
            if (scale > 1) {
                unsigned shift = scale == 2 ? 1 : scale == 4 ? 2 : 3;
                offset = irb.CreateShl(offset, irb.getIntN(addrsz, shift));
            }
            res = res ? irb.CreateAdd(offset, res) : offset;
        }
        if (inst.op(1).base()){
            llvm::Value* reg = GetReg(MapReg(inst.op(1).base()), facet);
            res = res ? irb.CreateAdd(reg, res) : reg;
        }
        res = res ? res : irb.getIntN(addrsz, 0);
    }


    llvm::Type* op_type = irb.getIntNTy(inst.op(0).bits());
    OpStoreGp(inst.op(0), irb.CreateZExtOrTrunc(res, op_type));
}

void Lifter::LiftXlat(const Instr& inst) {
    llvm::Value* al = GetReg(ArchReg::RAX, Facet::I8);
    llvm::Value* bx;
    if (inst.addrsz() == 8) {
        bx = GetReg(ArchReg::RBX, Facet::PTR);
    } else {
        bx = GetReg(ArchReg::RBX, Facet::I32);
        bx = irb.CreateIntToPtr(bx, irb.getPtrTy());
    }

    llvm::Value* off = irb.CreateZExt(al, irb.getInt32Ty());
    llvm::Value* ptr = irb.CreateGEP(irb.getInt8Ty(), bx, off);
    StoreGp(ArchReg::RAX, irb.CreateLoad(irb.getInt8Ty(), ptr));
}

void Lifter::LiftCmovcc(const Instr& inst, Condition cond) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    // Note that 32-bit registers are still zero-extended when cond is false.
    OpStoreGp(inst.op(0), irb.CreateSelect(FlagCond(cond), op2, op1));
}

void Lifter::LiftSetcc(const Instr& inst, Condition cond) {
    OpStoreGp(inst.op(0), irb.CreateZExt(FlagCond(cond), irb.getInt8Ty()));
}

void Lifter::LiftCext(const Instr& inst) {
    unsigned sz = inst.opsz() * 8;
    llvm::Value* ax = GetReg(ArchReg::RAX, Facet::In(sz / 2));
    StoreGp(ArchReg::RAX, irb.CreateSExt(ax, irb.getIntNTy(sz)));
}

void Lifter::LiftCsep(const Instr& inst) {
    unsigned sz = inst.opsz() * 8;
    llvm::Value* ax = GetReg(ArchReg::RAX, Facet::In(sz));
    StoreGp(ArchReg::RDX, irb.CreateAShr(ax, sz - 1));
}

void Lifter::LiftBitscan(const Instr& inst, bool trailing) {
    unsigned sz = inst.op(1).bits();
    llvm::Value* dst = OpLoad(inst.op(0), sz == 16 ? Facet::I16 : Facet::I64);
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    auto id = trailing ? llvm::Intrinsic::cttz : llvm::Intrinsic::ctlz;
    llvm::Value* res = irb.CreateBinaryIntrinsic(id, src,
                                                 /*zero_undef=*/irb.getTrue());
    if (!trailing)
        res = irb.CreateSub(irb.getIntN(sz, sz - 1), res);
    // BSF/BSR don't modify dest if src is zero. This is specified on AMD, and
    // Intel apparently behaves similar on all x86-64 implementations. There is
    // also no zero extension of 32-bit operands.
    // See: https://sourceware.org/bugzilla/show_bug.cgi?id=31748
    llvm::Value* eq = irb.CreateIsNull(src);
    if (sz == 32) {
        res = irb.CreateZExt(res, irb.getInt64Ty());
        SetReg(MapReg(inst.op(0).reg()), irb.CreateSelect(eq, dst, res));
    } else {
        OpStoreGp(inst.op(0), irb.CreateSelect(eq, dst, res));
    }

    SetReg(ArchReg::ZF, eq);
    SetFlagUndef({ArchReg::OF, ArchReg::SF, ArchReg::AF, ArchReg::PF, ArchReg::CF});
}

void Lifter::LiftPopcnt(const Instr& inst) {
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    OpStoreGp(inst.op(0), irb.CreateUnaryIntrinsic(llvm::Intrinsic::ctpop, src));

    FlagCalcZ(src);
    SetReg(ArchReg::OF, irb.getFalse());
    SetReg(ArchReg::SF, irb.getFalse());
    SetReg(ArchReg::AF, irb.getFalse());
    SetReg(ArchReg::PF, irb.getInt8(1)); // constant 1 for low 8 bits -> PF=0
    SetReg(ArchReg::CF, irb.getFalse());
}

void Lifter::LiftBittest(const Instr& inst, llvm::Instruction::BinaryOps op,
                         llvm::AtomicRMWInst::BinOp atomic_op) {
    llvm::Value* index = OpLoad(inst.op(1), Facet::I);
    unsigned op_size = inst.op(0).bits();
    assert((op_size == 16 || op_size == 32 || op_size == 64) &&
            "invalid bittest operation size");

    llvm::Value* addr = nullptr;
    if (inst.op(0).is_mem()) {
        addr = OpAddr(inst.op(0), irb.getIntNTy(op_size));
        // Immediate operands are truncated, register operands are sign-extended
        if (inst.op(1).is_reg()) {
            llvm::Value* off = irb.CreateAShr(index, __builtin_ctz(op_size));
            off = irb.CreateSExt(off, irb.getInt64Ty());
            addr = irb.CreateGEP(irb.getIntNTy(op_size), addr, off);
        }
    }

    // Immediate operands are just 8 bits.
    if (index->getType()->getIntegerBitWidth() != op_size)
        index = irb.CreateZExt(index, irb.getIntNTy(op_size));
    // Truncated here because memory operand may need full value.
    index = irb.CreateAnd(index, irb.getIntN(op_size, op_size-1));
    llvm::Value* mask = irb.CreateShl(irb.getIntN(op_size, 1), index);
    llvm::Value* modmask = inst.type() != FDI_BTR ? mask : irb.CreateNot(mask);

    llvm::Value* val;
    if (inst.has_lock()) {
        auto ord = llvm::AtomicOrdering::SequentiallyConsistent;
        val = irb.CreateAtomicRMW(atomic_op, addr, modmask, {}, ord);
        goto skip_writeback;
    }

    if (inst.op(0).is_reg())
        val = OpLoad(inst.op(0), Facet::I);
    else
        val = irb.CreateLoad(irb.getIntNTy(op_size), addr);

    if (inst.type() != FDI_BT) {
        llvm::Value* newval = irb.CreateBinOp(op, val, modmask);
        if (inst.op(0).is_reg())
            OpStoreGp(inst.op(0), newval);
        else // LL_OP_MEM
            irb.CreateStore(newval, addr);
    }

skip_writeback:;
    llvm::Value* bit = irb.CreateAnd(val, mask);
    // Zero flag is not modified
    SetReg(ArchReg::CF, irb.CreateICmpNE(bit, irb.getIntN(op_size, 0)));
    SetFlagUndef({ArchReg::OF, ArchReg::SF, ArchReg::AF, ArchReg::PF});
}

void Lifter::LiftMovbe(const Instr& inst) {
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    OpStoreGp(inst.op(0), irb.CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftBswap(const Instr& inst) {
    assert(inst.op(0).is_reg() && "bswap with non-reg operand");
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    OpStoreGp(inst.op(0), irb.CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftJmp(const Instr& inst) {
    // Force default data segment, 3e is notrack.
    if (inst.op(0).is_pcrel())
        SetIP(inst.end() + inst.op(0).pcrel());
    else
        SetIP(OpLoad(inst.op(0), Facet::I64, ALIGN_NONE, FD_REG_DS));
}

void Lifter::LiftJcc(const Instr& inst, Condition cond) {
    SetIPCond(FlagCond(cond), inst.end() + inst.op(0).pcrel(), inst.end());
}

void Lifter::LiftJcxz(const Instr& inst) {
    unsigned sz = inst.addrsz();
    llvm::Value* cx = GetReg(ArchReg::RCX, Facet::In(sz * 8));
    llvm::Value* cond = irb.CreateICmpEQ(cx, irb.getIntN(sz*8, 0));
    SetIPCond(cond, inst.end() + inst.op(0).pcrel(), inst.end());
}

void Lifter::LiftLoop(const Instr& inst) {
    unsigned sz = inst.addrsz();

    // Decrement RCX/ECX
    llvm::Value* cx = GetReg(ArchReg::RCX, Facet::In(sz * 8));
    cx = irb.CreateSub(cx, irb.getIntN(sz * 8, 1));
    StoreGp(ArchReg::RCX, cx);

    // Construct condition
    llvm::Value* cond = irb.CreateICmpNE(cx, irb.getIntN(sz*8, 0));
    if (inst.type() == FDI_LOOPZ)
        cond = irb.CreateAnd(cond, GetFlag(ArchReg::ZF));
    else if (inst.type() == FDI_LOOPNZ)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(ArchReg::ZF)));

    SetIPCond(cond, inst.end() + inst.op(0).pcrel(), inst.end());
}

void Lifter::LiftCall(const Instr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({ArchReg::OF, ArchReg::SF, ArchReg::ZF, ArchReg::AF, ArchReg::PF,
                      ArchReg::CF});

    // Force default data segment, 3e is notrack.
    llvm::Value* new_rip;
    if (inst.op(0).is_pcrel())
        new_rip = AddrIPRel(inst.op(0).pcrel());
    else
        new_rip = OpLoad(inst.op(0), Facet::I, ALIGN_NONE, FD_REG_DS);
    StackPush(AddrIPRel()); // return address
    SetIP(new_rip);

    if (cfg.call_function) {
        CallExternalFunction(cfg.call_function);
        // Note that it is not possible to have a "no-evil-rets" optimization
        // which would just continue execution: things like setjmp/longjmp and
        // exceptions skip some return addresses by modifying the stack pointer.
        // We will continue with the tail_function (if specified) and enlarge
        // our shadow stack; and things will be slow. If someone uses such
        // constructs often or on a critical path, they get what they deserve.
        SetIPCallret(inst.end());
    }
}

void Lifter::LiftRet(const Instr& inst) {
    // TODO: support 16-bit address size override
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({ArchReg::OF, ArchReg::SF, ArchReg::ZF, ArchReg::AF, ArchReg::PF,
                      ArchReg::CF});

    SetIP(StackPop());

    if (inst.op(0)) {
        llvm::Value* rsp = GetReg(ArchReg::RSP, Facet::PTR);
        rsp = irb.CreateConstGEP1_64(irb.getInt8Ty(), rsp, inst.op(0).imm());
        SetReg(ArchReg::RSP, rsp);
    }

    if (cfg.call_function) {
        // If we are in call-ret-lifting mode, forcefully return. Otherwise, we
        // might end up using tail_function, which we don't want here.
        ForceReturn();
    }
}

void Lifter::LiftSyscall(const Instr& inst) {
    SetReg(ArchReg::RCX, AddrIPRel());
    SetReg(ArchReg::GP(11), FlagAsReg(64));

    if (cfg.syscall_implementation)
        CallExternalFunction(cfg.syscall_implementation);
}

void Lifter::LiftCpuid(const Instr& inst) {
    // TODO: serialize
    llvm::Value* eax = irb.getInt64(0);
    llvm::Value* ecx = irb.getInt64(0);
    llvm::Value* edx = irb.getInt64(0);
    llvm::Value* ebx = irb.getInt64(0);
    if (cfg.cpuinfo_function) {
        llvm::Value* in_eax = GetReg(ArchReg::RAX, Facet::I32);
        llvm::Value* in_ecx = GetReg(ArchReg::RCX, Facet::I32);
        llvm::Value* res = irb.CreateCall(cfg.cpuinfo_function, {in_eax, in_ecx});
        llvm::Value* ecxeax = irb.CreateExtractValue(res, {0});
        llvm::Value* ebxedx = irb.CreateExtractValue(res, {1});
        eax = irb.CreateAnd(ecxeax, irb.getInt64(0xffffffff));
        ecx = irb.CreateLShr(ecxeax, 32);
        edx = irb.CreateAnd(ebxedx, irb.getInt64(0xffffffff));
        ebx = irb.CreateLShr(ebxedx, 32);
    }
    SetReg(ArchReg::RAX, eax);
    SetReg(ArchReg::RCX, ecx);
    SetReg(ArchReg::RDX, edx);
    SetReg(ArchReg::RBX, ebx);
}

void Lifter::LiftRdtsc(const Instr& inst) {
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto id = llvm::Intrinsic::readcyclecounter;
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id);
    llvm::Value* res = irb.CreateCall(intrinsic);
    llvm::Value* lo = irb.CreateTrunc(res, irb.getInt32Ty());
    llvm::Value* hi = irb.CreateLShr(res, irb.getInt64(32));
    SetReg(ArchReg::RAX, lo);
    SetReg(ArchReg::RDX, hi);
}

Lifter::RepInfo Lifter::RepBegin(const Instr& inst) {
    RepInfo info = {};

    bool condrep = inst.type() == FDI_SCAS || inst.type() == FDI_CMPS;
    if (inst.has_rep())
        info.mode = condrep ? RepInfo::REPZ : RepInfo::REP;
    else if (inst.has_repnz() && condrep)
        info.mode = RepInfo::REPNZ;
    else
        info.mode = RepInfo::NO_REP;

    llvm::Value* df = GetFlag(ArchReg::DF);
    info.adj = irb.CreateSelect(df, irb.getInt64(-1), irb.getInt64(1));

    info.ty = irb.getIntNTy(inst.opsz() * 8);
    if (inst.type() != FDI_LODS)
        info.di = GetReg(ArchReg::RDI, Facet::PTR);
    if (inst.type() != FDI_STOS && inst.type() != FDI_SCAS)
        info.si = GetReg(ArchReg::RSI, Facet::PTR);

    // Iff instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode != RepInfo::NO_REP) {
        info.header_block = irb.GetInsertBlock();
        info.loop_block = llvm::BasicBlock::Create(irb.getContext(), "", fi.fn);
        info.cont_block = llvm::BasicBlock::Create(irb.getContext(), "", fi.fn);

        if (condrep) {
            info.flags[0] = GetFlag(ArchReg::OF);
            info.flags[1] = GetFlag(ArchReg::SF);
            info.flags[2] = GetFlag(ArchReg::ZF);
            info.flags[3] = GetFlag(ArchReg::AF);
            info.flags[4] = GetFlag(ArchReg::PF);
            info.flags[5] = GetFlag(ArchReg::CF);
        }

        llvm::Value* count = GetReg(ArchReg::RCX, Facet::I64);
        llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
        llvm::Value* enter_loop = irb.CreateICmpNE(count, zero);
        irb.CreateCondBr(enter_loop, info.loop_block, info.cont_block);
        SetInsertBlock(info.loop_block);

        auto createPhi = [&] (llvm::BasicBlock* bb, llvm::Value* val) {
            llvm::PHINode* phi = llvm::PHINode::Create(val->getType(), 2, "", bb);
            phi->addIncoming(val, info.header_block);
            return phi;
        };
        if (info.di) {
            info.cont_di = createPhi(info.cont_block, info.di);
            info.di = info.loop_di = createPhi(info.loop_block, info.di);
        }
        if (info.si) {
            info.cont_si = createPhi(info.cont_block, info.si);
            info.si = info.loop_si = createPhi(info.loop_block, info.si);
        }
        info.cont_count = createPhi(info.cont_block, count);
        info.loop_count = createPhi(info.loop_block, count);
    }

    return info;
}

void Lifter::RepEnd(RepInfo info, llvm::Value* cmpA, llvm::Value* cmpB) {
    // If instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode == RepInfo::NO_REP) {
        if (info.di)
            SetReg(ArchReg::RDI, irb.CreateGEP(info.ty, info.di, info.adj));
        if (info.si)
            SetReg(ArchReg::RSI, irb.CreateGEP(info.ty, info.si, info.adj));
        if (cmpA && cmpB) {
            // Perform a normal CMP operation.
            FlagCalcSub(irb.CreateSub(cmpA, cmpB), cmpA, cmpB);
        }
        return;
    }

    // First update pointer registers with direction flag
    if (info.loop_di) {
        llvm::Value* adjDi = irb.CreateGEP(info.ty, info.loop_di, info.adj);
        info.loop_di->addIncoming(adjDi, info.loop_block);
        info.cont_di->addIncoming(adjDi, info.loop_block);
    }
    if (info.loop_si) {
        llvm::Value* adjSi = irb.CreateGEP(info.ty, info.loop_si, info.adj);
        info.loop_si->addIncoming(adjSi, info.loop_block);
        info.cont_si->addIncoming(adjSi, info.loop_block);
    }

    // Decrement count and check.
    llvm::Value* count = info.loop_count;
    count = irb.CreateSub(count, irb.getInt64(1));
    info.loop_count->addIncoming(count, info.loop_block);
    info.cont_count->addIncoming(count, info.loop_block);

    llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
    llvm::Value* cond = irb.CreateICmpNE(count, zero);
    if (info.mode == RepInfo::REPZ)
        cond = irb.CreateAnd(cond, irb.CreateICmpEQ(cmpA, cmpB));
    else if (info.mode == RepInfo::REPNZ)
        cond = irb.CreateAnd(cond, irb.CreateICmpNE(cmpA, cmpB));

    irb.CreateCondBr(cond, info.loop_block, info.cont_block);
    SetInsertBlock(info.cont_block);

    if (cmpA && cmpB) {
        llvm::Value* lf = info.merge(irb.getFalse(), irb.getTrue());
        // Ok, very stupid flag computation. If someone uses these flags, they
        // get the slow code they want.
        cmpA = info.merge(llvm::ConstantInt::getNullValue(cmpA->getType()), cmpA);
        cmpB = info.merge(llvm::ConstantInt::getNullValue(cmpB->getType()), cmpB);

        // Perform a normal CMP operation.
        FlagCalcSub(irb.CreateSub(cmpA, cmpB), cmpA, cmpB);
        // Merge flags
        SetReg(ArchReg::OF, irb.CreateSelect(lf, GetFlag(ArchReg::OF), info.flags[0]));
        SetReg(ArchReg::SF, irb.CreateSelect(lf, GetFlag(ArchReg::SF), info.flags[1]));
        SetReg(ArchReg::ZF, irb.CreateSelect(lf, GetFlag(ArchReg::ZF), info.flags[2]));
        SetReg(ArchReg::AF, irb.CreateSelect(lf, GetFlag(ArchReg::AF), info.flags[3]));
        SetReg(ArchReg::PF, irb.CreateNot(irb.CreateSelect(lf, GetFlag(ArchReg::PF), info.flags[4])));
        SetReg(ArchReg::CF, irb.CreateSelect(lf, GetFlag(ArchReg::CF), info.flags[5]));
    }

    if (info.cont_di)
        SetReg(ArchReg::RDI, info.cont_di);
    if (info.cont_si)
        SetReg(ArchReg::RSI, info.cont_si);
    SetReg(ArchReg::RCX, info.cont_count);
}

void Lifter::LiftLods(const Instr& inst) {
    llvm::Value* oldAx = nullptr;
    if (inst.has_rep())
        oldAx = GetReg(ArchReg::RAX, Facet::In(inst.opsz() == 4 ? 64 : inst.opsz() * 8));

    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!
    llvm::Value* newAx = irb.CreateLoad(rep_info.ty, rep_info.si);
    if (inst.has_rep() && inst.opsz() == 4)
        newAx = irb.CreateZExt(newAx, oldAx->getType());
    RepEnd(rep_info); // NOTE: this modifies control flow!

    if (inst.has_rep()) {
        StoreGp(ArchReg::RAX, rep_info.merge(oldAx, newAx));
    } else {
        StoreGp(ArchReg::RAX, newAx);
    }
}

void Lifter::LiftStos(const Instr& inst) {
    if (inst.has_rep() && inst.opsz() == 1) {
        // TODO: respect address size
        // TODO: support stosw/stosd/stosq if rax == 0
        llvm::Type* ty = irb.getIntNTy(inst.opsz() * 8);
        auto di = GetReg(ArchReg::RDI, Facet::PTR);
        auto cx = GetReg(ArchReg::RCX, Facet::I64);
        auto ax = GetReg(ArchReg::RAX, Facet::I8);
        auto df = GetFlag(ArchReg::DF);

        auto* df0_block = llvm::BasicBlock::Create(irb.getContext(), "", fi.fn);
        auto* df1_block = llvm::BasicBlock::Create(irb.getContext(), "", fi.fn);
        auto* cont_block = llvm::BasicBlock::Create(irb.getContext(), "", fi.fn);
        auto* di_phi = llvm::PHINode::Create(di->getType(), 2, "", cont_block);

        irb.CreateCondBr(df, df1_block, df0_block);

        SetInsertBlock(df0_block);
        irb.CreateMemSet(di, ax, cx, llvm::Align());
        di_phi->addIncoming(irb.CreateGEP(ty, di, cx), df0_block);
        irb.CreateBr(cont_block);

        SetInsertBlock(df1_block);
        auto adj = irb.CreateSub(irb.getInt64(1), cx);
        auto base = irb.CreateGEP(ty, di, adj);
        irb.CreateMemSet(base, ax, cx, llvm::Align());
        di_phi->addIncoming(irb.CreateGEP(ty, base, irb.getInt64(-1)), df1_block);
        irb.CreateBr(cont_block);

        SetInsertBlock(cont_block);
        SetReg(ArchReg::RDI, di_phi);
        SetReg(ArchReg::RCX, irb.getInt64(0));
        return;
    }

    // TODO: optimize REP STOSB and other sizes with constant zero to llvm
    // memset intrinsic.
    auto ax = GetReg(ArchReg::RAX, Facet::In(inst.opsz() * 8));

    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!
    irb.CreateStore(ax, rep_info.di);
    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftMovs(const Instr& inst) {
    // TODO: optimize REP MOVSB to use llvm memcpy intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!
    irb.CreateStore(irb.CreateLoad(rep_info.ty, rep_info.si), rep_info.di);
    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftScas(const Instr& inst) {
    auto src = GetReg(ArchReg::RAX, Facet::In(inst.opsz() * 8));

    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!
    llvm::Value* dst = irb.CreateLoad(rep_info.ty, rep_info.di);
    RepEnd(rep_info, src, dst); // NOTE: this modifies control flow!
}

void Lifter::LiftCmps(const Instr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!
    llvm::Value* src = irb.CreateLoad(rep_info.ty, rep_info.si);
    llvm::Value* dst = irb.CreateLoad(rep_info.ty, rep_info.di);
    RepEnd(rep_info, src, dst); // NOTE: this modifies control flow!
}

} // namespace::x86_64

/**
 * @}
 **/
