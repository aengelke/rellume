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

#include "lifter-private.h"

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

namespace rellume {

void Lifter::LiftMovgp(const Instr& inst, llvm::Instruction::CastOps cast) {
    // TODO: if the instruction moves the whole register, keep all facets.
    // TODO: implement this for all register-register moves.

    llvm::Value* val = OpLoad(inst.op(1), Facet::I);
    llvm::Type* tgt_ty = irb.getIntNTy(inst.op(0).bits());
    OpStoreGp(inst.op(0), irb.CreateCast(cast, val, tgt_ty));
}

// Implementation of ADD, ADC, SUB, SBB, CMP, and XADD
void Lifter::LiftArith(const Instr& inst, bool sub) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    if (inst.type() == FDI_ADC || inst.type() == FDI_SBB)
        op2 = irb.CreateAdd(op2, irb.CreateZExt(GetFlag(Facet::CF), op2->getType()));

    auto arith_op = sub ? llvm::Instruction::Sub : llvm::Instruction::Add;
    llvm::Value* res = irb.CreateBinOp(arith_op, op1, op2);

    if (inst.type() != FDI_CMP)
        OpStoreGp(inst.op(0), res);
    if (inst.type() == FDI_XADD)
        OpStoreGp(inst.op(1), op1);

    if (sub)
        FlagCalcSub(res, op1, op2);
    else
        FlagCalcAdd(res, op1, op2);
}

void Lifter::LiftCmpxchg(const Instr& inst) {
    auto acc = GetReg(X86Reg::RAX, Facet::In(inst.op(0).bits()));
    auto dst = OpLoad(inst.op(0), Facet::I);
    auto src = OpLoad(inst.op(1), Facet::I);

    // Full compare with acc and dst
    llvm::Value* cmp_res = irb.CreateSub(acc, dst);
    FlagCalcSub(cmp_res, acc, dst);

    // Store SRC if DST=ACC, else store DST again (i.e. don't change memory).
    OpStoreGp(inst.op(0), irb.CreateSelect(GetFlag(Facet::ZF), src, dst));
    // ACC gets the value from memory.
    OpStoreGp(X86Reg::RAX, dst);
}

void Lifter::LiftXchg(const Instr& inst) {
    // TODO: atomic memory operation
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    OpStoreGp(inst.op(0), op2);
    OpStoreGp(inst.op(1), op1);
}

void Lifter::LiftAndOrXor(const Instr& inst, llvm::Instruction::BinaryOps op,
                           bool writeback) {
    llvm::Value* res = irb.CreateBinOp(op, OpLoad(inst.op(0), Facet::I),
                                       OpLoad(inst.op(1), Facet::I));
    if (writeback)
        OpStoreGp(inst.op(0), res);

    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlagUndef({Facet::AF});
    SetFlag(Facet::CF, irb.getFalse());
    SetFlag(Facet::OF, irb.getFalse());
}

void Lifter::LiftNot(const Instr& inst) {
    OpStoreGp(inst.op(0), irb.CreateNot(OpLoad(inst.op(0), Facet::I)));
}

void Lifter::LiftNeg(const Instr& inst) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* res = irb.CreateNeg(op1);
    llvm::Value* zero = llvm::Constant::getNullValue(res->getType());
    FlagCalcSub(res, zero, op1);
    OpStoreGp(inst.op(0), res);
}

void Lifter::LiftIncDec(const Instr& inst) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = irb.getIntN(inst.op(0).bits(), 1);
    llvm::Value* res = nullptr;
    if (inst.type() == FDI_INC) {
        res = irb.CreateAdd(op1, op2);
        FlagCalcOAdd(res, op1, op2);
    } else if (inst.type() == FDI_DEC) {
        res = irb.CreateSub(op1, op2);
        FlagCalcOSub(res, op1, op2);
    }

    // Carry flag is _not_ updated.
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, op1, op2);
    OpStoreGp(inst.op(0), res);
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
    FlagCalcS(res);
    FlagCalcZ(res);
    FlagCalcP(res);
    SetFlag(Facet::CF, irb.CreateTrunc(cf_big, irb.getInt1Ty()));
    llvm::Value* zero = llvm::ConstantInt::get(src->getType(), 0);
    SetFlag(Facet::OF, irb.CreateICmpSLT(irb.CreateXor(src, res), zero));
    SetFlagUndef({Facet::AF});
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
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlagUndef({Facet::OF, Facet::AF, Facet::CF});
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
    SetFlagUndef({Facet::OF, Facet::CF});
}

void Lifter::LiftMul(const Instr& inst) {
    unsigned sz = inst.op(0).bits();
    bool sign = inst.type() == FDI_IMUL;

    unsigned op_cnt = inst.op(2) ? 3 : inst.op(1) ? 2 : 1;

    llvm::Value* op1;
    llvm::Value* op2;
    if (op_cnt == 1) {
        op1 = OpLoad(inst.op(0), Facet::I);
        op2 = GetReg(X86Reg::RAX, Facet::In(sz));
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
            OpStoreGp(X86Reg::RAX, ext_res);
        } else {
            // Don't use short_res to avoid having two multiplications.
            // TODO: is this concern still valid?
            llvm::Type* value_ty = irb.getIntNTy(sz);
            llvm::Value* res_a = irb.CreateTrunc(ext_res, value_ty);
            llvm::Value* high = irb.CreateLShr(ext_res, sz);
            llvm::Value* res_d = irb.CreateTrunc(high, value_ty);
            OpStoreGp(X86Reg::RAX, res_a);
            OpStoreGp(X86Reg::RDX, res_d);
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

    SetFlag(Facet::OF, overflow);
    SetFlag(Facet::CF, overflow);
    SetFlagUndef({Facet::SF, Facet::ZF, Facet::AF, Facet::PF});
}

void Lifter::LiftDiv(const Instr& inst) {
    unsigned sz = inst.op(0).bits();
    bool sign = inst.type() == FDI_IDIV;
    auto ext_op = sign ? llvm::Instruction::SExt : llvm::Instruction::ZExt;
    auto div_op = sign ? llvm::Instruction::SDiv : llvm::Instruction::UDiv;
    auto rem_op = sign ? llvm::Instruction::SRem : llvm::Instruction::URem;

    // TODO: raise #DE on division by zero or overflow.

    auto ex_ty = irb.getIntNTy(sz * 2);

    llvm::Value* dividend;
    if (sz == 8) {
        // Dividend is AX
        dividend = GetReg(X86Reg::RAX, Facet::I16);
    } else {
        // Dividend is DX:AX/EDX:EAX/RDX:RAX
        auto low = GetReg(X86Reg::RAX, Facet::In(sz));
        auto high = GetReg(X86Reg::RDX, Facet::In(sz));
        high = irb.CreateShl(irb.CreateZExt(high, ex_ty), sz);
        dividend = irb.CreateOr(irb.CreateZExt(low, ex_ty), high);
    }

    // Divisor is the operand
    auto divisor = irb.CreateCast(ext_op, OpLoad(inst.op(0), Facet::I), ex_ty);

    auto quot = irb.CreateBinOp(div_op, dividend, divisor);
    auto rem = irb.CreateBinOp(rem_op, dividend, divisor);

    auto val_ty = irb.getIntNTy(sz);
    quot = irb.CreateTrunc(quot, val_ty);
    rem = irb.CreateTrunc(rem, val_ty);

    if (sz == 8) {
        // Quotient is AL, remainder is AH
        OpStoreGp(X86Reg::RAX, Facet::I8, quot);
        OpStoreGp(X86Reg::RAX, Facet::I8H, rem);
    } else {
        // Quotient is AX/EAX/RAX, remainer is DX/EDX/RDX
        OpStoreGp(X86Reg::RAX, quot);
        OpStoreGp(X86Reg::RDX, rem);
    }

    SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                  Facet::CF});
}

void Lifter::LiftLea(const Instr& inst) {
    assert(inst.op(0).is_reg());
    assert(inst.op(1).is_mem());

    // Compute as integer
    unsigned addrsz = inst.op(1).addrsz() * 8;
    Facet facet = Facet{Facet::I}.Resolve(addrsz);
    llvm::Value* res = irb.getIntN(addrsz, inst.op(1).off());
    if (inst.op(1).base())
        res = irb.CreateAdd(res, GetReg(MapReg(inst.op(1).base()), facet));
    if (inst.op(1).scale() != 0) {
        llvm::Value* offset = GetReg(MapReg(inst.op(1).index()), facet);
        offset = irb.CreateMul(offset, irb.getIntN(addrsz, inst.op(1).scale()));
        res = irb.CreateAdd(res, offset);
    }

    llvm::Type* op_type = irb.getIntNTy(inst.op(0).bits());
    OpStoreGp(inst.op(0), irb.CreateZExtOrTrunc(res, op_type));
}

void Lifter::LiftXlat(const Instr& inst) {
    llvm::Value* al = GetReg(X86Reg::RAX, Facet::I8);
    llvm::Value* bx;
    if (inst.addrsz() == 8) {
        bx = GetReg(X86Reg::RBX, Facet::PTR);
        bx = irb.CreatePointerCast(bx, irb.getInt8PtrTy());
    } else {
        bx = GetReg(X86Reg::RBX, Facet::I32);
        bx = irb.CreateZExt(bx, irb.getInt64Ty());
        bx = irb.CreateIntToPtr(bx, irb.getInt8PtrTy());
    }

    llvm::Value* ptr = irb.CreateGEP(bx, irb.CreateZExt(al, irb.getInt32Ty()));
    OpStoreGp(X86Reg::RAX, irb.CreateLoad(irb.getInt8Ty(), ptr));
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
    llvm::Value* ax = GetReg(X86Reg::RAX, Facet::In(sz / 2));
    OpStoreGp(X86Reg::RAX, irb.CreateSExt(ax, irb.getIntNTy(sz)));
}

void Lifter::LiftCsep(const Instr& inst) {
    unsigned sz = inst.opsz() * 8;
    llvm::Value* ax = GetReg(X86Reg::RAX, Facet::In(sz));
    OpStoreGp(X86Reg::RDX, irb.CreateAShr(ax, sz - 1));
}

void Lifter::LiftBitscan(const Instr& inst, bool trailing) {
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    auto id = trailing ? llvm::Intrinsic::cttz : llvm::Intrinsic::ctlz;
    llvm::Value* res = irb.CreateBinaryIntrinsic(id, src,
                                                 /*zero_undef=*/irb.getTrue());
    if (!trailing) {
        unsigned sz = inst.op(1).bits();
        res = irb.CreateSub(irb.getIntN(sz, sz - 1), res);
    }
    OpStoreGp(inst.op(0), res);

    FlagCalcZ(src);
    SetFlagUndef({Facet::OF, Facet::SF, Facet::AF, Facet::PF, Facet::CF});
}

void Lifter::LiftBittest(const Instr& inst) {
    llvm::Value* index = OpLoad(inst.op(1), Facet::I);
    unsigned op_size = inst.op(0).bits();
    assert((op_size == 16 || op_size == 32 || op_size == 64) &&
            "invalid bittest operation size");

    llvm::Value* val;
    llvm::Value* addr = nullptr;
    if (inst.op(0).is_reg()) {
        val = OpLoad(inst.op(0), Facet::I);
    } else { // LL_OP_MEM
        addr = OpAddr(inst.op(0), irb.getIntNTy(op_size), inst.op(0).seg());
        // Immediate operands are truncated, register operands are sign-extended
        if (inst.op(1).is_reg()) {
            llvm::Value* off = irb.CreateAShr(index, __builtin_ctz(op_size));
            addr = irb.CreateGEP(addr, irb.CreateSExt(off, irb.getInt64Ty()));
        }
        val = irb.CreateLoad(addr);
    }

    // Truncated here because memory operand may need full value.
    index = irb.CreateAnd(index, irb.getIntN(op_size, op_size-1));
    llvm::Value* mask = irb.CreateShl(irb.getIntN(op_size, 1), index);

    llvm::Value* bit = irb.CreateAnd(val, mask);

    if (inst.type() == FDI_BT) {
        goto skip_writeback;
    } else if (inst.type() == FDI_BTC) {
        val = irb.CreateXor(val, mask);
    } else if (inst.type() == FDI_BTR) {
        val = irb.CreateAnd(val, irb.CreateNot(mask));
    } else if (inst.type() == FDI_BTS) {
        val = irb.CreateOr(val, mask);
    }

    if (inst.op(0).is_reg())
        OpStoreGp(inst.op(0), val);
    else // LL_OP_MEM
        irb.CreateStore(val, addr);

skip_writeback:
    // Zero flag is not modified
    SetFlag(Facet::CF, irb.CreateICmpNE(bit, irb.getIntN(op_size, 0)));
    SetFlagUndef({Facet::OF, Facet::SF, Facet::AF, Facet::PF});
}

void Lifter::LiftMovbe(const Instr& inst) {
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    OpStoreGp(inst.op(0), CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftBswap(const Instr& inst) {
    assert(inst.op(0).is_reg() && "bswap with non-reg operand");
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    OpStoreGp(inst.op(0), CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftJmp(const Instr& inst) {
    // Force default data segment, 3e is notrack.
    SetReg(X86Reg::IP, Facet::I64, OpLoad(inst.op(0), Facet::I64, ALIGN_NONE,
                                          FD_REG_DS));
}

void Lifter::LiftJcc(const Instr& inst, Condition cond) {
    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(FlagCond(cond),
        OpLoad(inst.op(0), Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftJcxz(const Instr& inst) {
    unsigned sz = inst.addrsz();
    llvm::Value* cx = GetReg(X86Reg::RCX, Facet::In(sz * 8));
    llvm::Value* cond = irb.CreateICmpEQ(cx, irb.getIntN(sz*8, 0));
    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(cond,
        OpLoad(inst.op(0), Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftLoop(const Instr& inst) {
    unsigned sz = inst.addrsz();

    // Decrement RCX/ECX
    llvm::Value* cx = GetReg(X86Reg::RCX, Facet::In(sz * 8));
    cx = irb.CreateSub(cx, irb.getIntN(sz * 8, 1));
    OpStoreGp(X86Reg::RCX, cx);

    // Construct condition
    llvm::Value* cond = irb.CreateICmpNE(cx, irb.getIntN(sz*8, 0));
    if (inst.type() == FDI_LOOPZ)
        cond = irb.CreateAnd(cond, GetFlag(Facet::ZF));
    else if (inst.type() == FDI_LOOPNZ)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(Facet::ZF)));

    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(cond,
        OpLoad(inst.op(0), Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftCall(const Instr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                      Facet::CF});

    // Force default data segment, 3e is notrack.
    llvm::Value* new_rip = OpLoad(inst.op(0), Facet::I, ALIGN_NONE, FD_REG_DS);
    StackPush(GetReg(X86Reg::IP, Facet::I64));
    SetReg(X86Reg::IP, Facet::I64, new_rip);
}

void Lifter::LiftRet(const Instr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                      Facet::CF});

    SetReg(X86Reg::IP, Facet::I64, StackPop());
}

LifterBase::RepInfo LifterBase::RepBegin(const Instr& inst) {
    RepInfo info = {};

    bool condrep = inst.type() == FDI_SCAS || inst.type() == FDI_CMPS;
    if (inst.has_rep())
        info.mode = condrep ? RepInfo::REPZ : RepInfo::REP;
    else if (inst.has_repnz() && condrep)
        info.mode = RepInfo::REPNZ;
    else
        info.mode = RepInfo::NO_REP;

    // Iff instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode != RepInfo::NO_REP) {
        info.loop_block = ablock.AddBlock();
        info.cont_block = ablock.AddBlock();

        llvm::Value* count = GetReg(X86Reg::RCX, Facet::I64);
        llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
        llvm::Value* enter_loop = irb.CreateICmpNE(count, zero);
        ablock.GetInsertBlock()->BranchTo(enter_loop, *info.loop_block,
                                          *info.cont_block);

        SetInsertBlock(info.loop_block);
    }

    llvm::Type* op_ty = irb.getIntNTy(inst.opsz() * 8)->getPointerTo();
    if (inst.type() != FDI_LODS)
        info.di = irb.CreatePointerCast(GetReg(X86Reg::RDI, Facet::PTR), op_ty);
    if (inst.type() != FDI_STOS && inst.type() != FDI_SCAS)
        info.si = irb.CreatePointerCast(GetReg(X86Reg::RSI, Facet::PTR), op_ty);

    return info;
}

void LifterBase::RepEnd(RepInfo info) {
    // First update pointer registers with direction flag
    llvm::Value* df = GetFlag(Facet::DF);
    llvm::Value* adj = irb.CreateSelect(df, irb.getInt64(-1), irb.getInt64(1));

    if (info.di)
        SetRegPtr(X86Reg::RDI, irb.CreateGEP(info.di, adj));
    if (info.si)
        SetRegPtr(X86Reg::RSI, irb.CreateGEP(info.si, adj));

    // If instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode == RepInfo::NO_REP)
        return;

    // Decrement count and check.
    llvm::Value* count = GetReg(X86Reg::RCX, Facet::I64);
    count = irb.CreateSub(count, irb.getInt64(1));
    SetReg(X86Reg::RCX, Facet::I64, count);

    llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
    llvm::Value* cond = irb.CreateICmpNE(count, zero);
    if (info.mode == RepInfo::REPZ)
        cond = irb.CreateAnd(cond, GetFlag(Facet::ZF));
    else if (info.mode == RepInfo::REPNZ)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(Facet::ZF)));

    ablock.GetInsertBlock()->BranchTo(cond, *info.loop_block, *info.cont_block);
    SetInsertBlock(info.cont_block);
}

void Lifter::LiftLods(const Instr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    unsigned size = inst.opsz();
    OpStoreGp(X86Reg::RAX, irb.CreateLoad(irb.getIntNTy(size), rep_info.di));

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftStos(const Instr& inst) {
    // TODO: optimize REP STOSB and other sizes with constant zero to llvm
    // memset intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    auto ax = GetReg(X86Reg::RAX, Facet::In(inst.opsz() * 8));
    irb.CreateStore(ax, rep_info.di);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftMovs(const Instr& inst) {
    // TODO: optimize REP MOVSB to use llvm memcpy intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    irb.CreateStore(irb.CreateLoad(rep_info.si), rep_info.di);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftScas(const Instr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    auto src = GetReg(X86Reg::RAX, Facet::In(inst.opsz() * 8));
    llvm::Value* dst = irb.CreateLoad(rep_info.di);
    // Perform a normal CMP operation.
    FlagCalcSub(irb.CreateSub(src, dst), src, dst);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftCmps(const Instr& inst) {
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
