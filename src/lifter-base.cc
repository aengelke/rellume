/**
 * This file is part of Rellume.
 *
 * (c) 2016-2020, Alexis Engelke <alexis.engelke@googlemail.com>
 * (c) 2020, Dominik Okwieka <dominik.okwieka@t-online.de>
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

#include "lifter-base.h"

#include "arch.h"
#include "basicblock.h"
#include "config.h"
#include "function-info.h"
#include "instr.h"

#include <llvm/IR/Instruction.h>
#include <llvm/Transforms/Utils/Cloning.h>

namespace rellume {

void LifterBase::SetIP(uint64_t inst_addr, bool nofold) {
    llvm::Value* off = irb.getInt64(inst_addr - fi.pc_base_addr);
    llvm::Value* rip = irb.CreateAdd(fi.pc_base_value, off);
    if (nofold) {
        auto bitcast = llvm::Instruction::BitCast;
        rip = irb.Insert(llvm::CastInst::Create(bitcast, rip, rip->getType()));
    }
    SetReg(ArchReg::IP, Facet::I64, rip);
}

llvm::Value* LifterBase::AddrConst(uint64_t addr, llvm::PointerType* ptr_ty) {
    if (addr == 0)
        return llvm::ConstantPointerNull::get(ptr_ty);

    if (cfg.global_base_value) {
        uintptr_t offset = addr - cfg.global_base_addr;
        auto ptr = irb.CreateGEP(cfg.global_base_value, irb.getInt64(offset));
        return irb.CreatePointerCast(ptr, ptr_ty);
    }

    return irb.CreateIntToPtr(irb.getInt64(addr), ptr_ty);
}

void LifterBase::FlagCalcP(llvm::Value* value) {
    llvm::Value* trunc = irb.CreateTruncOrBitCast(value, irb.getInt8Ty());
    llvm::Value* count = CreateUnaryIntrinsic(llvm::Intrinsic::ctpop, trunc);
    SetFlag(Facet::PF, irb.CreateNot(irb.CreateTrunc(count, irb.getInt1Ty())));
}

void LifterBase::FlagCalcA(llvm::Value* res, llvm::Value* lhs,
                           llvm::Value* rhs) {
    llvm::Value* tmp = irb.CreateXor(irb.CreateXor(lhs, rhs), res);
    llvm::Value* masked = irb.CreateAnd(tmp, llvm::ConstantInt::get(res->getType(), 16));
    SetFlag(Facet::AF, irb.CreateICmpNE(masked, llvm::Constant::getNullValue(res->getType())));
}

void LifterBase::FlagCalcAdd(llvm::Value* res, llvm::Value* lhs,
                             llvm::Value* rhs, bool skip_carry) {
    auto zero = llvm::Constant::getNullValue(res->getType());
    SetFlag(Facet::ZF, irb.CreateICmpEQ(res, zero));
    SetFlag(Facet::SF, irb.CreateICmpSLT(res, zero));
    FlagCalcP(res);
    FlagCalcA(res, lhs, rhs);
    if (!skip_carry)
        SetFlag(Facet::CF, irb.CreateICmpULT(res, lhs));

    if (cfg.enableOverflowIntrinsics) {
        llvm::Intrinsic::ID id = llvm::Intrinsic::sadd_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, lhs, rhs);
        SetFlag(Facet::OF, irb.CreateExtractValue(packed, 1));
    } else {
        llvm::Value* tmp1 = irb.CreateNot(irb.CreateXor(lhs, rhs));
        llvm::Value* tmp2 = irb.CreateAnd(tmp1, irb.CreateXor(res, lhs));
        SetFlag(Facet::OF, irb.CreateICmpSLT(tmp2, zero));
    }
}

void LifterBase::FlagCalcSub(llvm::Value* res, llvm::Value* lhs,
                             llvm::Value* rhs, bool skip_carry, bool alt_zf) {
    auto zero = llvm::Constant::getNullValue(res->getType());
    llvm::Value* sf = irb.CreateICmpSLT(res, zero);  // also used for OF

    if (alt_zf)
        SetFlag(Facet::ZF, irb.CreateICmpEQ(lhs, rhs));
    else
        SetFlag(Facet::ZF, irb.CreateICmpEQ(res, zero));
    SetFlag(Facet::SF, sf);
    FlagCalcP(res);
    FlagCalcA(res, lhs, rhs);
    if (!skip_carry)
        SetFlag(Facet::CF, irb.CreateICmpULT(lhs, rhs));

    // Set overflow flag using arithmetic comparisons
    SetFlag(Facet::OF, irb.CreateICmpNE(sf, irb.CreateICmpSLT(lhs, rhs)));
}

void LifterBase::CallExternalFunction(llvm::Function* fn) {
    CallConv cconv = CallConv::FromFunction(fn, cfg.arch);
    llvm::CallInst* call = cconv.Call(fn, ablock.GetInsertBlock(), fi);
    assert(call && "failed to create call for external function");

    // Directly inline alwaysinline functions
    if (fn->hasFnAttribute(llvm::Attribute::AlwaysInline)) {
        llvm::InlineFunctionInfo ifi;
#if LL_LLVM_MAJOR < 11
        llvm::InlineFunction(llvm::CallSite(call), ifi);
#else
        llvm::InlineFunction(*call, ifi);
#endif
    }
}

} // namespace rellume
