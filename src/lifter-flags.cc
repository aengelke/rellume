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

#include "instr.h"
#include "regfile.h"
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>


/**
 * \defgroup LLFlags Flags
 * \brief Computation of X86 flags
 *
 * @{
 **/

namespace rellume {

llvm::Value*
LifterBase::FlagCond(Condition cond)
{
    llvm::Value* result = nullptr;
    switch (static_cast<Condition>(static_cast<int>(cond) & ~1))
    {
    case Condition::O:  result = GetFlag(Facet::OF); break;
    case Condition::C:  result = GetFlag(Facet::CF); break;
    case Condition::Z:  result = GetFlag(Facet::ZF); break;
    case Condition::BE: result = irb.CreateOr(GetFlag(Facet::CF), GetFlag(Facet::ZF)); break;
    case Condition::S:  result = GetFlag(Facet::SF); break;
    case Condition::P:  result = GetFlag(Facet::PF); break;
    case Condition::L:  result = irb.CreateICmpNE(GetFlag(Facet::SF), GetFlag(Facet::OF)); break;
    case Condition::LE: result = irb.CreateOr(GetFlag(Facet::ZF), irb.CreateICmpNE(GetFlag(Facet::SF), GetFlag(Facet::OF))); break;
    default: assert(0);
    }

    return static_cast<int>(cond) & 1 ? irb.CreateNot(result) : result;
}

static const std::pair<Facet, unsigned> RFLAGS_INDICES[] = {
    {Facet::CF, 0}, {Facet::PF, 2}, {Facet::AF, 4}, {Facet::ZF, 6},
    {Facet::SF, 7}, {Facet::DF, 10}, {Facet::OF, 11},
};

llvm::Value* LifterBase::FlagAsReg(unsigned size) {
    llvm::Value* res = irb.getInt64(0x202); // IF
    llvm::Type* ty = res->getType();
    for (auto& kv : RFLAGS_INDICES) {
        llvm::Value* ext_bit = irb.CreateZExt(GetFlag(kv.first), ty);
        res = irb.CreateOr(res, irb.CreateShl(ext_bit, kv.second));
    }
    return irb.CreateTruncOrBitCast(res, irb.getIntNTy(size));
}

void LifterBase::FlagFromReg(llvm::Value* val) {
    unsigned sz = val->getType()->getIntegerBitWidth();
    for (auto& kv : RFLAGS_INDICES) {
        if (kv.second >= sz)
            break;
        llvm::Value* bit = irb.CreateLShr(val, kv.second);
        SetFlag(kv.first, irb.CreateTrunc(bit, irb.getInt1Ty()));
    }
}

void
LifterBase::FlagCalcP(llvm::Value* value)
{
    llvm::Value* trunc = irb.CreateTruncOrBitCast(value, irb.getInt8Ty());
    llvm::Value* count = CreateUnaryIntrinsic(llvm::Intrinsic::ctpop, trunc);
    llvm::Value* bit = irb.CreateTruncOrBitCast(count, irb.getInt1Ty());
    SetFlag(Facet::PF, irb.CreateNot(bit));
}

void
LifterBase::FlagCalcA(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs)
{
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
                             llvm::Value* rhs, bool skip_carry) {
    auto zero = llvm::Constant::getNullValue(res->getType());
    llvm::Value* sf = irb.CreateICmpSLT(res, zero);  // also used for OF

    SetFlag(Facet::ZF, irb.CreateICmpEQ(lhs, rhs));
    SetFlag(Facet::SF, sf);
    FlagCalcP(res);
    FlagCalcA(res, lhs, rhs);
    if (!skip_carry)
        SetFlag(Facet::CF, irb.CreateICmpULT(lhs, rhs));

    // Set overflow flag using arithmetic comparisons
    SetFlag(Facet::OF, irb.CreateICmpNE(sf, irb.CreateICmpSLT(lhs, rhs)));
}

} // namespace

/**
 * @}
 **/
