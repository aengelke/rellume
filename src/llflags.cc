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

#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm-c/Core.h>

#include <llstate-internal.h>

#include <llcommon-internal.h>
#include <llregfile-internal.h>
#include <rellume/instr.h>

/**
 * \defgroup LLFlags Flags
 * \brief Computation of X86 flags
 *
 * Ported from https://github.com/trailofbits/mcsema .
 *
 * @{
 **/

llvm::Value*
LLStateBase::FlagCond(Condition cond)
{
    RegFile::FlagCache& cache = regfile.GetFlagCache();
    if (cache.valid)
    {
        switch (cond)
        {
        case Condition::BE: return irb.CreateICmpULE(cache.lhs, cache.rhs);
        case Condition::A:  return irb.CreateICmpUGT(cache.lhs, cache.rhs);
        case Condition::L:  return irb.CreateICmpSLT(cache.lhs, cache.rhs);
        case Condition::GE: return irb.CreateICmpSGE(cache.lhs, cache.rhs);
        case Condition::LE: return irb.CreateICmpSLE(cache.lhs, cache.rhs);
        case Condition::G:  return irb.CreateICmpSGT(cache.lhs, cache.rhs);
        default: break;
        }
    }

    llvm::Value* result;
    switch (static_cast<Condition>(static_cast<int>(cond) & ~1))
    {
    case Condition::O:  result = GetFlag(RFLAG_OF); break;
    case Condition::C:  result = GetFlag(RFLAG_CF); break;
    case Condition::Z:  result = GetFlag(RFLAG_ZF); break;
    case Condition::BE: result = irb.CreateOr(GetFlag(RFLAG_CF), GetFlag(RFLAG_ZF)); break;
    case Condition::S:  result = GetFlag(RFLAG_SF); break;
    case Condition::P:  result = GetFlag(RFLAG_PF); break;
    case Condition::L:  result = irb.CreateICmpNE(GetFlag(RFLAG_SF), GetFlag(RFLAG_OF)); break;
    case Condition::LE: result = irb.CreateOr(GetFlag(RFLAG_ZF), irb.CreateICmpNE(GetFlag(RFLAG_SF), GetFlag(RFLAG_OF))); break;
    default: assert(0);
    }

    return static_cast<int>(cond) & 1 ? irb.CreateNot(result) : result;
}

llvm::Value* LLStateBase::FlagAsReg(unsigned size) {
    llvm::Value* res = irb.getInt64(0x202); // IF
    llvm::Type* ty = res->getType();
    static const std::pair<int, unsigned> flags[] = {
        {RFLAG_CF, 0}, {RFLAG_PF, 2}, {RFLAG_AF, 4}, {RFLAG_ZF, 6},
        {RFLAG_SF, 7}, {RFLAG_OF, 11},
    };
    for (auto& kv : flags) {
        llvm::Value* ext_bit = irb.CreateZExt(GetFlag(kv.first), ty);
        res = irb.CreateOr(res, irb.CreateShl(ext_bit, kv.second));
    }
    return irb.CreateTruncOrBitCast(res, irb.getIntNTy(size));
}

void
LLStateBase::FlagCalcP(llvm::Value* value)
{
    llvm::Value* trunc = irb.CreateTruncOrBitCast(value, irb.getInt8Ty());
#if LL_LLVM_MAJOR >= 8
    llvm::Value* count = irb.CreateUnaryIntrinsic(llvm::Intrinsic::ctpop, trunc);
#else
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto id = llvm::Intrinsic::ctpop;
    llvm::Function* intrinsic = llvm::Intrinsic::getDeclaration(module, id, {irb.getInt8Ty()});
    llvm::Value* count = irb.CreateCall(intrinsic, {trunc});
#endif
    llvm::Value* bit = irb.CreateTruncOrBitCast(count, irb.getInt1Ty());
    SetFlag(RFLAG_PF, irb.CreateNot(bit));
}

void
LLStateBase::FlagCalcA(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs)
{
    llvm::Value* tmp = irb.CreateXor(irb.CreateXor(lhs, rhs), res);
    llvm::Value* masked = irb.CreateAnd(tmp, llvm::ConstantInt::get(res->getType(), 16));
    SetFlag(RFLAG_AF, irb.CreateICmpNE(masked, llvm::Constant::getNullValue(res->getType())));
}

void
LLStateBase::FlagCalcOAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs)
{
    if (cfg.enableOverflowIntrinsics)
    {
        llvm::Intrinsic::ID id = llvm::Intrinsic::sadd_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, lhs, rhs);
        SetFlag(RFLAG_OF, irb.CreateExtractValue(packed, 1));
    }
    else
    {
        llvm::Value* tmp1 = irb.CreateNot(irb.CreateXor(lhs, rhs));
        llvm::Value* tmp2 = irb.CreateAnd(tmp1, irb.CreateXor(res, lhs));
        SetFlag(RFLAG_OF, irb.CreateICmpSLT(tmp2, llvm::Constant::getNullValue(res->getType())));
    }
}

void
LLStateBase::FlagCalcOSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs)
{
    if (cfg.enableOverflowIntrinsics)
    {
        llvm::Intrinsic::ID id = llvm::Intrinsic::ssub_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, lhs, rhs);
        SetFlag(RFLAG_OF, irb.CreateExtractValue(packed, 1));
    }
    else
    {
        auto tmp = irb.CreateAnd(irb.CreateXor(lhs, rhs), irb.CreateXor(res, lhs));
        SetFlag(RFLAG_OF, irb.CreateICmpSLT(tmp, llvm::Constant::getNullValue(res->getType())));
    }
}

/**
 * @}
 **/
