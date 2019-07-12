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
LLStateBase::FlagCond(LLInstrType type, LLInstrType base)
{
    RegFile::FlagCache& cache = regfile.GetFlagCache();
    int condition = type - base;
    if (cache.valid)
    {
        switch (condition)
        {
        case 6: return irb.CreateICmpULE(cache.lhs, cache.rhs);
        case 7: return irb.CreateICmpUGT(cache.lhs, cache.rhs);
        case 12: return irb.CreateICmpSLT(cache.lhs, cache.rhs);
        case 13: return irb.CreateICmpSGE(cache.lhs, cache.rhs);
        case 14: return irb.CreateICmpSLE(cache.lhs, cache.rhs);
        case 15: return irb.CreateICmpSGT(cache.lhs, cache.rhs);
        }
    }

    llvm::Value* result;
    switch (condition & ~1)
    {
    case 0: result = GetFlag(RFLAG_OF); break;
    case 2: result = GetFlag(RFLAG_CF); break;
    case 4: result = GetFlag(RFLAG_ZF); break;
    case 6: result = irb.CreateOr(GetFlag(RFLAG_CF), GetFlag(RFLAG_ZF)); break;
    case 8: result = GetFlag(RFLAG_SF); break;
    case 10: result = GetFlag(RFLAG_PF); break;
    case 12: result = irb.CreateICmpNE(GetFlag(RFLAG_SF), GetFlag(RFLAG_OF)); break;
    case 14: result = irb.CreateOr(GetFlag(RFLAG_ZF), irb.CreateICmpNE(GetFlag(RFLAG_SF), GetFlag(RFLAG_OF))); break;
    }

    return condition & 1 ? irb.CreateNot(result) : result;
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
