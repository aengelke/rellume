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

#include <llflags-internal.h>

#include <llcommon-internal.h>
#include <llinstr-internal.h>
#include <llregfile-internal.h>
#include <llstate-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLFlags Flags
 * \brief Computation of X86 flags
 *
 * Ported from https://github.com/trailofbits/mcsema .
 *
 * @{
 **/

/**
 * Evaluate a condition based on the condition from the instruction type.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param type The instruction type
 * \param base The base instruction type (this is the overflow variant)
 * \param state The module state
 **/
LLVMValueRef
ll_flags_condition(LLInstrType type, LLInstrType base, LLState* state)
{
    int condition = type - base;
    int conditionType = condition >> 1;
    bool negate = condition & 1;

    LLVMValueRef result;
    RegFile::FlagCache& flagCache = state->regfile->GetFlagCache();

    switch (conditionType)
    {
        case 0: // JO / JNO
            result = ll_get_flag(RFLAG_OF, state);
            break;
        case 1: // JC / JNC
            result = ll_get_flag(RFLAG_CF, state);
            break;
        case 2: // JZ / JNZ
            result = ll_get_flag(RFLAG_ZF, state);
            break;
        case 3: // JBE / JA
            if (flagCache.valid)
            {
                result = llvm::wrap(state->irb.CreateICmpULE(flagCache.lhs, flagCache.rhs));
            }
            else
            {
                result = LLVMBuildOr(state->builder, ll_get_flag(RFLAG_CF, state), ll_get_flag(RFLAG_ZF, state), "");
            }
            break;
        case 4: // JS / JNS
            result = ll_get_flag(RFLAG_SF, state);
            break;
        case 5: // JP / JNP
            result = ll_get_flag(RFLAG_PF, state);
            break;
        case 6: // JL / JGE
            if (flagCache.valid)
            {
                result = llvm::wrap(state->irb.CreateICmpSLT(flagCache.lhs, flagCache.rhs));
            }
            else
            {
                result = LLVMBuildICmp(state->builder, LLVMIntNE, ll_get_flag(RFLAG_SF, state), ll_get_flag(RFLAG_OF, state), "");
            }
            break;
        case 7: // JLE / JG
            if (flagCache.valid)
            {
                result = llvm::wrap(state->irb.CreateICmpSLE(flagCache.lhs, flagCache.rhs));
            }
            else
            {
                result = LLVMBuildICmp(state->builder, LLVMIntNE, ll_get_flag(RFLAG_SF, state), ll_get_flag(RFLAG_OF, state), "");
                result = LLVMBuildOr(state->builder, result, ll_get_flag(RFLAG_ZF, state), "");
            }
            break;
        default:
            result = NULL;
            break;
    }

    if (negate)
    {
        result = LLVMBuildNot(state->builder, result, "");
    }

    return result;
}

void
ll_flags_set_of_imul(LLVMValueRef result, LLVMValueRef lhs, LLVMValueRef rhs, LLState* state)
{
    LLVMTypeRef intType = LLVMTypeOf(lhs);
    LLVMTypeRef intLargeType = LLVMIntTypeInContext(state->context, LLVMGetIntTypeWidth(intType) * 2);
    LLVMValueRef overflowFlag;

    if (state->cfg.enableOverflowIntrinsics)
    {
        LLVMValueRef intrinsicSmulWithOverflow = ll_support_get_intrinsic(state->builder, LL_INTRINSIC_SMUL_WITH_OVERFLOW, &intType, 1);
        LLVMValueRef args[2] = { lhs, rhs };
        LLVMValueRef packedData = LLVMBuildCall(state->builder, intrinsicSmulWithOverflow, args, 2, "");
        overflowFlag = LLVMBuildExtractValue(state->builder, packedData, 1, "");
    }
    else
    {
        LLVMValueRef longResult;
        LLVMValueRef shortResult;

        if (LLVMTypeOf(result) != intLargeType)
        {
            lhs = LLVMBuildCast(state->builder, LLVMSExt, lhs, intLargeType, "");
            rhs = LLVMBuildCast(state->builder, LLVMSExt, rhs, intLargeType, "");
            longResult = LLVMBuildMul(state->builder, lhs, rhs, "");
            shortResult = result;
        }
        else
        {
            longResult = result;
            shortResult = LLVMBuildTrunc(state->builder, result, intType, "");
        }

        shortResult = LLVMBuildSExt(state->builder, shortResult, intLargeType, "");
        overflowFlag = LLVMBuildICmp(state->builder, LLVMIntNE, longResult, shortResult, "");
    }

    ll_set_flag(RFLAG_OF, overflowFlag, state);
    ll_set_flag(RFLAG_CF, overflowFlag, state);
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
 * Set the flags for a bitwise operation. The flag cache will be invalidated.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param result The result of the operation
 * \param state The module state
 **/
void
ll_flags_set_bit(LLState* state, LLVMValueRef result, LLVMValueRef lhs, LLVMValueRef rhs)
{
    LLVMTypeRef i1 = LLVMInt1TypeInContext(state->context);

    ll_set_flag(RFLAG_AF, LLVMGetUndef(i1), state);
    ll_set_flag(RFLAG_CF, LLVMConstInt(i1, 0, false), state);
    ll_set_flag(RFLAG_OF, LLVMConstInt(i1, 0, false), state);

    ll_flags_set_zf(result, state);
    ll_flags_set_sf(result, state);
    ll_flags_set_pf(result, state);

    (void) lhs;
    (void) rhs;
}

void
ll_flags_set_shl(LLState* state, LLVMValueRef result, LLVMValueRef lhs, LLVMValueRef rhs)
{
    // TODO
    ll_flags_invalidate(state);
    ll_flags_set_zf(result, state);
    ll_flags_set_sf(result, state);
    ll_flags_set_pf(result, state);

    (void) lhs;
    (void) rhs;
}

void
ll_flags_set_shr(LLState* state, LLVMValueRef result, LLVMValueRef lhs, LLVMValueRef rhs)
{
    // TODO
    ll_flags_invalidate(state);
    ll_flags_set_zf(result, state);
    ll_flags_set_sf(result, state);
    ll_flags_set_pf(result, state);

    (void) lhs;
    (void) rhs;
}

void
ll_flags_set_sar(LLState* state, LLVMValueRef result, LLVMValueRef lhs, LLVMValueRef rhs)
{
    // TODO
    ll_flags_invalidate(state);
    ll_flags_set_zf(result, state);
    ll_flags_set_sf(result, state);
    ll_flags_set_pf(result, state);

    (void) lhs;
    (void) rhs;
}

/**
 * Invalidate the flags and the flag cache.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param state The module state
 **/
void
ll_flags_invalidate(LLState* state)
{
    LLVMTypeRef i1 = LLVMInt1TypeInContext(state->context);

    ll_set_flag(RFLAG_AF, LLVMGetUndef(i1), state);
    ll_set_flag(RFLAG_CF, LLVMGetUndef(i1), state);
    ll_set_flag(RFLAG_OF, LLVMGetUndef(i1), state);
    ll_set_flag(RFLAG_SF, LLVMGetUndef(i1), state);
    ll_set_flag(RFLAG_ZF, LLVMGetUndef(i1), state);
    ll_set_flag(RFLAG_PF, LLVMGetUndef(i1), state);
}

/**
 * @}
 **/
