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

#include <llvm/ADT/ArrayRef.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>

#include <llsupport-internal.h>

/**
 * \defgroup LLSupport Support
 * \brief Support functions for the LLVM API
 *
 * @{
 **/

/**
 * Get the declaration of an LLVM intrinsic with the given types.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param module The LLVM module
 * \param intrinsic The intrinsic
 * \param types The types of the instantiation, or NULL
 * \param typeCount The number of types
 * \returns A declaration of the requested intrinsic
 **/
extern "C"
LLVMValueRef
ll_support_get_intrinsic(LLVMBuilderRef builder, LLSupportIntrinsics intrinsic, LLVMTypeRef* types, unsigned typeCount)
{
    llvm::Module* module = llvm::unwrap(builder)->GetInsertBlock()->getModule();
    llvm::ArrayRef<llvm::Type*> Tys(llvm::unwrap(types), typeCount);
    llvm::Intrinsic::ID intrinsicId;

    switch (intrinsic)
    {
        case LL_INTRINSIC_CTPOP: intrinsicId = llvm::Intrinsic::ctpop; break;
        case LL_INTRINSIC_SADD_WITH_OVERFLOW: intrinsicId = llvm::Intrinsic::sadd_with_overflow; break;
        case LL_INTRINSIC_SSUB_WITH_OVERFLOW: intrinsicId = llvm::Intrinsic::ssub_with_overflow; break;
        case LL_INTRINSIC_SMUL_WITH_OVERFLOW: intrinsicId = llvm::Intrinsic::smul_with_overflow; break;
        default: intrinsicId = llvm::Intrinsic::not_intrinsic; break;
    }

    return llvm::wrap(llvm::Intrinsic::getDeclaration(module, intrinsicId, Tys));
}

/**
 * Enable unsafe algebra on the result of a floating-point instruction.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param value The result of a supported floating-point instruction
 **/
extern "C"
void
ll_support_enable_fast_math(LLVMValueRef value)
{
#if LL_LLVM_MAJOR >= 6
    llvm::unwrap<llvm::Instruction>(value)->setFast(true);
#else
    llvm::unwrap<llvm::Instruction>(value)->setHasUnsafeAlgebra(true);
#endif
}

extern "C"
LLVMTypeRef
ll_support_builder_return_type(LLVMBuilderRef builder)
{
    return llvm::wrap(llvm::unwrap(builder)->getCurrentFunctionReturnType());
}

/**
 * @}
 **/
