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

#ifndef LL_SUPPORT_H
#define LL_SUPPORT_H

#include <llvm-c/Core.h>
#include <llvm-c/Transforms/PassManagerBuilder.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup LLSupport
 **/
enum LLSupportIntrinsics {
    LL_INTRINSIC_DO_NOTHING,
    LL_INTRINSIC_CTPOP,
    LL_INTRINSIC_SADD_WITH_OVERFLOW,
    LL_INTRINSIC_SSUB_WITH_OVERFLOW,
    LL_INTRINSIC_SMUL_WITH_OVERFLOW,
};

typedef enum LLSupportIntrinsics LLSupportIntrinsics;

LLVMValueRef ll_support_get_intrinsic(LLVMModuleRef, LLSupportIntrinsics, LLVMTypeRef*, unsigned);
LLVMAttributeRef ll_support_get_enum_attr(LLVMContextRef, const char*);
void ll_support_enable_fast_math(LLVMValueRef);
LLVMBool ll_support_is_constant_int(LLVMValueRef);

#ifdef __cplusplus
}
#endif

#endif
