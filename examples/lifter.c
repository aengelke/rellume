/**
 * This file is part of Rellume.
 *
 * (c) 2019, Alexis Engelke <alexis.engelke@googlemail.com>
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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <llvm-c/Core.h>

#include <rellume/decoder.h>
#include <rellume/func.h>


static
float
sample_func(size_t n, float* arr)
{
    float res = 0;
    for (size_t i = 0; i < n; i++)
        res += arr[i];
    return res;
}

int
main(void)
{
    // Create LLVM module
    LLVMModuleRef mod = LLVMModuleCreateWithName("lifter");

    // Construct type of function in LLVM IR
    LLVMTypeRef fnty_args[2] = {LLVMInt64Type(), LLVMPointerType(LLVMFloatType(), 0)};
    LLVMTypeRef fnty = LLVMFunctionType(LLVMFloatType(), fnty_args, 2, false);

    // Create function for lifting
    LLFunc* fn = ll_func("sample_func", mod);
    // Lift the whole function by following all direct jumps
    ll_func_decode(fn, (uintptr_t) sample_func);
    LLVMValueRef llvm_fn = ll_func_lift(fn);
    llvm_fn = ll_func_wrap_sysv(llvm_fn, fnty, mod);

    LLVMDumpValue(llvm_fn);

    return 0;
}

/*

Sample output:

define float @sample_func(i64, float*) {
  %3 = icmp eq i64 %0, 0, !asm.flag.zf !0
  br i1 %3, label %7, label %4

; <label>:4:                                      ; preds = %2
  %5 = bitcast float* %1 to i8*
  %6 = getelementptr float, float* %1, i64 %0
  br label %8

; <label>:7:                                      ; preds = %8, %2
  %merge = phi float [ 0.000000e+00, %2 ], [ %13, %8 ]
  ret float %merge

; <label>:8:                                      ; preds = %8, %4
  %9 = phi i8* [ %5, %4 ], [ %14, %8 ], !asm.reg.rsi !0
  %10 = phi float [ 0.000000e+00, %4 ], [ %13, %8 ], !asm.reg.xmm0 !0
  %11 = bitcast i8* %9 to float*
  %12 = load float, float* %11, align 4
  %13 = fadd float %10, %12
  %14 = getelementptr i8, i8* %9, i64 4, !asm.reg.rsi !0
  %15 = bitcast float* %6 to i8*
  %16 = icmp eq i8* %14, %15
  br i1 %16, label %7, label %8
}

*/

