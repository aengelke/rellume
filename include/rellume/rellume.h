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

#ifndef RELLUME_RELLUME_H
#define RELLUME_RELLUME_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <llvm-c/Core.h>

#include "rellume/instr.h"

#ifdef __cplusplus
extern "C" {
#endif

struct LLBasicBlock;

typedef struct LLBasicBlock LLBasicBlock;

void ll_basic_block_add_inst(LLBasicBlock*, LLInstr*);
void ll_basic_block_add_branches(LLBasicBlock*, LLBasicBlock*, LLBasicBlock*);


struct LLFunc;

typedef struct LLFunc LLFunc;

LLFunc* ll_func(LLVMModuleRef mod);

void ll_func_enable_overflow_intrinsics(LLFunc* fn, bool enable);
void ll_func_enable_fast_math(LLFunc* fn, bool enable);
void ll_func_set_stack_size(LLFunc* fn, size_t size);
void ll_func_set_global_base(LLFunc* fn, uintptr_t base, LLVMValueRef value);

LLBasicBlock* ll_func_add_block(LLFunc* fn);
LLVMValueRef ll_func_lift(LLFunc* fn);
void ll_func_dispose(LLFunc*);

LLVMValueRef ll_func_wrap_sysv(LLVMValueRef llvm_fn, LLVMTypeRef ty, LLVMModuleRef mod);

int ll_func_decode(LLFunc* func, uintptr_t addr);

#ifdef __cplusplus
}
#endif

#endif
