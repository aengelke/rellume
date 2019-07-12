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
#include <llvm-c/Core.h>

#include <rellume/rellume.h>

#include <llbasicblock-internal.h>
#include <llfunction-internal.h>


LLFunc* ll_func(LLVMModuleRef mod) {
    return reinterpret_cast<LLFunc*>(new rellume::Function(llvm::unwrap(mod)));
}
void ll_func_enable_overflow_intrinsics(LLFunc* fn, bool enable) {
    reinterpret_cast<rellume::Function*>(fn)->EnableOverflowIntrinsics(enable);
}
void ll_func_enable_fast_math(LLFunc* fn, bool enable) {
    reinterpret_cast<rellume::Function*>(fn)->EnableFastMath(enable);
}
void ll_func_set_global_base(LLFunc* fn, uintptr_t base, LLVMValueRef value) {
    reinterpret_cast<rellume::Function*>(fn)->SetGlobalBase(base, llvm::unwrap(value));
}

void ll_func_add_inst(LLFunc* fn, uint64_t block_addr, LLInstr* instr) {
    return reinterpret_cast<rellume::Function*>(fn)->AddInst(block_addr, *instr);
}
LLVMValueRef ll_func_lift(LLFunc* fn) {
    return llvm::wrap(reinterpret_cast<rellume::Function*>(fn)->Lift());
}
void ll_func_dispose(LLFunc* fn) {
    delete reinterpret_cast<rellume::Function*>(fn);
}

int ll_func_decode(LLFunc* func, uintptr_t addr) {
    return reinterpret_cast<rellume::Function*>(func)->Decode(addr);
}
