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

#include "rellume/rellume.h"

#include "function.h"
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>
#include <llvm-c/Core.h>
#include <cstdbool>
#include <cstdint>


namespace {
static rellume::Function* unwrap(LLFunc* fn) {
    return reinterpret_cast<rellume::Function*>(fn);
}
}

LLFunc* ll_func(LLVMModuleRef mod) {
    return reinterpret_cast<LLFunc*>(new rellume::Function(llvm::unwrap(mod)));
}
void ll_func_enable_overflow_intrinsics(LLFunc* fn, bool enable) {
    unwrap(fn)->EnableOverflowIntrinsics(enable);
}
void ll_func_enable_fast_math(LLFunc* fn, bool enable) {
    unwrap(fn)->EnableFastMath(enable);
}
void ll_func_set_global_base(LLFunc* fn, uintptr_t base, LLVMValueRef value) {
    unwrap(fn)->SetGlobalBase(base, llvm::unwrap(value));
}

void ll_func_add_inst(LLFunc* fn, uint64_t block_addr, LLInstr* instr) {
    unwrap(fn)->AddInst(block_addr, *instr);
}
LLVMValueRef ll_func_lift(LLFunc* fn) {
    return llvm::wrap(unwrap(fn)->Lift());
}
void ll_func_dispose(LLFunc* fn) {
    delete unwrap(fn);
}

int ll_func_decode(LLFunc* func, uintptr_t addr) {
    return unwrap(func)->Decode(addr);
}
int ll_func_decode2(LLFunc* func, uintptr_t addr, RellumeMemAccessCb mem_acc,
                    void* user_arg) {
    return unwrap(func)->Decode(addr, [=](uintptr_t addr, uint8_t* buf, size_t buf_sz) {
        return mem_acc(addr, buf, buf_sz, user_arg);
    });
}
