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

#include "rellume/instr.h"
#include "rellume/rellume.h"

#include "callconv.h"
#include "config.h"
#include "function.h"
#include "instr.h"
#include "transforms.h"
#include <fadec.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>
#include <llvm-c/Core.h>
#include <cstdbool>
#include <cstdint>


namespace {
static rellume::LLConfig* unwrap(LLConfig* fn) {
    return reinterpret_cast<rellume::LLConfig*>(fn);
}
static rellume::Function* unwrap(LLFunc* fn) {
    return reinterpret_cast<rellume::Function*>(fn);
}
}

LLConfig* ll_config_new(void) {
    return reinterpret_cast<LLConfig*>(new rellume::LLConfig());
}
void ll_config_free(LLConfig* cfg) {
    delete unwrap(cfg);
}
void ll_config_set_hhvm(LLConfig* cfg, bool hhvm) {
    unwrap(cfg)->callconv = hhvm ? rellume::CallConv::HHVM : rellume::CallConv::SPTR;
}
void ll_config_set_sptr_addrspace(LLConfig* cfg, unsigned addrspace) {
    unwrap(cfg)->sptr_addrspace = addrspace;
}
void ll_config_enable_overflow_intrinsics(LLConfig* cfg, bool enable) {
    unwrap(cfg)->enableOverflowIntrinsics = enable;
}
void ll_config_enable_fast_math(LLConfig* cfg, bool enable) {
    unwrap(cfg)->enableFastMath = enable;
}
void ll_config_enable_verify_ir(LLConfig* cfg, bool enable) {
    unwrap(cfg)->verify_ir = enable;
}
void ll_config_set_global_base(LLConfig* cfg, uintptr_t base, LLVMValueRef value) {
    unwrap(cfg)->global_base_addr = base;
    unwrap(cfg)->global_base_value = llvm::unwrap(value);
}
void ll_config_set_instr_impl(LLConfig* cfg, FdInstrType type, LLVMValueRef value) {
    unwrap(cfg)->instr_overrides[type] = llvm::unwrap<llvm::Function>(value);
}
void ll_config_set_call_ret_clobber_flags(LLConfig* cfg, bool enable) {
    unwrap(cfg)->call_ret_clobber_flags = enable;
}
void ll_config_set_use_native_segment_base(LLConfig* cfg, bool enable) {
    unwrap(cfg)->use_native_segment_base = enable;
}


LLFunc* ll_func_new(LLVMModuleRef mod, LLConfig* cfg) {
    return reinterpret_cast<LLFunc*>(new rellume::Function(llvm::unwrap(mod), unwrap(cfg)));
}

void ll_func_add_inst(LLFunc* fn, uint64_t block_addr, FdInstr* instr) {
    unwrap(fn)->AddInst(block_addr, *static_cast<const rellume::Instr*>(instr));
}
LLVMValueRef ll_func_lift(LLFunc* fn) {
    return llvm::wrap(unwrap(fn)->Lift());
}
void ll_func_dispose(LLFunc* fn) {
    delete unwrap(fn);
}

int ll_func_decode(LLFunc* func, uintptr_t addr) {
    return unwrap(func)->Decode(addr, rellume::Function::DecodeStop::ALL);
}
int ll_func_decode2(LLFunc* func, uintptr_t addr, RellumeMemAccessCb mem_acc,
                    void* user_arg) {
    return ll_func_decode3(func, addr, RELLUME_DECODE_ALL, mem_acc, user_arg);
}
int ll_func_decode3(LLFunc* func, uintptr_t addr, LLDecodeStop stop,
                    RellumeMemAccessCb mem_acc, void* user_arg) {
    auto memacc_l = [=](uintptr_t maddr, uint8_t* buf, size_t buf_sz) {
        return mem_acc(maddr, buf, buf_sz, user_arg);
    };
    auto decode_stop = static_cast<rellume::Function::DecodeStop>(stop);
    return unwrap(func)->Decode(addr, decode_stop, memacc_l);
}

void ll_func_fast_opt(LLVMValueRef llvm_fn) {
    rellume::FastOpt(llvm::unwrap<llvm::Function>(llvm_fn));
}
LLVMValueRef ll_func_wrap_sysv(LLVMValueRef fn, LLVMTypeRef ty,
                               LLVMModuleRef mod, size_t stack_sz) {
    return llvm::wrap(rellume::WrapSysVAbi(llvm::unwrap<llvm::Function>(fn),
                                           llvm::unwrap<llvm::FunctionType>(ty),
                                           stack_sz));
}
