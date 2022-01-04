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

#include "callconv.h"
#include "config.h"
#include "function.h"
#include "instr.h"
#include "transforms.h"

#include <llvm-c/Core.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>

#include <cstdbool>
#include <cstdint>
#include <cstring>

namespace {
static rellume::LLConfig* unwrap(LLConfig* fn) {
    return reinterpret_cast<rellume::LLConfig*>(fn);
}
static rellume::Function* unwrap(LLFunc* fn) {
    return reinterpret_cast<rellume::Function*>(fn);
}
} // namespace

LLConfig* ll_config_new(void) {
    return reinterpret_cast<LLConfig*>(new rellume::LLConfig());
}
void ll_config_free(LLConfig* cfg) { delete unwrap(cfg); }
void ll_config_set_hhvm(LLConfig* cfg, bool hhvm) {
    rellume::LLConfig* rlcfg = unwrap(cfg);
    rlcfg->callconv = hhvm ? rellume::CallConv::X86_64_HHVM : rellume::CallConv::X86_64_SPTR;
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
void ll_config_set_position_independent_code(LLConfig* cfg, bool enable) {
    unwrap(cfg)->position_independent_code = enable;
}
void ll_config_set_global_base(LLConfig* cfg, uintptr_t base,
                               LLVMValueRef value) {
    unwrap(cfg)->global_base_addr = base;
    unwrap(cfg)->global_base_value = llvm::unwrap(value);
}
void ll_config_set_pc_base(LLConfig* cfg, uintptr_t base, LLVMValueRef value) {
    unwrap(cfg)->pc_base_addr = base;
    unwrap(cfg)->pc_base_value = llvm::unwrap(value);
}
void ll_config_set_instr_impl(LLConfig* cfg, unsigned type,
                              LLVMValueRef value) {
    unwrap(cfg)->instr_overrides[type] = llvm::unwrap<llvm::Function>(value);
}
void ll_config_set_tail_func(LLConfig* cfg, LLVMValueRef value) {
    llvm::Value* uw_value = llvm::unwrap(value);
    unwrap(cfg)->tail_function = llvm::cast_or_null<llvm::Function>(uw_value);
}
void ll_config_set_call_func(LLConfig* cfg, LLVMValueRef value) {
    llvm::Value* uw_value = llvm::unwrap(value);
    unwrap(cfg)->call_function = llvm::cast_or_null<llvm::Function>(uw_value);
}
void ll_config_set_syscall_impl(LLConfig* cfg, LLVMValueRef value) {
    unwrap(cfg)->syscall_implementation = llvm::unwrap<llvm::Function>(value);
}
void ll_config_set_cpuinfo_func(LLConfig* cfg, LLVMValueRef value) {
    llvm::Value* uw_value = llvm::unwrap(value);
    unwrap(cfg)->cpuinfo_function = llvm::cast_or_null<llvm::Function>(uw_value);
}
void ll_config_set_instr_marker(LLConfig* cfg, LLVMValueRef value) {
    if (value)
        unwrap(cfg)->instr_marker = llvm::unwrap<llvm::Function>(value);
    else
        unwrap(cfg)->instr_marker = nullptr;
}
void ll_config_set_call_ret_clobber_flags(LLConfig* cfg, bool enable) {
    unwrap(cfg)->call_ret_clobber_flags = enable;
}
void ll_config_set_use_native_segment_base(LLConfig* cfg, bool enable) {
    unwrap(cfg)->use_native_segment_base = enable;
}
void ll_config_enable_full_facets(LLConfig* cfg, bool enable) {
    unwrap(cfg)->full_facets = enable;
}
bool ll_config_set_architecture(LLConfig* cfg, const char *s) {
    if (!strcmp(s, "x86-64") || !strcmp(s, "x86_64")) {
#ifdef RELLUME_WITH_X86_64
        unwrap(cfg)->arch = rellume::Arch::X86_64;
        unwrap(cfg)->callconv = rellume::CallConv::X86_64_SPTR;
        return true;
#endif // RELLUME_WITH_X86_64
    } else if (!strcmp(s, "rv64")) {
#ifdef RELLUME_WITH_RV64
        unwrap(cfg)->arch = rellume::Arch::RV64;
        unwrap(cfg)->callconv = rellume::CallConv::RV64_SPTR;
        return true;
#endif // RELLUME_WITH_RV64
    }
    return false;
}

// Rellume Function API

LLFunc* ll_func_new(LLVMModuleRef mod, LLConfig* cfg) {
    return reinterpret_cast<LLFunc*>(
        new rellume::Function(llvm::unwrap(mod), unwrap(cfg)));
}

LLVMValueRef ll_func_lift(LLFunc* fn) { return llvm::wrap(unwrap(fn)->Lift()); }
void ll_func_dispose(LLFunc* fn) { delete unwrap(fn); }

int ll_func_add_instr(LLFunc* func, uintptr_t block_addr, uintptr_t addr,
                      size_t bufsz, const uint8_t* buf) {
    return unwrap(func)->AddInst(block_addr, addr, bufsz, buf);
}

static int ll_func_decode(LLFunc* func, uintptr_t addr,
                          rellume::Function::DecodeStop stop,
                          RellumeMemAccessCb mem_acc, void* user_arg) {
    rellume::Function::MemReader rl_memacc;
    if (mem_acc) {
        rl_memacc = [=](uintptr_t maddr, uint8_t* buf, size_t buf_sz) {
            return mem_acc(maddr, buf, buf_sz, user_arg);
        };
    } else {
        rl_memacc = [](uintptr_t mem_addr, uint8_t* buf, size_t buf_sz) {
            memcpy(buf, reinterpret_cast<uint8_t*>(mem_addr), buf_sz);
            return buf_sz;
        };
    }
    return unwrap(func)->Decode(addr, stop, rl_memacc);
}
int ll_func_decode_instr(LLFunc* func, uintptr_t addr,
                         RellumeMemAccessCb mem_acc, void* user_arg) {
    return ll_func_decode(func, addr, rellume::Function::DecodeStop::INSTR,
                          mem_acc, user_arg);
}
int ll_func_decode_block(LLFunc* func, uintptr_t addr,
                         RellumeMemAccessCb mem_acc, void* user_arg) {
    return ll_func_decode(func, addr, rellume::Function::DecodeStop::BASICBLOCK,
                          mem_acc, user_arg);
}
int ll_func_decode_cfg(LLFunc* func, uintptr_t addr, RellumeMemAccessCb mem_acc,
                       void* user_arg) {
    return ll_func_decode(func, addr, rellume::Function::DecodeStop::ALL,
                          mem_acc, user_arg);
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
