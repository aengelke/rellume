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
#include <stddef.h>
#include <stdint.h>

#include <llvm-c/Core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RELLUME_API __attribute__((visibility("default")))
#define RELLUME_DEPRECATED __attribute__((deprecated))


typedef struct LLConfig LLConfig;

RELLUME_API LLConfig* ll_config_new(void);
RELLUME_API void ll_config_free(LLConfig*);

/// Set the architecture to x86-64 and use HHVM (true) or SPTR (false) calling
/// convention. Deprecated: avoid using the HHVM calling convention.
RELLUME_API void ll_config_set_hhvm(LLConfig*, bool) RELLUME_DEPRECATED;
RELLUME_API void ll_config_set_sptr_addrspace(LLConfig*, unsigned);
RELLUME_API void ll_config_enable_overflow_intrinsics(LLConfig*, bool);
RELLUME_API void ll_config_enable_fast_math(LLConfig*, bool);
RELLUME_API void ll_config_enable_verify_ir(LLConfig*, bool);
RELLUME_API void ll_config_set_position_independent_code(LLConfig*, bool);
RELLUME_API void ll_config_set_pc_base(LLConfig*, uintptr_t, LLVMValueRef);
RELLUME_API void ll_config_set_global_base(LLConfig*, uintptr_t, LLVMValueRef);
RELLUME_API void ll_config_set_instr_impl(LLConfig*, unsigned,
                                          LLVMValueRef) RELLUME_DEPRECATED;
RELLUME_API void ll_config_set_tail_func(LLConfig*, LLVMValueRef);
RELLUME_API void ll_config_set_call_func(LLConfig*, LLVMValueRef);
RELLUME_API void ll_config_set_syscall_impl(LLConfig*, LLVMValueRef);
RELLUME_API void ll_config_set_cpuinfo_func(LLConfig*, LLVMValueRef);
RELLUME_API void ll_config_set_instr_marker(LLConfig*, LLVMValueRef);
RELLUME_API void ll_config_set_call_ret_clobber_flags(LLConfig*, bool);
RELLUME_API void ll_config_set_use_native_segment_base(LLConfig*, bool);
RELLUME_API void ll_config_enable_full_facets(LLConfig*, bool);

/// Sets the architecture. Currently the only valid options is "x86_64", which
/// is also default, and "rv64". Return true, if the architecture is supported.
/// If no architecture is specified explicitly and the lifter is configured
/// without support for x86-64, th behavior of ll_func_new is undefined.
/// For backwards compatibility, also "x86-64" is accepted as valid option.
RELLUME_API bool ll_config_set_architecture(LLConfig*, const char*);

typedef struct LLFunc LLFunc;

RELLUME_API LLFunc* ll_func_new(LLVMModuleRef mod, LLConfig*);

RELLUME_API LLVMValueRef ll_func_lift(LLFunc* fn);
RELLUME_API void ll_func_dispose(LLFunc*);

RELLUME_API int ll_func_add_instr(LLFunc* func, uintptr_t block_addr,
                                  uintptr_t addr, size_t bufsz,
                                  const uint8_t* buf);
typedef size_t(* RellumeMemAccessCb)(size_t, uint8_t*, size_t, void*);
RELLUME_API int ll_func_decode_instr(LLFunc* func, uintptr_t addr,
                                     RellumeMemAccessCb cb, void* user_arg);
RELLUME_API int ll_func_decode_block(LLFunc* func, uintptr_t addr,
                                     RellumeMemAccessCb cb, void* user_arg);
RELLUME_API int ll_func_decode_cfg(LLFunc* func, uintptr_t addr,
                                   RellumeMemAccessCb cb, void* user_arg);

#ifdef __cplusplus
}
#endif

#endif
