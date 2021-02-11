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

#ifndef RELLUME_CONFIG_H
#define RELLUME_CONFIG_H

#include "arch.h"
#include "callconv.h"
#include <cstdbool>
#include <cstdint>
#include <unordered_map>


namespace llvm {
class Function;
class Value;
}

namespace rellume {

struct LLConfig {
    /// Enable the usage of overflow intrinsics instead of bitwise operations
    /// when setting the overflow flag. For dynamic values this leads to better
    /// code which relies on the overflow flag again. However, immediate values
    /// are not folded when they are guaranteed to (not) overflow.
    bool enableOverflowIntrinsics = false;
    /// Unsafe floating-point optimizations, corresponds to -ffast-math.
    bool enableFastMath = false;
    /// Make CALL and RET clobber all status flags.
    bool call_ret_clobber_flags = false;
    /// Use native registers FS and GS for segmented memory access
    bool use_native_segment_base = false;
    /// Generate PHI nodes for all facets (instead of only one per register)
    bool full_facets = false;
    /// Verify the IR after lifting.
    bool verify_ir = false;
    /// Don't use absolute instruction addresses to set RIP. The actual RIP is
    /// supplied as in the RIP register field of the CPU struct.
    bool position_independent_code = false;

    /// Instruction Set Architecture of the code to lift.
    Arch arch = Arch::DEFAULT;

    /// Optimize generated IR for the HHVM calling convention.
    CallConv callconv = CallConv::X86_64_SPTR;
    /// Address space for CPU struct pointer parameter
    unsigned sptr_addrspace = 0;

    /// The global offset base
    uintptr_t global_base_addr = 0;
    /// The global variable used to access constant memory regions. Points to
    /// globalOffsetBase.
    llvm::Value* global_base_value = nullptr;

    /// Overridden implementations for specific instruction. The function must
    /// take a pointer to the CPU state as a single argument.
    std::unordered_map<uint32_t, llvm::Function*> instr_overrides;

    /// If non-null, this function is called as a tail-call instead of
    /// returning.
    llvm::Function* tail_function = nullptr;

    /// If non-null, this function is called on a call instruction. Decoding
    /// continues after the CALL instruction as if the instruction did not
    /// modify control flow (albeit checking that the RIP matches). A return
    /// will be lifted as a ret instruction, regardless of the value of
    /// tail_function.
    llvm::Function* call_function = nullptr;

    /// Implementation of syscall semantics. If not specified, a syscall behaves
    /// as a no-op. The function must take a pointer to the CPU state as a
    /// single argument.
    llvm::Function* syscall_implementation = nullptr;

    /// Function which is called before the instruction code is lifted. The
    /// function takes the value of RIP (which points at the end of the
    /// instruction) and a metadata containing an MDString with the FdInstr.
    llvm::Function* instr_marker = nullptr;
};

} // namespace

#endif
