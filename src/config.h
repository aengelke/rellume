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

#include "callconv.h"
#include "rellume/instr.h"
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
    /// Generate additional pointer facets using getelementptr for ADD/SUB/LEA.
    bool use_gep_ptr_arithmetic = false;
    /// Use pointer types for 64-bit reg-reg compares instead of ptrtoint.
    bool prefer_pointer_cmp = false;
    /// Make CALL and RET clobber all status flags.
    bool call_ret_clobber_flags = false;
    /// Verify the IR after lifting.
    bool verify_ir = false;

    /// Optimize generated IR for the HHVM calling convention.
    CallConv callconv = CallConv::SPTR;

    /// The global offset base
    uintptr_t global_base_addr = 0;
    /// The global variable used to access constant memory regions. Points to
    /// globalOffsetBase.
    llvm::Value* global_base_value = nullptr;

    /// Overridden implementations for specific instruction. The function must
    /// take a pointer to the CPU state as a single argument.
    std::unordered_map<LLInstrType, llvm::Function*> instr_overrides;
};

} // namespace

#endif
