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

#include <cstdbool>
#include <cstdint>


namespace llvm {
class Value;
}

namespace rellume {

struct LLConfig {
    /**
     * \brief Whether overflow intrinsics should be used.
     **/
    bool enableOverflowIntrinsics;
    /**
     * \brief Whether unsafe floating-point optimizations may be applied.
     * Corresponds to -ffast-math.
     **/
    bool enableFastMath;

    /// Whether to use pointer for 64-bit reg-reg compares instead of using
    /// ptrtoint.
    bool prefer_pointer_cmp;

    /// Whether to verify the IR after lifting.
    bool verify_ir;
    /// Whether to optimize the IR after lifting.
    bool optimize_ir;

    /**
     * \brief The global offset base
     **/
    uintptr_t global_base_addr;
    /**
     * \brief The global variable used to access constant memory regions. Points
     * to globalOffsetBase.
     **/
    llvm::Value* global_base_value;
};

typedef struct LLConfig LLConfig;

} // namespace

#endif
