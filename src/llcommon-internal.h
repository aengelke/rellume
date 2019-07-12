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

#ifndef LL_COMMON_H
#define LL_COMMON_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <llvm/IR/Value.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief Emit a warning and jump into a debugger
 **/
#define warn_if_reached() do { printf("!WARN %s: Code should not be reached.\n", __func__); __asm__("int3"); } while (0)

/**
 * \brief The size of a vector
 **/
#define LL_VECTOR_REGISTER_SIZE 128

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

#ifdef __cplusplus
}
#endif

#endif
