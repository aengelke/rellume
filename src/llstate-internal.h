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

#ifndef LL_STATE_H
#define LL_STATE_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <llvm-c/Core.h>
#include <llvm-c/ExecutionEngine.h>

#include <llregfile-internal.h>

#ifdef __cplusplus
extern "C" {
#endif

struct LLConfig {
    size_t stackSize;

    /**
     * \brief Whether overflow intrinsics should be used.
     **/
    bool enableOverflowIntrinsics;
    /**
     * \brief Whether unsafe floating-point optimizations may be applied.
     * Corresponds to -ffast-math.
     **/
    bool enableFastMath;
    /**
     * \brief Whether to force full loop unrolling on all loops
     **/
    bool enableFullLoopUnroll;

    /**
     * \brief The global offset base
     **/
    uintptr_t globalOffsetBase;
    /**
     * \brief The global variable used to access constant memory regions. Points
     * to globalOffsetBase.
     **/
    LLVMValueRef globalBase;
};

typedef struct LLConfig LLConfig;

/**
 * \brief The LLVM state of the back-end.
 **/
struct LLState {
    LLConfig cfg;

    /**
     * \brief The LLVM Context
     **/
    LLVMContextRef context;
    /**
     * \brief The LLVM Builder
     **/
    LLVMBuilderRef builder;

    /**
     * \brief The empty metadata node
     **/
    LLVMValueRef emptyMD;

    /**
     * \brief The current register file
     **/
    LLRegisterFile* regfile;
};

typedef struct LLState LLState;

#define ll_get_register(reg,facet,state) ll_regfile_get(state->regfile,facet,reg,state->builder)
#define ll_clear_register(reg,state) ll_regfile_clear(state->regfile,reg,state->context)
#define ll_set_register(reg,facet,value,clear,state) ll_regfile_set(state->regfile,facet,reg,value,clear,state->builder)
#define ll_get_flag(reg,state) ll_regfile_get_flag(state->regfile,reg)
#define ll_set_flag(reg,value,state) ll_regfile_set_flag(state->regfile,reg,value)
#define ll_get_flag_cache(state) ll_regfile_get_flag_cache(state->regfile)

#ifdef __cplusplus
}
#endif

#endif
