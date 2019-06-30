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
#include <llvm/IR/IRBuilder.h>
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

    /// Whether to use pointer for 64-bit reg-reg compares instead of using
    /// ptrtoint.
    bool prefer_pointer_cmp;

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
class LLState {
public:
    LLState(llvm::LLVMContext& ctx) : context(llvm::wrap(&ctx)), irb(ctx) {
        builder = llvm::wrap(&irb);
    }

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
     * \brief The current register file
     **/
    LLRegisterFile* regfile;

    llvm::IRBuilder<> irb;


    llvm::Value* GetReg(LLReg reg, RegisterFacet facet) {
        return llvm::unwrap(ll_regfile_get(regfile, facet, reg, llvm::wrap(&irb)));
    }
    void SetReg(LLReg reg, RegisterFacet facet, llvm::Value* value, bool clear = true) {
        ll_regfile_set(regfile, facet, reg, llvm::wrap(value), clear, llvm::wrap(&irb));
    }
    llvm::Value* GetFlag(int flag) {
        return llvm::unwrap(ll_regfile_get_flag(regfile, flag));
    }
    void SetFlag(int flag, llvm::Value* value) {
        ll_regfile_set_flag(regfile, flag, llvm::wrap(value), llvm::wrap(&(irb.getContext())));
    }
};

#define ll_get_register(reg,facet,state) llvm::wrap((state)->GetReg(reg, facet))
#define ll_set_register(reg,facet,value,clear,state) (state)->SetReg(reg, facet, llvm::unwrap(value), clear)
#define ll_get_flag(reg,state) llvm::wrap((state)->GetFlag(reg))
#define ll_set_flag(reg,value,state) (state)->SetFlag(reg, llvm::unwrap(value))
#define ll_clear_register(reg,state) ll_regfile_clear(state->regfile,reg,state->context)
#define ll_get_flag_cache(state) ll_regfile_get_flag_cache(state->regfile)

#ifdef __cplusplus
}
#endif

#endif
