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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <llvm-c/Analysis.h>
#include <llvm-c/Core.h>
#include <llvm-c/Support.h>
#include <llvm-c/Transforms/IPO.h>
#include <llvm-c/Transforms/Scalar.h>
#include <llvm-c/Transforms/Vectorize.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>

#include <llfunc.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <lloperand-internal.h>
#include <llregfile-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLFunc Func2
 * \brief Representation of a function
 *
 * @{
 **/

struct LLFunc {
    LLVMValueRef llvm;
    LLState state;

    /**
     * \brief The basic block count
     **/
    size_t bbCount;
    /**
     * \brief The allocated size for basic blocks
     **/
    size_t bbsAllocated;
    /**
     * \brief Array of basics blocks belonging to this function
     **/
    LLBasicBlock** bbs;

    /**
     * \brief The initial basic block, which is the entry point
     **/
    LLBasicBlock* initialBB;
};

static
void
ll_func_create_entry(LLFunc* fn)
{
    LLState* state = &fn->state;

    LLVMTypeRef i1 = LLVMInt1TypeInContext(state->context);
    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
    LLVMTypeRef iVec = LLVMIntTypeInContext(state->context, LL_VECTOR_REGISTER_SIZE);

    size_t paramCount = LLVMCountParams(fn->llvm);

    llvm::Function* llvm_fn = llvm::unwrap<llvm::Function>(fn->llvm);
    llvm::BasicBlock* first_bb = llvm_fn->empty() ? nullptr : &llvm_fn->front();
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(*llvm::unwrap(state->context), "", llvm_fn, first_bb);
    LLBasicBlock* initialBB = ll_basic_block_new(llvm::wrap(llvm_bb), &fn->state);
    ll_basic_block_set_current(initialBB);

    // Iterate over the parameters to initialize the registers.
    LLVMValueRef params = LLVMGetFirstParam(fn->llvm);

    // Set all registers to undef first.
    for (int i = 0; i < LL_RI_GPMax; i++)
        ll_set_register(ll_reg(LL_RT_GP64, i), FACET_I64, LLVMGetUndef(i64), true, state);

    for (int i = 0; i < LL_RI_XMMMax; i++)
        ll_set_register(ll_reg(LL_RT_XMM, i), FACET_IVEC, LLVMGetUndef(iVec), true, state);

    for (int i = 0; i < RFLAG_Max; i++)
        ll_set_flag(i, LLVMGetUndef(i1), state);


    LLReg gpRegs[6] = {
        ll_reg(LL_RT_GP64, LL_RI_DI),
        ll_reg(LL_RT_GP64, LL_RI_SI),
        ll_reg(LL_RT_GP64, LL_RI_D),
        ll_reg(LL_RT_GP64, LL_RI_C),
        ll_reg(LL_RT_GP64, 8),
        ll_reg(LL_RT_GP64, 9),
    };
    int gpRegOffset = 0;
    int fpRegOffset = 0;
    for (size_t i = 0; i < paramCount; i++)
    {
        LLVMTypeKind paramTypeKind = LLVMGetTypeKind(LLVMTypeOf(params));
        LLInstrOp operand;

        if (paramTypeKind == LLVMPointerTypeKind)
        {
            LLVMValueRef intValue = LLVMBuildPtrToInt(state->builder, params, i64, "");
            operand = getRegOp(gpRegs[gpRegOffset]);
            ll_operand_store(OP_SI, ALIGN_MAXIMUM, &operand, REG_DEFAULT, intValue, state);
            gpRegOffset++;
        }
        else if (paramTypeKind == LLVMIntegerTypeKind)
        {
            operand = getRegOp(gpRegs[gpRegOffset]);
            ll_operand_store(OP_SI, ALIGN_MAXIMUM, &operand, REG_DEFAULT, params, state);
            gpRegOffset++;
        }
        else if (paramTypeKind == LLVMFloatTypeKind)
        {
            operand = getRegOp(ll_reg(LL_RT_XMM, fpRegOffset));
            ll_operand_store(OP_SF32, ALIGN_MAXIMUM, &operand, REG_ZERO_UPPER_SSE, params, state);
            fpRegOffset++;
        }
        else if (paramTypeKind == LLVMDoubleTypeKind)
        {
            operand = getRegOp(ll_reg(LL_RT_XMM, fpRegOffset));
            ll_operand_store(OP_SF64, ALIGN_MAXIMUM, &operand, REG_ZERO_UPPER_SSE, params, state);
            fpRegOffset++;
        }
        else
            warn_if_reached();

        params = LLVMGetNextParam(params);
    }

    // Setup virtual stack
    LLVMValueRef stackSize = LLVMConstInt(i64, state->cfg.stackSize, false);
    LLVMValueRef stack = LLVMBuildArrayAlloca(state->builder, i8, stackSize, "");
    LLVMValueRef sp = LLVMBuildGEP(state->builder, stack, &stackSize, 1, "");
    ll_set_register(ll_reg(LL_RT_GP64, LL_RI_SP), FACET_PTR, sp, true, state);

    LLVMSetAlignment(stack, 16);

    fn->initialBB = initialBB;
}

LLFunc*
ll_func(const char* name, LLVMTypeRef ty, LLVMModuleRef mod)
{
    LLFunc* fn = new LLFunc();
    fn->llvm = LLVMAddFunction(mod, name, ty);
    fn->bbCount = 0;
    fn->bbs = NULL;
    fn->bbsAllocated = 0;

    LLState* state = &fn->state;
    state->context = LLVMGetModuleContext(mod);
    state->builder = LLVMCreateBuilderInContext(state->context);

    state->cfg.globalBase = NULL;
    state->cfg.stackSize = 128; // FIXME
    state->cfg.enableOverflowIntrinsics = false;
    state->cfg.enableFastMath = false;
    state->cfg.enableFullLoopUnroll = false;

    return fn;
}

/**
 * Enable the usage of overflow intrinsics instead of bitwise operations when
 * setting the overflow flag. For dynamic values this leads to better code which
 * relies on the overflow flag again. However, immediate values are not folded
 * when they are guaranteed to overflow.
 *
 * This function must be called before the IR of the function is built.
 *
 * \author Alexis Engelke
 *
 * \param state The module state
 * \param enable Whether overflow intrinsics shall be used
 **/
void
ll_func_enable_overflow_intrinsics(LLFunc* fn, bool enable)
{
    fn->state.cfg.enableOverflowIntrinsics = enable;
}

/**
 * Enable unsafe floating-point optimizations, similar to -ffast-math.
 *
 * This function must be called before the IR of the function is built.
 *
 * \author Alexis Engelke
 *
 * \param state The module state
 * \param enable Whether unsafe floating-point optimizations may be performed
 **/
void
ll_func_enable_fast_math(LLFunc* fn, bool enable)
{
    fn->state.cfg.enableFastMath = enable;
}

/**
 * Force loop unrolling whenever possible.
 *
 * \author Alexis Engelke
 *
 * \param state The module state
 * \param enable Whether force loop unrolling
 **/
void
ll_func_enable_full_loop_unroll(LLFunc* fn, bool enable)
{
    fn->state.cfg.enableFullLoopUnroll = enable;
}

void
ll_func_set_global_base(LLFunc* fn, uintptr_t base, LLVMValueRef value)
{
    fn->state.cfg.globalOffsetBase = base;
    fn->state.cfg.globalBase = value;
}

/**
 * Dispose a function.
 *
 * \author Alexis Engelke
 *
 * \param fn The function
 **/
void
ll_func_dispose(LLFunc* fn)
{
    LLVMDisposeBuilder(fn->state.builder);

    if (fn->bbsAllocated != 0)
    {
        for (size_t i = 0; i < fn->bbCount; i++)
            ll_basic_block_dispose(fn->bbs[i]);

        free(fn->bbs);
    }

    free(fn);
}

/**
 * Dump the LLVM IR of the function.
 *
 * \author Alexis Engelke
 *
 * \param state The function
 **/
void
ll_func_dump(LLFunc* fn)
{
    char* value = LLVMPrintValueToString(fn->llvm);
    puts(value);
    LLVMDisposeMessage(value);
}

static void
ll_func_add_basic_block(LLFunc* function, LLBasicBlock* bb)
{
    if (function->bbsAllocated == 0)
    {
        function->bbs = (LLBasicBlock**) malloc(sizeof(LLBasicBlock*) * 10);
        function->bbsAllocated = 10;

        if (function->bbs == NULL)
            warn_if_reached();
    }
    else if (function->bbsAllocated == function->bbCount)
    {
        function->bbs = (LLBasicBlock**) realloc(function->bbs, sizeof(LLBasicBlock*) * function->bbsAllocated * 2);
        function->bbsAllocated *= 2;

        if (function->bbs == NULL)
            warn_if_reached();
    }

    function->bbs[function->bbCount] = bb;
    function->bbCount++;
}

LLBasicBlock*
ll_func_add_block(LLFunc* fn)
{
    LLVMBasicBlockRef llvmBB = LLVMAppendBasicBlockInContext(fn->state.context, fn->llvm, "");
    LLBasicBlock* bb = ll_basic_block_new(llvmBB, &fn->state);
    ll_basic_block_add_phis(bb);
    ll_func_add_basic_block(fn, bb);
    return bb;
}

LLVMValueRef
ll_func_lift(LLFunc* fn)
{
    size_t bbCount = fn->bbCount;

    if (fn->bbCount == 0)
        return NULL;

    ll_func_create_entry(fn);

    // The initial basic block falls through to the first lifted block.
    ll_basic_block_add_branches(fn->initialBB, NULL, fn->bbs[0]);
    ll_basic_block_terminate(fn->initialBB);

    for (size_t i = 0; i < bbCount; i++)
    {
        ll_basic_block_terminate(fn->bbs[i]);
        ll_basic_block_fill_phis(fn->bbs[i]);
    }

    bool error = LLVMVerifyFunction(fn->llvm, LLVMPrintMessageAction);
    if (error)
        return NULL;

    // Run some optimization passes to remove most of the bloat
    LLVMPassManagerRef pm = LLVMCreateFunctionPassManagerForModule(LLVMGetGlobalParent(fn->llvm));
    LLVMInitializeFunctionPassManager(pm);

    LLVMAddEarlyCSEPass(pm);
    LLVMAddGVNPass(pm);
    LLVMAddCFGSimplificationPass(pm);
    LLVMAddInstructionCombiningPass(pm);
    LLVMAddAggressiveDCEPass(pm);

    LLVMRunFunctionPassManager(pm, fn->llvm);

    LLVMFinalizeFunctionPassManager(pm);
    LLVMDisposePassManager(pm);

    return fn->llvm;
}

/**
 * @}
 **/
