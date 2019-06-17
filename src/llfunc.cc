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
#include <llvm/ADT/SmallVector.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Type.h>
#include <llvm/Transforms/Utils/Cloning.h>

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

    llvm::Function* llvm_fn = llvm::unwrap<llvm::Function>(fn->llvm);
    llvm::BasicBlock* first_bb = llvm_fn->empty() ? nullptr : &llvm_fn->front();
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(*llvm::unwrap(state->context), "", llvm_fn, first_bb);
    LLBasicBlock* initialBB = ll_basic_block_new(llvm::wrap(llvm_bb), &fn->state);
    ll_basic_block_set_current(initialBB);

    llvm::Value* param = llvm_fn->arg_begin();
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);

    llvm::Value* regs = builder->CreateLoad(param);
    for (unsigned i = 0; i < LL_RI_GPMax; i++)
        ll_set_register(ll_reg(LL_RT_GP64, i), FACET_I64, llvm::wrap(builder->CreateExtractValue(regs, {1, i})), true, state);

    for (unsigned i = 0; i < LL_RI_XMMMax; i++)
        ll_set_register(ll_reg(LL_RT_XMM, i), FACET_IVEC, llvm::wrap(builder->CreateExtractValue(regs, {3, i})), true, state);

    for (unsigned i = 0; i < RFLAG_Max; i++)
        ll_set_flag(i, llvm::wrap(builder->CreateExtractValue(regs, {2, i})), state);

    // Setup virtual stack
    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
    LLVMValueRef stackSize = LLVMConstInt(i64, state->cfg.stackSize, false);
    LLVMValueRef stack = LLVMBuildArrayAlloca(state->builder, i8, stackSize, "");
    LLVMValueRef sp = LLVMBuildGEP(state->builder, stack, &stackSize, 1, "");
    ll_set_register(ll_reg(LL_RT_GP64, LL_RI_SP), FACET_PTR, sp, true, state);

    LLVMSetAlignment(stack, 16);

    fn->initialBB = initialBB;
}

LLFunc*
ll_func(const char* name, LLVMModuleRef mod)
{
    LLFunc* fn = new LLFunc();

    LLState* state = &fn->state;
    state->context = LLVMGetModuleContext(mod);
    state->builder = LLVMCreateBuilderInContext(state->context);

    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    llvm::SmallVector<llvm::Type*, LL_RI_GPMax+LL_RI_XMMMax+RFLAG_Max> cpu_types;
    cpu_types.push_back(builder->getInt64Ty()); // instruction pointer
    cpu_types.push_back(llvm::ArrayType::get(builder->getInt64Ty(), 16));
    cpu_types.push_back(llvm::ArrayType::get(builder->getInt1Ty(), 6));
    cpu_types.push_back(llvm::ArrayType::get(builder->getIntNTy(LL_VECTOR_REGISTER_SIZE), 16));
    llvm::Type* cpu_type = llvm::StructType::get(builder->getContext(), cpu_types);
    llvm::Type* cpu_type_ptr = llvm::PointerType::get(cpu_type, 0);
    llvm::Type* void_type = builder->getVoidTy();
    llvm::Type* fn_type = llvm::FunctionType::get(void_type, {cpu_type_ptr}, false);

    fn->llvm = LLVMAddFunction(mod, name, llvm::wrap(fn_type));
    fn->bbCount = 0;
    fn->bbs = NULL;
    fn->bbsAllocated = 0;

    state->cfg.globalBase = NULL;
    state->cfg.stackSize = 128;
    state->cfg.enableOverflowIntrinsics = false;
    state->cfg.enableFastMath = false;
    state->cfg.enableFullLoopUnroll = false;
    state->cfg.prefer_pointer_cmp = false;

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
ll_func_set_stack_size(LLFunc* fn, size_t size)
{
    fn->state.cfg.stackSize = size;
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

static
void
ll_func_optimize(LLVMValueRef llvm_fn)
{
    LLVMPassManagerRef pm = LLVMCreateFunctionPassManagerForModule(LLVMGetGlobalParent(llvm_fn));
    LLVMInitializeFunctionPassManager(pm);

    // Fold some common subexpressions
    LLVMAddEarlyCSEPass(pm);
    // Replace aggregates (i.e. cpu type struct) with scalars
    LLVMAddScalarReplAggregatesPass(pm);
    // Combine instructions to simplify code
    LLVMAddInstructionCombiningPass(pm);
    // Aggressive DCE to remove phi cycles, etc.
    LLVMAddAggressiveDCEPass(pm);
    // Simplify CFG, removes some redundant function exists and empty blocks
    LLVMAddCFGSimplificationPass(pm);

    LLVMRunFunctionPassManager(pm, llvm_fn);

    LLVMFinalizeFunctionPassManager(pm);
    LLVMDisposePassManager(pm);
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
    ll_func_optimize(fn->llvm);

    return fn->llvm;
}

LLVMValueRef
ll_func_wrap_sysv(LLVMValueRef llvm_fn, LLVMTypeRef ty, LLVMModuleRef mod)
{
    llvm::LLVMContext& ctx = llvm::unwrap(mod)->getContext();
    llvm::Function* orig_fn = llvm::unwrap<llvm::Function>(llvm_fn);
    llvm::FunctionType* fn_ty = llvm::unwrap<llvm::FunctionType>(ty);
    llvm::Function* new_fn = llvm::Function::Create(fn_ty, llvm::GlobalValue::ExternalLinkage, "glob", llvm::unwrap(mod));
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(ctx, "", new_fn, nullptr);

    llvm::IRBuilder<>* builder = new llvm::IRBuilder<>(ctx);
    builder->SetInsertPoint(llvm_bb);

    llvm::FunctionType* cpu_call_type = orig_fn->getFunctionType();
    llvm::Type* cpu_type = cpu_call_type->getParamType(0)->getPointerElementType();
    llvm::Value* cpu_arg = llvm::UndefValue::get(cpu_type);

    unsigned gp_regs[6] = { 7, 6, 2, 1, 8, 9 };
    unsigned gpRegOffset = 0;
    unsigned fpRegOffset = 0;
    for (auto arg = new_fn->arg_begin(); arg != new_fn->arg_end(); ++arg)
    {
        llvm::Type::TypeID type_kind = arg->getType()->getTypeID();

        if (type_kind == llvm::Type::TypeID::IntegerTyID)
        {
            cpu_arg = builder->CreateInsertValue(cpu_arg, arg, {1, gp_regs[gpRegOffset]});
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::PointerTyID)
        {
            llvm::Value* intval = builder->CreatePtrToInt(arg, builder->getInt64Ty());
            cpu_arg = builder->CreateInsertValue(cpu_arg, intval, {1, gp_regs[gpRegOffset]});
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::FloatTyID || type_kind == llvm::Type::TypeID::DoubleTyID)
        {
            llvm::Type* int_type = builder->getIntNTy(arg->getType()->getPrimitiveSizeInBits());
            llvm::Type* vec_type = builder->getIntNTy(LL_VECTOR_REGISTER_SIZE);
            llvm::Value* intval = builder->CreateBitCast(arg, int_type);
            llvm::Value* ext = builder->CreateZExt(intval, vec_type);
            cpu_arg = builder->CreateInsertValue(cpu_arg, ext, {3, fpRegOffset});
            fpRegOffset++;
        }
        else
            warn_if_reached();
    }

    llvm::Value* alloca = builder->CreateAlloca(cpu_type, int{0});
    builder->CreateStore(cpu_arg, alloca);
    llvm::CallInst* call = builder->CreateCall(cpu_call_type, orig_fn, {alloca});
    cpu_arg = builder->CreateLoad(cpu_type, alloca);

    llvm::Type* ret_type = new_fn->getReturnType();
    switch (ret_type->getTypeID())
    {
        llvm::Value* ret;

        case llvm::Type::TypeID::VoidTyID:
            builder->CreateRetVoid();
            break;
        case llvm::Type::TypeID::IntegerTyID:
            ret = builder->CreateExtractValue(cpu_arg, {1, 0});
            ret = builder->CreateTruncOrBitCast(ret, ret_type);
            builder->CreateRet(ret);
            break;
        case llvm::Type::TypeID::PointerTyID:
            ret = builder->CreateExtractValue(cpu_arg, {1, 0});
            ret = builder->CreateIntToPtr(ret, ret_type);
            builder->CreateRet(ret);
            break;
        case llvm::Type::TypeID::FloatTyID:
        case llvm::Type::TypeID::DoubleTyID:
            ret = builder->CreateExtractValue(cpu_arg, {3, 0});
            ret = builder->CreateTrunc(ret, builder->getIntNTy(ret_type->getPrimitiveSizeInBits()));
            ret = builder->CreateBitCast(ret, ret_type);
            builder->CreateRet(ret);
            break;
        default:
            warn_if_reached();
            break;
    }

    llvm::InlineFunctionInfo ifi;
    llvm::InlineFunction(llvm::CallSite(call), ifi);

    bool error = LLVMVerifyFunction(llvm::wrap(new_fn), LLVMPrintMessageAction);
    if (error)
        return NULL;

    ll_func_optimize(llvm::wrap(new_fn));

    return llvm::wrap(new_fn);
}

/**
 * @}
 **/
