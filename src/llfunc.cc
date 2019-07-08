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
#include <vector>
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
#include <llvm/IR/Verifier.h>
#include <llvm/Transforms/Utils/Cloning.h>

#include <llfunction-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <lloperand-internal.h>
#include <llregfile-internal.h>

/**
 * \defgroup LLFunc Func2
 * \brief Representation of a function
 *
 * @{
 **/

namespace rellume
{

void Function::CreateEntry()
{
    llvm::BasicBlock* first_bb = llvm->empty() ? nullptr : &llvm->front();
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(state.irb.getContext(), "", llvm, first_bb);
    initialBB = new BasicBlock(llvm_bb, state);
    initialBB->SetCurrent();

    llvm::Value* param = llvm->arg_begin();
    llvm::IRBuilder<>* builder = llvm::unwrap(state.builder);

    llvm::Value* regs = builder->CreateLoad(param);
    for (unsigned i = 0; i < LL_RI_GPMax; i++)
        state.SetReg(LLReg(LL_RT_GP64, i), Facet::I64, builder->CreateExtractValue(regs, {1, i}));

    for (unsigned i = 0; i < LL_RI_XMMMax; i++)
        state.SetReg(LLReg(LL_RT_XMM, i), Facet::IVEC, builder->CreateExtractValue(regs, {3, i}));

    for (unsigned i = 0; i < RFLAG_Max; i++)
        state.SetFlag(i, builder->CreateExtractValue(regs, {2, i}));
}

Function::Function(llvm::Module* mod) : state(mod->getContext())
{
    LLState* state = &this->state;

    llvm::IRBuilder<>* builder = &state->irb;
    llvm::SmallVector<llvm::Type*, 4> cpu_types;
    cpu_types.push_back(builder->getInt64Ty()); // instruction pointer
    cpu_types.push_back(llvm::ArrayType::get(builder->getInt64Ty(), 16));
    cpu_types.push_back(llvm::ArrayType::get(builder->getInt1Ty(), 6));
    cpu_types.push_back(llvm::ArrayType::get(builder->getIntNTy(LL_VECTOR_REGISTER_SIZE), 16));
    llvm::Type* cpu_type = llvm::StructType::get(builder->getContext(), cpu_types);
    llvm::Type* cpu_type_ptr = llvm::PointerType::get(cpu_type, 0);
    llvm::Type* void_type = builder->getVoidTy();
    llvm::FunctionType* fn_type = llvm::FunctionType::get(void_type, {cpu_type_ptr}, false);

    llvm = llvm::Function::Create(fn_type, llvm::GlobalValue::ExternalLinkage, "", mod);
    initialBB = nullptr;

    state->cfg.globalBase = NULL;
    state->cfg.stackSize = 128;
    state->cfg.enableOverflowIntrinsics = false;
    state->cfg.enableFastMath = false;
    state->cfg.prefer_pointer_cmp = false;
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
void Function::EnableOverflowIntrinsics(bool enable)
{
    state.cfg.enableOverflowIntrinsics = enable;
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
void Function::EnableFastMath(bool enable)
{
    state.cfg.enableFastMath = enable;
}

void Function::SetGlobalBase(uintptr_t base, llvm::Value* value)
{
    state.cfg.globalOffsetBase = base;
    state.cfg.globalBase = llvm::wrap(value);
}

/**
 * Dispose a function.
 *
 * \author Alexis Engelke
 *
 * \param fn The function
 **/
Function::~Function()
{
    for (auto it = blocks.begin(); it != blocks.end(); ++it)
        delete *it;
    if (initialBB != nullptr)
        delete initialBB;
}

BasicBlock* Function::AddBlock()
{
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(llvm->getContext(), "", llvm, nullptr);
    BasicBlock* bb = new BasicBlock(llvm_bb, state);
    bb->AddPhis();
    blocks.push_back(bb);
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

llvm::Function* Function::Lift()
{
    if (blocks.size() == 0)
        return NULL;

    CreateEntry();

    // The initial basic block falls through to the first lifted block.
    initialBB->AddBranches(NULL, blocks[0]);
    initialBB->Terminate();

    for (auto it = blocks.begin(); it != blocks.end(); ++it)
    {
        (*it)->Terminate();
        (*it)->FillPhis();
    }

    if (llvm::verifyFunction(*(llvm), &llvm::errs()))
        return NULL;

    // Run some optimization passes to remove most of the bloat
    ll_func_optimize(llvm::wrap(llvm));

    return llvm;
}

}

extern "C"
__attribute__((visibility("default")))
LLVMValueRef
ll_func_wrap_sysv(LLVMValueRef llvm_fn, LLVMTypeRef ty, LLVMModuleRef mod, size_t stack_size)
{
    llvm::LLVMContext& ctx = llvm::unwrap(mod)->getContext();
    llvm::Function* orig_fn = llvm::unwrap<llvm::Function>(llvm_fn);
    llvm::FunctionType* fn_ty = llvm::unwrap<llvm::FunctionType>(ty);
    llvm::Function* new_fn = llvm::Function::Create(fn_ty, llvm::GlobalValue::ExternalLinkage, "glob", llvm::unwrap(mod));
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(ctx, "", new_fn);

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

    llvm::Value* stack_sz_val = builder->getInt64(stack_size);
    llvm::AllocaInst* stack = builder->CreateAlloca(builder->getInt8Ty(), stack_sz_val);
    stack->setAlignment(16);
    llvm::Value* sp_ptr = builder->CreateGEP(stack, {stack_sz_val});
    llvm::Value* sp = builder->CreatePtrToInt(sp_ptr, builder->getInt64Ty());
    cpu_arg = builder->CreateInsertValue(cpu_arg, sp, {1, 4});

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

    delete builder;

    llvm::InlineFunctionInfo ifi;
    llvm::InlineFunction(llvm::CallSite(call), ifi);

    bool error = LLVMVerifyFunction(llvm::wrap(new_fn), LLVMPrintMessageAction);
    if (error)
        return NULL;

    rellume::ll_func_optimize(llvm::wrap(new_fn));

    return llvm::wrap(new_fn);
}

/**
 * @}
 **/
