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
#include <unordered_map>

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

/**
 * \defgroup LLFunc Func2
 * \brief Representation of a function
 *
 * @{
 **/

namespace rellume
{

void Function::CreateEntry(BasicBlock& entry_bb)
{
    llvm::BasicBlock* first_bb = llvm->empty() ? nullptr : &llvm->front();
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(state.irb.getContext(), "", llvm, first_bb);
    llvm::IRBuilder<> irb(llvm_bb);
    RegFile rf(llvm_bb);

    llvm::Value* param = llvm->arg_begin();
    llvm::Value* regs = irb.CreateLoad(param);

    llvm::Value* next_rip = irb.CreateExtractValue(regs, {0});
    rf.SetReg(LLReg(LL_RT_IP, 0), Facet::I64, next_rip, true);

    for (unsigned i = 0; i < LL_RI_GPMax; i++)
        rf.SetReg(LLReg(LL_RT_GP64, i), Facet::I64, irb.CreateExtractValue(regs, {1, i}), true);

    for (unsigned i = 0; i < LL_RI_XMMMax; i++)
        rf.SetReg(LLReg(LL_RT_XMM, i), Facet::IVEC, irb.CreateExtractValue(regs, {3, i}), true);

    for (unsigned i = 0; i < RFLAG_Max; i++)
        rf.SetFlag(i, irb.CreateExtractValue(regs, {2, i}));

    irb.CreateBr(entry_bb.Llvm());

    entry_bb.AddToPhis(llvm_bb, rf);
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

BasicBlock* Function::AddBlock(uint64_t address)
{
    if (block_map.size() == 0)
        entry_addr = address;

    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(llvm->getContext(), "", llvm, nullptr);
    block_map[address] = std::make_unique<BasicBlock>(llvm_bb, state);
    return block_map[address].get();
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

std::unique_ptr<BasicBlock> Function::CreateExit() {
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(state.irb.getContext(), "", llvm, nullptr);
    auto exit_block = std::make_unique<BasicBlock>(llvm_bb, state);
    exit_block->SetCurrent();

    // Pack CPU struct and return
    llvm::Value* param = llvm->arg_begin();
    llvm::Type* cpu_type = param->getType()->getPointerElementType();
    llvm::Value* result = llvm::UndefValue::get(cpu_type);

    llvm::Value* value = state.GetReg(LLReg(LL_RT_IP, 0), Facet::I64);
    result = state.irb.CreateInsertValue(result, value, {0});

    for (unsigned i = 0; i < LL_RI_GPMax; i++)
    {
        value = state.GetReg(LLReg(LL_RT_GP64, i), Facet::I64);
        result = state.irb.CreateInsertValue(result, value, {1, i});
    }

    for (unsigned i = 0; i < LL_RI_XMMMax; i++)
    {
        value = state.GetReg(LLReg(LL_RT_XMM, i), Facet::IVEC);
        result = state.irb.CreateInsertValue(result, value, {3, i});
    }

    for (unsigned i = 0; i < RFLAG_Max; i++)
    {
        value = state.GetFlag(i);
        result = state.irb.CreateInsertValue(result, value, {2, i});
    }

    state.irb.CreateStore(result, param);
    state.irb.CreateRetVoid();

    return exit_block;
}

BasicBlock& Function::ResolveAddr(llvm::Value* addr, BasicBlock& def) {
    if (auto const_addr = llvm::dyn_cast<llvm::ConstantInt>(addr))
    {
        auto block_it = block_map.find(const_addr->getZExtValue());
        if (block_it != block_map.end())
            return *(block_it->second);
    }
    return def;
}

llvm::Function* Function::Lift()
{
    if (block_map.size() == 0)
        return NULL;

    CreateEntry(*block_map[entry_addr]);
    std::unique_ptr<BasicBlock> exit_block = CreateExit();

    for (auto it = block_map.begin(); it != block_map.end(); ++it)
    {
        it->second->SetCurrent();

        llvm::Value* next_rip = state.GetReg(LLReg(LL_RT_IP, 0), Facet::I64);
        if (auto select = llvm::dyn_cast<llvm::SelectInst>(next_rip))
        {
            BasicBlock& true_block = ResolveAddr(select->getTrueValue(), *exit_block);
            BasicBlock& false_block = ResolveAddr(select->getFalseValue(), *exit_block);

            // In case both blocks are the same create a single branch only.
            if (std::addressof(true_block) != std::addressof(false_block))
            {
                state.irb.CreateCondBr(select->getCondition(),
                                       true_block.Llvm(), false_block.Llvm());
                true_block.AddToPhis(*(it->second));
                false_block.AddToPhis(*(it->second));
            }
            else
            {
                state.irb.CreateBr(true_block.Llvm());
                true_block.AddToPhis(*(it->second));
            }
        }
        else
        {
            BasicBlock& next_block = ResolveAddr(next_rip, *exit_block);
            state.irb.CreateBr(next_block.Llvm());
            next_block.AddToPhis(*(it->second));
        }
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
