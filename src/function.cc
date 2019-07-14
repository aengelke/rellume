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

#include "function.h"

#include "basicblock.h"
#include "config.h"
#include "lifter.h"
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
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Transforms/InstCombine/InstCombine.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Cloning.h>
#include <cassert>
#include <cstdint>
#include <vector>
#include <unordered_map>


/**
 * \defgroup LLFunc Func2
 * \brief Representation of a function
 *
 * @{
 **/

namespace rellume {

void Function::CreateEntry(BasicBlock& entry_bb)
{
    llvm::BasicBlock* first_bb = llvm->empty() ? nullptr : &llvm->front();
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(llvm->getContext(), "", llvm, first_bb);
    llvm::IRBuilder<> irb(llvm_bb);
    // FIXME: no longer leak memory
    RegFile& rf = *(new RegFile(llvm_bb));

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

Function::Function(llvm::Module* mod)
{
    llvm::IRBuilder<> irb(mod->getContext());
    llvm::SmallVector<llvm::Type*, 4> cpu_types;
    cpu_types.push_back(irb.getInt64Ty()); // instruction pointer
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt64Ty(), 16));
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt1Ty(), 6));
    cpu_types.push_back(llvm::ArrayType::get(irb.getIntNTy(LL_VECTOR_REGISTER_SIZE), 16));
    llvm::Type* cpu_type = llvm::StructType::get(irb.getContext(), cpu_types);
    llvm::Type* cpu_type_ptr = llvm::PointerType::get(cpu_type, 0);
    llvm::Type* void_type = irb.getVoidTy();
    llvm::FunctionType* fn_type = llvm::FunctionType::get(void_type, {cpu_type_ptr}, false);

    llvm = llvm::Function::Create(fn_type, llvm::GlobalValue::ExternalLinkage, "", mod);

    cfg.global_base_value = nullptr;
    cfg.enableOverflowIntrinsics = false;
    cfg.enableFastMath = false;
    cfg.prefer_pointer_cmp = false;
    cfg.verify_ir = false;
}

Function::~Function() = default;

void Function::AddInst(uint64_t block_addr, const LLInstr& inst)
{
    auto block_it = block_map.find(block_addr);
    if (block_it == block_map.end()) {
        if (block_map.size() == 0)
            entry_addr = block_addr;

        llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(llvm->getContext(),
                                                             "", llvm, nullptr);
        block_map[block_addr] = std::make_unique<BasicBlock>(llvm_bb);
    }

    block_map[block_addr]->AddInst(inst, cfg);
}

std::unique_ptr<BasicBlock> Function::CreateExit() {
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(llvm->getContext(), "", llvm, nullptr);
    auto exit_block = std::make_unique<BasicBlock>(llvm_bb);
    Lifter state(cfg, exit_block->regfile, exit_block->Llvm());

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
        Lifter state(cfg, it->second->regfile, it->second->Llvm());

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

    // Walk over blocks as long as phi nodes could have been added. We stop when
    // alls phis are filled.
    // TODO: improve walk ordering and efficiency (e.g. by adding predecessors
    // to the set of remaining blocks when something changed)
    bool changed = true;
    while (changed) {
        changed = false;
        for (auto& item : block_map)
            changed |= item.second->FillPhis();
        changed |= exit_block->FillPhis();
    }

    if (cfg.verify_ir && llvm::verifyFunction(*(llvm), &llvm::errs()))
        return NULL;

    // Run some optimization passes to remove most of the bloat
    llvm::legacy::FunctionPassManager pm(llvm->getParent());
    pm.doInitialization();

    // Aggressive DCE to remove phi cycles, etc.
    pm.add(llvm::createAggressiveDCEPass());
    // Fold some common subexpressions
    pm.add(llvm::createEarlyCSEPass());
    // Combine instructions to simplify code, but avoid expensive transforms
    pm.add(llvm::createInstructionCombiningPass(false));

    pm.run(*llvm);
    pm.doFinalization();

    return llvm;
}

}

extern "C"
__attribute__((visibility("default")))
LLVMValueRef
ll_func_wrap_sysv(LLVMValueRef llvm_fn, LLVMTypeRef ty, LLVMModuleRef mod, size_t stack_size)
{
    llvm::Function* orig_fn = llvm::unwrap<llvm::Function>(llvm_fn);
    llvm::LLVMContext& ctx = orig_fn->getContext();
    llvm::FunctionType* fn_ty = llvm::unwrap<llvm::FunctionType>(ty);
    llvm::Function* new_fn = llvm::Function::Create(fn_ty, llvm::GlobalValue::ExternalLinkage, "glob", orig_fn->getParent());
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(ctx, "", new_fn);

    llvm::IRBuilder<> irb(llvm_bb);

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
            cpu_arg = irb.CreateInsertValue(cpu_arg, arg, {1, gp_regs[gpRegOffset]});
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::PointerTyID)
        {
            llvm::Value* intval = irb.CreatePtrToInt(arg, irb.getInt64Ty());
            cpu_arg = irb.CreateInsertValue(cpu_arg, intval, {1, gp_regs[gpRegOffset]});
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::FloatTyID || type_kind == llvm::Type::TypeID::DoubleTyID)
        {
            llvm::Type* int_type = irb.getIntNTy(arg->getType()->getPrimitiveSizeInBits());
            llvm::Type* vec_type = irb.getIntNTy(LL_VECTOR_REGISTER_SIZE);
            llvm::Value* intval = irb.CreateBitCast(arg, int_type);
            llvm::Value* ext = irb.CreateZExt(intval, vec_type);
            cpu_arg = irb.CreateInsertValue(cpu_arg, ext, {3, fpRegOffset});
            fpRegOffset++;
        }
        else
        {
            assert(false);
        }
    }

    llvm::Value* stack_sz_val = irb.getInt64(stack_size);
    llvm::AllocaInst* stack = irb.CreateAlloca(irb.getInt8Ty(), stack_sz_val);
    stack->setAlignment(16);
    llvm::Value* sp_ptr = irb.CreateGEP(stack, {stack_sz_val});
    llvm::Value* sp = irb.CreatePtrToInt(sp_ptr, irb.getInt64Ty());
    cpu_arg = irb.CreateInsertValue(cpu_arg, sp, {1, 4});

    llvm::Value* alloca = irb.CreateAlloca(cpu_type, int{0});
    irb.CreateStore(cpu_arg, alloca);
    llvm::CallInst* call = irb.CreateCall(cpu_call_type, orig_fn, {alloca});
    cpu_arg = irb.CreateLoad(cpu_type, alloca);

    llvm::Type* ret_type = new_fn->getReturnType();
    switch (ret_type->getTypeID())
    {
        llvm::Value* ret;

        case llvm::Type::TypeID::VoidTyID:
            irb.CreateRetVoid();
            break;
        case llvm::Type::TypeID::IntegerTyID:
            ret = irb.CreateExtractValue(cpu_arg, {1, 0});
            ret = irb.CreateTruncOrBitCast(ret, ret_type);
            irb.CreateRet(ret);
            break;
        case llvm::Type::TypeID::PointerTyID:
            ret = irb.CreateExtractValue(cpu_arg, {1, 0});
            ret = irb.CreateIntToPtr(ret, ret_type);
            irb.CreateRet(ret);
            break;
        case llvm::Type::TypeID::FloatTyID:
        case llvm::Type::TypeID::DoubleTyID:
            ret = irb.CreateExtractValue(cpu_arg, {3, 0});
            ret = irb.CreateTrunc(ret, irb.getIntNTy(ret_type->getPrimitiveSizeInBits()));
            ret = irb.CreateBitCast(ret, ret_type);
            irb.CreateRet(ret);
            break;
        default:
            assert(false);
            break;
    }

    llvm::InlineFunctionInfo ifi;
    llvm::InlineFunction(llvm::CallSite(call), ifi);

    if (llvm::verifyFunction(*new_fn, &llvm::errs()))
        return NULL;

    llvm::legacy::FunctionPassManager pm(new_fn->getParent());
    pm.doInitialization();

    // instrcombine will get rid of the CPU type struct
    pm.add(llvm::createInstructionCombiningPass(false));
    // Simplify CFG, removes some redundant function exists and empty blocks
    pm.add(llvm::createCFGSimplificationPass());

    pm.run(*new_fn);
    pm.doFinalization();

    return llvm::wrap(new_fn);
}

/**
 * @}
 **/
