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
#include <memory>
#include <vector>
#include <unordered_map>


/**
 * \defgroup LLFunc Func2
 * \brief Representation of a function
 *
 * @{
 **/

namespace rellume {

static llvm::Value* GepHelper(llvm::IRBuilder<>& irb, llvm::Value* base, llvm::ArrayRef<unsigned> idxs) {
    llvm::SmallVector<llvm::Value*, 4> consts;
    for (auto& idx : idxs)
        consts.push_back(irb.getInt32(idx));
    return irb.CreateGEP(base, consts);
}

std::unique_ptr<BasicBlock> Function::CreateEntry()
{
    llvm::BasicBlock* first_bb = llvm->empty() ? nullptr : &llvm->front();
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(llvm->getContext(), "", llvm, first_bb);
    auto entry_block = std::make_unique<BasicBlock>(llvm_bb);

    entry_block->regfile.UpdateAllFromMem(llvm->arg_begin());

    return entry_block;
}

Function::Function(llvm::Module* mod)
{
    llvm::LLVMContext& ctx = mod->getContext();
    llvm::Type* void_type = llvm::Type::getVoidTy(ctx);
    llvm::Type* i8p_type = llvm::Type::getInt8PtrTy(ctx);
    auto fn_type = llvm::FunctionType::get(void_type, {i8p_type}, false);

    llvm = llvm::Function::Create(fn_type, llvm::GlobalValue::ExternalLinkage, "", mod);
    llvm->addParamAttr(0, llvm::Attribute::NoAlias);
    llvm->addParamAttr(0, llvm::Attribute::NoCapture);
    llvm->addParamAttr(0, llvm::Attribute::getWithAlignment(ctx, 16));
    llvm->addDereferenceableParamAttr(0, 0x190);

    cfg.global_base_value = nullptr;
    cfg.enableOverflowIntrinsics = false;
    cfg.enableFastMath = false;
    cfg.prefer_pointer_cmp = false;
    cfg.verify_ir = false;
    cfg.optimize_ir = true;
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

    // Pack CPU struct and return
    exit_block->regfile.UpdateAllInMem(llvm->arg_begin());

    llvm::IRBuilder<> irb(exit_block->Llvm());
    irb.CreateRetVoid();

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

    // !!! DANGER !!!
    // The entry and exit blocks go out of scope when this function returns, but
    // other blocks may still have references to them. This means:
    //
    //    AFTER Lift() HAS BEEN CALLED, DO NOT TOUCH ANY BASIC BLOCK!
    std::unique_ptr<BasicBlock> entry_block = CreateEntry();
    std::unique_ptr<BasicBlock> exit_block = CreateExit();

    entry_block->BranchTo(*block_map[entry_addr]);

    for (auto it = block_map.begin(); it != block_map.end(); ++it) {
        llvm::Value* next_rip = it->second->regfile.GetReg(LLReg(LL_RT_IP, 0), Facet::I64);
        if (auto select = llvm::dyn_cast<llvm::SelectInst>(next_rip)) {
            it->second->BranchTo(select->getCondition(),
                                 ResolveAddr(select->getTrueValue(), *exit_block),
                                 ResolveAddr(select->getFalseValue(), *exit_block));
        } else {
            it->second->BranchTo(ResolveAddr(next_rip, *exit_block));
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

    if (cfg.optimize_ir) {
        // Run some optimization passes to remove most of the bloat
        llvm::legacy::FunctionPassManager pm(llvm->getParent());
        pm.doInitialization();

        // Aggressive DCE to remove phi cycles, etc.
        pm.add(llvm::createAggressiveDCEPass());
        // Fold some common subexpressions with MemorySSA to remove obsolete stores
        pm.add(llvm::createEarlyCSEPass(true));
        // Combine instructions to simplify code, but avoid expensive transforms
        pm.add(llvm::createInstructionCombiningPass(false));

        pm.run(*llvm);
        pm.doFinalization();
    }

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

    llvm::SmallVector<llvm::Type*, 4> cpu_types;
    cpu_types.push_back(irb.getInt64Ty()); // instruction pointer
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt64Ty(), 16));
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt1Ty(), 6));
    cpu_types.push_back(llvm::ArrayType::get(irb.getIntNTy(LL_VECTOR_REGISTER_SIZE), 16));
    llvm::Type* cpu_type = llvm::StructType::get(irb.getContext(), cpu_types);

    llvm::Value* alloca = irb.CreateAlloca(cpu_type, int{0});

    unsigned gp_regs[6] = { 7, 6, 2, 1, 8, 9 };
    unsigned gpRegOffset = 0;
    unsigned fpRegOffset = 0;
    for (auto arg = new_fn->arg_begin(); arg != new_fn->arg_end(); ++arg)
    {
        llvm::Type::TypeID type_kind = arg->getType()->getTypeID();

        if (type_kind == llvm::Type::TypeID::IntegerTyID)
        {
            irb.CreateStore(arg, rellume::GepHelper(irb, alloca, {0, 1, gp_regs[gpRegOffset]}));
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::PointerTyID)
        {
            llvm::Value* intval = irb.CreatePtrToInt(arg, irb.getInt64Ty());
            irb.CreateStore(intval, rellume::GepHelper(irb, alloca, {0, 1, gp_regs[gpRegOffset]}));
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::FloatTyID || type_kind == llvm::Type::TypeID::DoubleTyID)
        {
            llvm::Type* int_type = irb.getIntNTy(arg->getType()->getPrimitiveSizeInBits());
            llvm::Type* vec_type = irb.getIntNTy(LL_VECTOR_REGISTER_SIZE);
            llvm::Value* intval = irb.CreateBitCast(arg, int_type);
            llvm::Value* ext = irb.CreateZExt(intval, vec_type);
            irb.CreateStore(ext, rellume::GepHelper(irb, alloca, {0, 3, fpRegOffset}));
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
    irb.CreateStore(sp, rellume::GepHelper(irb, alloca, {0, 1, 4}));

    llvm::Value* call_arg = irb.CreatePointerCast(alloca, irb.getInt8PtrTy());
    llvm::CallInst* call = irb.CreateCall(orig_fn, {call_arg});

    llvm::Type* ret_type = new_fn->getReturnType();
    switch (ret_type->getTypeID())
    {
        llvm::Value* ret;

        case llvm::Type::TypeID::VoidTyID:
            irb.CreateRetVoid();
            break;
        case llvm::Type::TypeID::IntegerTyID:
            ret = irb.CreateLoad(rellume::GepHelper(irb, alloca, {0, 1, 0}));
            ret = irb.CreateTruncOrBitCast(ret, ret_type);
            irb.CreateRet(ret);
            break;
        case llvm::Type::TypeID::PointerTyID:
            ret = irb.CreateLoad(rellume::GepHelper(irb, alloca, {0, 1, 0}));
            ret = irb.CreateIntToPtr(ret, ret_type);
            irb.CreateRet(ret);
            break;
        case llvm::Type::TypeID::FloatTyID:
        case llvm::Type::TypeID::DoubleTyID:
            ret = irb.CreateLoad(rellume::GepHelper(irb, alloca, {0, 3, 0}));
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

    // replace CPU struct with scalars
    pm.add(llvm::createSROAPass());
    // instrcombine will get rid of lots of bloat from the CPU struct
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
