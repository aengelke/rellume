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

    entry_block = CreateEntry();

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

    // Pack CPU struct and return
    exit_block->regfile.UpdateAllInMem(llvm->arg_begin());

    llvm::IRBuilder<> irb(exit_block->EndBlock());
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

    exit_block = CreateExit();

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

    return llvm;
}

}

/**
 * @}
 **/
