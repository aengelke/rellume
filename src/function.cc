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
#include "callconv.h"
#include "config.h"
#include "function-info.h"
#include "lifter.h"
#include "regfile.h"
#include <llvm/ADT/DepthFirstIterator.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/InstIterator.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Transforms/Utils/BasicBlockUtils.h>
#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_map>


/**
 * \defgroup LLFunc Func2
 * \brief Representation of a function
 *
 * @{
 **/

namespace rellume {

static llvm::Value* CreateSptr(FunctionInfo& fi, llvm::IRBuilder<> irb,
                               size_t offset, size_t bits) {
    unsigned as = fi.sptr_raw->getType()->getPointerAddressSpace();
    llvm::Value* ptr = irb.CreateConstGEP1_64(fi.sptr_raw, offset);
    return irb.CreatePointerCast(ptr, irb.getIntNTy(bits)->getPointerTo(as));
}

static void CreateSptrs(FunctionInfo& fi, llvm::BasicBlock* llvm_block) {
    llvm::IRBuilder<> irb(llvm_block);

#define RELLUME_NAMED_REG(name,nameu,sz,off) \
    fi.sptr[SptrIdx::nameu] = CreateSptr(fi, irb, off, sz == 1 ? sz : sz * 8);
#include <rellume/cpustruct-private.inc>
#undef RELLUME_NAMED_REG
}

Function::Function(llvm::Module* mod, LLConfig* cfg) : cfg(cfg), fi{}
{
    llvm::LLVMContext& ctx = mod->getContext();
    llvm = llvm::Function::Create(cfg->callconv.FnType(ctx, cfg->sptr_addrspace),
                                  llvm::GlobalValue::ExternalLinkage, "", mod);
    llvm->setCallingConv(cfg->callconv.FnCallConv());

    // CPU struct pointer parameters has some extra properties.
    unsigned cpu_param_idx = cfg->callconv.CpuStructParamIdx();
    llvm->addFnAttr("null-pointer-is-valid", "true");
    llvm->addParamAttr(cpu_param_idx, llvm::Attribute::NoAlias);
    llvm->addParamAttr(cpu_param_idx, llvm::Attribute::NoCapture);
    llvm->addParamAttr(cpu_param_idx, llvm::Attribute::getWithAlignment(ctx, 16));
    llvm->addDereferenceableParamAttr(cpu_param_idx, 0x190);

    fi.fn = llvm;
    fi.sptr_raw = &llvm->arg_begin()[cpu_param_idx];

    // Create entry basic block as first block in the function.
    entry_block = std::make_unique<ArchBasicBlock>(fi, /*no_phis=*/true);

    // Initialize the sptr pointers in the function info.
    RegFile* entry_regfile = entry_block->GetInsertBlock()->GetRegFile();
    CreateSptrs(fi, entry_regfile->GetInsertBlock());
    // And initially fill register file.
    entry_regfile->Clear();
    cfg->callconv.Unpack(*entry_regfile, fi);

    fi.entry_ip_value = entry_regfile->GetReg(X86Reg::IP, Facet::I64);
}

Function::~Function() = default;

bool Function::AddInst(uint64_t block_addr, const Instr& inst)
{
    if (block_map.size() == 0) {
        fi.entry_ip = block_addr;
        if (!cfg->position_independent_code) {
            llvm::Type* i64 = llvm::Type::getInt64Ty(llvm->getContext());
            fi.entry_ip_value = llvm::ConstantInt::get(i64, fi.entry_ip);
        }
    }
    if (block_map.find(block_addr) == block_map.end())
        block_map[block_addr] = std::make_unique<ArchBasicBlock>(fi);

    return LiftInstruction(inst, fi, *cfg, *block_map[block_addr]);
}

ArchBasicBlock& Function::ResolveAddr(llvm::Value* addr) {
    uint64_t addr_skew = 0;
    if (cfg->position_independent_code) {
        addr_skew = fi.entry_ip;
        // Strip off the base_rip from the expression.
        auto binop = llvm::dyn_cast<llvm::BinaryOperator>(addr);
        if (!binop || binop->getOpcode() != llvm::Instruction::Add ||
            binop->getOperand(0) != fi.entry_ip_value)
            return *exit_block;
        addr = binop->getOperand(1);
    }

    if (auto const_addr = llvm::dyn_cast<llvm::ConstantInt>(addr)) {
        auto block_it = block_map.find(addr_skew + const_addr->getZExtValue());
        if (block_it != block_map.end())
            return *(block_it->second);
    }
    return *exit_block;
}

llvm::Function* Function::Lift() {
    if (block_map.size() == 0)
        return nullptr;

    // Merge modified registers from all generated basic blocks.
    RegisterSet modified_regs;
    for (auto& item : block_map)
        modified_regs |= item.second->ModifiedRegs();

    exit_block = std::make_unique<ArchBasicBlock>(fi);

    // Exit block packs values together and optionally returns something.
    RegFile* exit_regfile = exit_block->GetInsertBlock()->GetRegFile();
    llvm::IRBuilder<> irb(exit_regfile->GetInsertBlock());
    irb.CreateRet(cfg->callconv.Pack(*exit_regfile, fi, &modified_regs));

    entry_block->BranchTo(*block_map[fi.entry_ip]);

    for (auto it = block_map.begin(); it != block_map.end(); ++it) {
        RegFile* regfile = it->second->GetInsertBlock()->GetRegFile();
        llvm::Value* next_rip = regfile->GetReg(X86Reg::IP, Facet::I64);
        if (auto select = llvm::dyn_cast<llvm::SelectInst>(next_rip)) {
            it->second->BranchTo(select->getCondition(),
                                 ResolveAddr(select->getTrueValue()),
                                 ResolveAddr(select->getFalseValue()));
        } else {
            it->second->BranchTo(ResolveAddr(next_rip));
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

    // Remove calls to llvm.ssa_copy, which got inserted to avoid PHI nodes in
    // the register file.
    for (auto it = llvm::inst_begin(llvm), e = llvm::inst_end(llvm); it != e;) {
        llvm::Instruction* inst = &*it++;
        auto* intr = llvm::dyn_cast<llvm::IntrinsicInst>(inst);
        if (!intr || intr->getIntrinsicID() != llvm::Intrinsic::ssa_copy)
          continue;

        inst->replaceAllUsesWith(intr->getOperand(0));
        inst->eraseFromParent();
    }

    // Remove blocks without predecessors. This can happen if constants get
    // folded already during construction, e.g. xor eax,eax;test eax,eax;jz
#if LL_LLVM_MAJOR >= 9
    llvm::EliminateUnreachableBlocks(*llvm);
#else
    llvm::df_iterator_default_set<llvm::BasicBlock*> reachable;
    for (llvm::BasicBlock* block : llvm::depth_first_ext(llvm, reachable))
        (void) block;
    llvm::SmallVector<llvm::BasicBlock*, 8> dead_blocks;
    for (llvm::BasicBlock& bb : *llvm)
        if (!reachable.count(&bb))
            dead_blocks.push_back(&bb);
    llvm::DeleteDeadBlocks(dead_blocks);
#endif

    if (cfg->verify_ir && llvm::verifyFunction(*(llvm), &llvm::errs()))
        return nullptr;

    return llvm;
}

}

/**
 * @}
 **/
