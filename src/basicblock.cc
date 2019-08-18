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

#include "basicblock.h"

#include "config.h"
#include "facet.h"
#include "lifter.h"
#include "regfile.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Metadata.h>
#include <cstdio>
#include <vector>



/**
 * \defgroup LLBasicBlock Basic Block
 * \brief Representation of a basic block
 *
 * @{
 **/

namespace rellume {

BasicBlock::BasicBlock(llvm::Function* fn, Kind kind, llvm::Value* mem_arg) :
            regfile() {
    first_block = llvm::BasicBlock::Create(fn->getContext(), "", fn, nullptr);
    regfile.SetInsertBlock(first_block);

    if (kind != ENTRY) {
        // Initialize all registers with a generator which adds a PHI node when
        // the value-facet combination is requested.
        regfile.InitAll([this](const LLReg reg, const Facet facet) {
            return [this, reg, facet]() {
                llvm::IRBuilder<> irb(first_block, first_block->begin());
                auto phi = irb.CreatePHI(facet.Type(irb.getContext()), 4);
                empty_phis.push_back(std::make_tuple(reg, facet, phi));
                return phi;
            };
        });
    } else { // kind == ENTRY
        regfile.UpdateAllFromMem(mem_arg);
    }

    if (kind == EXIT) {
        // Exit block packs the values into memory and returns.
        regfile.UpdateAllInMem(mem_arg);
        llvm::IRBuilder<> irb(first_block);
        irb.CreateRetVoid();
    }
}

void BasicBlock::AddInst(const LLInstr& inst, const LLConfig& cfg)
{
    // Set new instruction pointer register
    llvm::IRBuilder<> irb(EndBlock());
    llvm::Value* ripValue = irb.getInt64(inst.addr + inst.len);
    regfile.SetReg(LLReg(LL_RT_IP, 0), Facet::I64, ripValue, true);

    // Add separator for debugging.
    llvm::Function* intrinsicDoNothing = llvm::Intrinsic::getDeclaration(EndBlock()->getModule(), llvm::Intrinsic::donothing, {});
    irb.CreateCall(intrinsicDoNothing);

    Lifter state(cfg, regfile);

    // Check overridden implementations first.
    const auto& override = cfg.instr_overrides.find(inst.type);
    if (override != cfg.instr_overrides.end()) {
        state.LiftOverride(inst, override->second);
        return;
    }

    switch (inst.type)
    {
#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include "rellume/opcodes.inc"
#undef DEF_IT

        default:
    not_implemented:
            fprintf(stderr, "Could not handle instruction at %#zx\n", inst.addr);
            assert(0);
            break;
    }
}

void BasicBlock::BranchTo(BasicBlock& next) {
    llvm::IRBuilder<> irb(EndBlock());
    irb.CreateBr(next.first_block);
    next.predecessors.push_back(this);
}

void BasicBlock::BranchTo(llvm::Value* cond, BasicBlock& then,
                             BasicBlock& other) {
    // In case both blocks are the same create a single branch only.
    if (std::addressof(then) == std::addressof(other)) {
        BranchTo(then);
        return;
    }

    llvm::IRBuilder<> irb(EndBlock());
    irb.CreateCondBr(cond, then.first_block, other.first_block);
    then.predecessors.push_back(this);
    other.predecessors.push_back(this);
}

bool BasicBlock::FillPhis() {
    if (empty_phis.empty())
        return false;

    for (auto& item : empty_phis) {
        LLReg reg = std::get<0>(item);
        Facet facet = std::get<1>(item);
        llvm::PHINode* phi = std::get<2>(item);
        for (BasicBlock* pred : predecessors) {
            llvm::Value* value = pred->regfile.GetReg(reg, facet);
            phi->addIncoming(value, pred->EndBlock());
        }
    }
    empty_phis.clear();

    return true;
}

} // namespace

/**
 * @}
 **/
