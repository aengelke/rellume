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

#include "facet.h"
#include "regfile.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Metadata.h>
#include <cstdio>
#include <deque>
#include <set>
#include <vector>



/**
 * \defgroup LLBasicBlock Basic Block
 * \brief Representation of a basic block
 *
 * @{
 **/

namespace rellume {

BasicBlock::BasicBlock(llvm::Function* fn, Phis phi_mode)
        : regfile() {
    llvm_block = llvm::BasicBlock::Create(fn->getContext(), "", fn, nullptr);
    regfile.SetInsertBlock(llvm_block);

    if (phi_mode != Phis::NONE) {
        // Initialize all registers with a generator which adds a PHI node when
        // the value-facet combination is requested.
        empty_phis.reserve(32);
        regfile.InitWithPHIs(&empty_phis, /*all=*/phi_mode == Phis::ALL);
    }
}

void BasicBlock::BranchTo(BasicBlock& next) {
    assert(!llvm_block->getTerminator() && "attempting to add second terminator");

    llvm::IRBuilder<> irb(llvm_block);
    irb.CreateBr(next.llvm_block);
    next.predecessors.push_back(this);
    successors.push_back(&next);
}

void BasicBlock::BranchTo(llvm::Value* cond, BasicBlock& then,
                             BasicBlock& other) {
    // In case both blocks are the same create a single branch only.
    if (std::addressof(then) == std::addressof(other)) {
        BranchTo(then);
        return;
    }

    assert(!llvm_block->getTerminator() && "attempting to add second terminator");

    llvm::IRBuilder<> irb(llvm_block);
    irb.CreateCondBr(cond, then.llvm_block, other.llvm_block);
    then.predecessors.push_back(this);
    other.predecessors.push_back(this);
    successors.push_back(&then);
    successors.push_back(&other);
}

bool BasicBlock::FillPhis() {
    if (empty_phis.empty())
        return false;

    for (const auto& [reg, facet, phi] : empty_phis) {
        // This makes use of the property that a RegFile will never store a PHI
        // node using SetReg. Otherwise things will blow up, because the
        // register file may still have a reference to the (currently) unused
        // PHI node.
        if (phi->user_empty()) {
            phi->eraseFromParent();
            continue;
        }
        for (BasicBlock* pred : predecessors) {
            llvm::Value* value = pred->regfile.GetReg(reg, facet);
            if (facet == Facet::PTR && value->getType() != phi->getType()) {
                llvm::IRBuilder<> irb(pred->llvm_block->getTerminator());
                value = irb.CreatePointerCast(value, phi->getType());
            }
            phi->addIncoming(value, pred->llvm_block);
        }
    }
    empty_phis.clear();

    return true;
}

} // namespace

/**
 * @}
 **/
