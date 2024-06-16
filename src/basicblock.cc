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

ArchBasicBlock::ArchBasicBlock(llvm::Function* fn, size_t max_preds)
        : max_preds(max_preds) {
    llvm_block = llvm::BasicBlock::Create(fn->getContext(), "", fn, nullptr);

    if (max_preds != SIZE_MAX)
        predecessors.reserve(max_preds);
}

void ArchBasicBlock::InitWithPHIs(Arch arch, bool seal) {
    InitEmpty(arch, llvm_block);

    // When sealing the block, we know that no more predecessors will follow.
    if ((seal || max_preds == 1) && predecessors.size() == 1) {
        regfile->InitWithRegFile(predecessors[0]->GetRegFile());
    } else {
        // Initialize all registers with a generator which adds a PHI node
        // when the value-facet combination is requested.
        empty_phis.reserve(32);
        regfile->InitWithPHIs(llvm_block, &empty_phis);
    }
}

void ArchBasicBlock::BranchTo(ArchBasicBlock& next) {
    assert(!EndBlock()->getTerminator() && "attempting to add second terminator");

    llvm::IRBuilder<> irb(EndBlock());
    auto branch = irb.CreateBr(next.llvm_block);
    regfile->SetInsertPoint(branch->getIterator());
    next.predecessors.push_back(this);
    successors.push_back(&next);
}

void ArchBasicBlock::BranchTo(llvm::Value* cond, ArchBasicBlock& then,
                              ArchBasicBlock& other) {
    // In case both blocks are the same create a single branch only.
    if (std::addressof(then) == std::addressof(other)) {
        BranchTo(then);
        return;
    }

    assert(!EndBlock()->getTerminator() && "attempting to add second terminator");

    llvm::IRBuilder<> irb(EndBlock());
    auto branch = irb.CreateCondBr(cond, then.llvm_block, other.llvm_block);
    regfile->SetInsertPoint(branch->getIterator());
    then.predecessors.push_back(this);
    other.predecessors.push_back(this);
    successors.push_back(&then);
    successors.push_back(&other);
}

bool ArchBasicBlock::FillPhis() {
    assert(predecessors.size() <= max_preds);
    if (empty_phis.empty())
        return false;

    for (const auto& [reg, facet, phi] : empty_phis) {
        for (ArchBasicBlock* pred : predecessors) {
            llvm::Value* value = pred->regfile->GetReg(reg, facet);
            phi->addIncoming(value, pred->EndBlock());
            if (predecessors.size() == 1 && phi != value) {
                phi->replaceAllUsesWith(value);
            }
        }
    }
    empty_phis.clear();

    return true;
}

} // namespace

/**
 * @}
 **/
