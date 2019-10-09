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
#include "regfile.h"
#include "rellume/instr.h"
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

BasicBlock::BasicBlock(llvm::Function* fn, const LLConfig& cfg, Kind kind)
        : regfile() {
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
    }

    // For ENTRY or EXIT kinds, we either need to setup all values or store them
    // back to memory.
    if (kind == ENTRY) {
        regfile.InitAll(nullptr);
        cfg.callconv.Unpack(regfile, fn, &mem_ref_values);
    } else if (kind == EXIT) {
        llvm::Value* ret_val = cfg.callconv.Pack(regfile, fn, &mem_ref_values);

        llvm::IRBuilder<> irb(first_block);
        if (ret_val == nullptr)
            irb.CreateRetVoid();
        else
            irb.CreateRet(ret_val);
        terminated = true;
    }
}

void BasicBlock::BranchTo(BasicBlock& next) {
    assert(!terminated && "attempting to add second terminator");

    llvm::IRBuilder<> irb(EndBlock());
    irb.CreateBr(next.first_block);
    next.predecessors.push_back(this);
    terminated = true;
}

void BasicBlock::BranchTo(llvm::Value* cond, BasicBlock& then,
                             BasicBlock& other) {
    // In case both blocks are the same create a single branch only.
    if (std::addressof(then) == std::addressof(other)) {
        BranchTo(then);
        return;
    }

    assert(!terminated && "attempting to add second terminator");

    llvm::IRBuilder<> irb(EndBlock());
    irb.CreateCondBr(cond, then.first_block, other.first_block);
    then.predecessors.push_back(this);
    other.predecessors.push_back(this);
    terminated = true;
}

bool BasicBlock::FillPhis() {
    if (empty_phis.empty())
        return false;

    for (auto& item : empty_phis) {
        LLReg reg = std::get<0>(item);
        Facet facet = std::get<1>(item);
        llvm::PHINode* phi = std::get<2>(item);
        for (BasicBlock* pred : predecessors) {
            assert(pred->terminated && "attempt to fill PHIs from open block");
            llvm::Value* value = pred->regfile.GetReg(reg, facet);
            phi->addIncoming(value, pred->EndBlock());
        }
    }
    empty_phis.clear();

    return true;
}

void BasicBlock::RemoveUnmodifiedStores(const BasicBlock& entry) {
    assert(entry.mem_ref_values.size() == mem_ref_values.size());

    // Remove stores to the CPU struct where the only possible value to be
    // stored is the value initially loaded from the struct.
    for (size_t i = 0; i < mem_ref_values.size(); i++) {
        if (mem_ref_values[i] == nullptr || !llvm::isa<llvm::StoreInst>(mem_ref_values[i]))
            continue;

        llvm::StoreInst* store = llvm::cast<llvm::StoreInst>(mem_ref_values[i]);
        llvm::LoadInst* load = llvm::cast<llvm::LoadInst>(entry.mem_ref_values[i]);

        llvm::Value* stored_val = store->getValueOperand();
        if (!llvm::isa<llvm::PHINode>(stored_val)) {
            if (stored_val == load)
                store->removeFromParent();
            continue;
        }

        // Follow PHI nodes
        std::set<llvm::PHINode*> visited_phis;
        std::deque<llvm::PHINode*> phis;
        phis.push_back(llvm::cast<llvm::PHINode>(stored_val));
        while (!phis.empty()) {
            llvm::PHINode* current_phi = phis.front();
            phis.pop_front();
            visited_phis.insert(current_phi);

            for (llvm::Value* incoming : current_phi->incoming_values()) {
                if (auto inc_phi = llvm::dyn_cast<llvm::PHINode>(incoming)) {
                    // Don't iterate twice over a PHI node.
                    if (visited_phis.count(inc_phi) == 0)
                        phis.push_back(inc_phi);
                } else if (incoming != load) {
                    // A different value than the load is in the PHI-graph, so
                    // do not replace.
                    goto next_store;
                }
            }
        }

        store->eraseFromParent();

next_store:
        (void) 0;
    }
}

} // namespace

/**
 * @}
 **/
