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
#include "function-info.h"
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

BasicBlock::BasicBlock(FunctionInfo& fi, const LLConfig& cfg, Kind kind)
        : regfile() {
    llvm_block = llvm::BasicBlock::Create(fi.fn->getContext(), "", fi.fn, nullptr);
    regfile.SetInsertBlock(llvm_block);

    if (kind != ENTRY) {
        // Initialize all registers with a generator which adds a PHI node when
        // the value-facet combination is requested.
        regfile.InitAll([this](const X86Reg reg, const Facet facet) {
            return DeferredValue([](X86Reg reg, Facet facet, llvm::BasicBlock* bb, void** user_args) {
                llvm::IRBuilder<> irb(bb, bb->begin());
                auto phi = irb.CreatePHI(facet.Type(irb.getContext()), 4);
                auto self = static_cast<BasicBlock*>(user_args[0]);
                self->empty_phis.push_back(std::make_tuple(reg, facet, phi));
                return llvm::cast<llvm::Value>(phi);
            }, {this});
        });
    }

    // For ENTRY or EXIT kinds, we either need to setup all values or store them
    // back to memory.
    if (kind == ENTRY) {
        regfile.Clear();
        llvm::IRBuilder<> irb(llvm_block);
        fi.InitSptr(irb);
        cfg.callconv.Unpack(regfile, fi);
    } else if (kind == EXIT) {
        llvm::Value* ret_val = cfg.callconv.Pack(regfile, fi);

        llvm::IRBuilder<> irb(llvm_block);
        if (ret_val == nullptr)
            irb.CreateRetVoid();
        else
            irb.CreateRet(ret_val);
    }
}

void BasicBlock::BranchTo(BasicBlock& next) {
    assert(!IsTerminated() && "attempting to add second terminator");

    llvm::IRBuilder<> irb(llvm_block);
    irb.CreateBr(next.llvm_block);
    next.predecessors.push_back(this);
}

void BasicBlock::BranchTo(llvm::Value* cond, BasicBlock& then,
                             BasicBlock& other) {
    // In case both blocks are the same create a single branch only.
    if (std::addressof(then) == std::addressof(other)) {
        BranchTo(then);
        return;
    }

    assert(!IsTerminated() && "attempting to add second terminator");

    llvm::IRBuilder<> irb(llvm_block);
    irb.CreateCondBr(cond, then.llvm_block, other.llvm_block);
    then.predecessors.push_back(this);
    other.predecessors.push_back(this);
}

bool BasicBlock::FillPhis() {
    if (empty_phis.empty())
        return false;

    for (auto& item : empty_phis) {
        X86Reg reg = std::get<0>(item);
        Facet facet = std::get<1>(item);
        llvm::PHINode* phi = std::get<2>(item);
        for (BasicBlock* pred : predecessors) {
            assert(pred->IsTerminated() && "attempt to fill PHIs from open block");
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
