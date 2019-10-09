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

#ifndef LL_BASIC_BLOCK_H
#define LL_BASIC_BLOCK_H

#include "config.h"
#include "facet.h"
#include "lifter.h"
#include "regfile.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <tuple>
#include <vector>


namespace rellume {

class BasicBlock
{
public:
    enum Kind {
        DEFAULT, ENTRY, EXIT
    };
    BasicBlock(llvm::Function* fn, const LLConfig& cfg, Kind kind = DEFAULT);

    BasicBlock(BasicBlock&& rhs);
    BasicBlock& operator=(BasicBlock&& rhs);

    BasicBlock(const BasicBlock&) = delete;
    BasicBlock& operator=(const BasicBlock&) = delete;

    void AddInst(const LLInstr& inst, const LLConfig& cfg) {
        Lifter state(cfg, regfile);
        state.Lift(inst);
    }
    void BranchTo(BasicBlock& next);
    void BranchTo(llvm::Value* cond, BasicBlock& then, BasicBlock& other);
    bool FillPhis();

    void RemoveUnmodifiedStores(const BasicBlock& entry);

    llvm::Value* NextRip() {
        return regfile.GetReg(LLReg(LL_RT_IP, 0), Facet::I64);
    }

    bool IsTerminated() {
        return terminated;
    }

private:
    llvm::BasicBlock* EndBlock() {
        // The ending block is the last insertion point.
        return regfile.GetInsertBlock();
    }

    /// First LLVM basic block for the x86 basic block.
    llvm::BasicBlock* first_block;

    /// The register file for the basic block
    RegFile regfile;

    std::vector<BasicBlock*> predecessors;
    std::vector<std::tuple<LLReg, Facet, llvm::PHINode*>> empty_phis;

    // Stores load/store instructions for CPU struct access in entry/exit blocks
    std::vector<llvm::Value*> mem_ref_values;

    bool terminated = false;
};

class ArchBasicBlock
{
private:
    llvm::Function* fn;
    const LLConfig& cfg;

    std::vector<std::unique_ptr<BasicBlock>> low_blocks;
    BasicBlock* insert_block;

public:
    ArchBasicBlock(llvm::Function* fn, const LLConfig& cfg,
                   BasicBlock::Kind kind = BasicBlock::DEFAULT)
            : fn(fn), cfg(cfg) {
        low_blocks.push_back(std::make_unique<BasicBlock>(fn, cfg, kind));
        insert_block = low_blocks[0].get();
    }

    ArchBasicBlock(ArchBasicBlock&& rhs);
    ArchBasicBlock& operator=(ArchBasicBlock&& rhs);

    ArchBasicBlock(const ArchBasicBlock&) = delete;
    ArchBasicBlock& operator=(const ArchBasicBlock&) = delete;

private:
    BasicBlock& BeginBlock() {
        return *low_blocks[0];
    }

public:
    void AddInst(const LLInstr& inst) {
        insert_block->AddInst(inst, cfg);
    }

    BasicBlock* AddBlock() {
        low_blocks.push_back(std::make_unique<BasicBlock>(fn, cfg, BasicBlock::DEFAULT));
        return low_blocks[low_blocks.size()-1].get();
    }
    void SetInsertBlock(BasicBlock* new_insert_block) {
        insert_block = new_insert_block;
    }

    void BranchTo(ArchBasicBlock& next) {
        insert_block->BranchTo(next.BeginBlock());
    }
    void BranchTo(llvm::Value* cond, ArchBasicBlock& then, ArchBasicBlock& other) {
        insert_block->BranchTo(cond, then.BeginBlock(), other.BeginBlock());
    }
    bool FillPhis() {
        bool res = false;
        for (const auto& lb : low_blocks)
            res |= lb->FillPhis();
        return res;
    }

    void RemoveUnmodifiedStores(ArchBasicBlock& entry) {
        insert_block->RemoveUnmodifiedStores(entry.BeginBlock());
    }

    llvm::Value* NextRip() {
        return insert_block->NextRip();
    }
};

}

#endif
