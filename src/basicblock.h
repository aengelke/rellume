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

#include "facet.h"
#include "regfile.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <tuple>
#include <vector>


namespace rellume {

class FunctionInfo;
class LLConfig;

class BasicBlock
{
public:
    enum Kind {
        DEFAULT, ENTRY, EXIT
    };
    BasicBlock(FunctionInfo& fi, const LLConfig& cfg, Kind kind = DEFAULT);

    BasicBlock(BasicBlock&& rhs);
    BasicBlock& operator=(BasicBlock&& rhs);

    BasicBlock(const BasicBlock&) = delete;
    BasicBlock& operator=(const BasicBlock&) = delete;

    void BranchTo(BasicBlock& next);
    void BranchTo(llvm::Value* cond, BasicBlock& then, BasicBlock& other);
    bool FillPhis();

    llvm::Value* NextRip() {
        return regfile.GetReg(X86Reg::IP, Facet::I64);
    }

    bool IsTerminated() {
        return !!llvm_block->getTerminator();
    }

    RegFile* GetRegFile() {
        return &regfile;
    }

private:
    /// First LLVM basic block for the x86 basic block.
    llvm::BasicBlock* llvm_block;

    /// The register file for the basic block
    RegFile regfile;

    std::vector<BasicBlock*> predecessors;
    std::vector<std::tuple<X86Reg, Facet, llvm::PHINode*>> empty_phis;
};

class ArchBasicBlock
{
private:
    FunctionInfo& fi;
    const LLConfig& cfg;

    std::vector<std::unique_ptr<BasicBlock>> low_blocks;
    BasicBlock* insert_block;

public:
    ArchBasicBlock(FunctionInfo& fi, const LLConfig& cfg,
                   BasicBlock::Kind kind = BasicBlock::DEFAULT)
            : fi(fi), cfg(cfg) {
        low_blocks.push_back(std::make_unique<BasicBlock>(fi, cfg, kind));
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
    BasicBlock* AddBlock() {
        low_blocks.push_back(std::make_unique<BasicBlock>(fi, cfg, BasicBlock::DEFAULT));
        return low_blocks[low_blocks.size()-1].get();
    }
    BasicBlock* GetInsertBlock() {
        return insert_block;
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

    bool IsTerminated() {
        return insert_block->IsTerminated();
    }
    llvm::Value* NextRip() {
        return insert_block->NextRip();
    }
};

}

#endif
