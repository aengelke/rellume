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

#include "arch.h"
#include "facet.h"
#include "regfile.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <tuple>
#include <vector>


namespace rellume {

class BasicBlock {
public:
    enum class Phis { NONE, NATIVE, ALL };

    BasicBlock(llvm::Function* fn, size_t max_preds);

    BasicBlock(BasicBlock&& rhs);
    BasicBlock& operator=(BasicBlock&& rhs);

    BasicBlock(const BasicBlock&) = delete;
    BasicBlock& operator=(const BasicBlock&) = delete;

    void BranchTo(BasicBlock& next);
    void BranchTo(llvm::Value* cond, BasicBlock& then, BasicBlock& other);
    bool FillPhis();

    void InitRegFile(Arch arch, Phis phi_mode);
    RegFile* GetRegFile() {
        return regfile.get();
    }

    const std::vector<BasicBlock*>& Predecessors() const {
        return predecessors;
    }
    const std::vector<BasicBlock*>& Successors() const {
        return successors;
    }

    operator llvm::BasicBlock*() const {
        return llvm_block;
    }

private:
    /// First LLVM basic block for the x86 basic block.
    llvm::BasicBlock* llvm_block;

    /// The register file for the basic block
    std::unique_ptr<RegFile> regfile;

    size_t max_preds;
    std::vector<BasicBlock*> predecessors;
    std::vector<BasicBlock*> successors;
    std::vector<std::tuple<ArchReg, Facet, llvm::PHINode*>> empty_phis;
};

class ArchBasicBlock
{
private:
    llvm::Function* fn;
    BasicBlock::Phis phi_mode;
    Arch arch;

    std::vector<std::unique_ptr<BasicBlock>> low_blocks;
    BasicBlock* insert_block;

public:
    ArchBasicBlock(llvm::Function* fn, BasicBlock::Phis phi_mode, Arch arch,
                   size_t max_preds)
            : fn(fn), phi_mode(phi_mode), arch(arch) {
        low_blocks.push_back(std::make_unique<BasicBlock>(fn, max_preds));
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
        low_blocks.push_back(std::make_unique<BasicBlock>(fn, SIZE_MAX));
        BasicBlock* res = low_blocks[low_blocks.size()-1].get();
        res->InitRegFile(arch, phi_mode);
        return res;
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
};

}

#endif
