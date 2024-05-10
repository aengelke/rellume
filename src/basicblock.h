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

class ArchBasicBlock {
public:
    ArchBasicBlock(llvm::Function* fn, size_t max_preds);

    ArchBasicBlock(ArchBasicBlock&& rhs);
    ArchBasicBlock& operator=(ArchBasicBlock&& rhs);

    ArchBasicBlock(const ArchBasicBlock&) = delete;
    ArchBasicBlock& operator=(const ArchBasicBlock&) = delete;

    void BranchTo(ArchBasicBlock& next);
    void BranchTo(llvm::Value* cond, ArchBasicBlock& then, ArchBasicBlock& other);
    bool FillPhis();

    void InitEmpty(Arch arch, llvm::BasicBlock* bb) {
        regfile = std::make_unique<RegFile>(arch, bb);
    }
    void InitWithPHIs(Arch arch, bool seal = false);
    std::unique_ptr<RegFile> TakeRegFile() {
        return std::move(regfile);
    }
    RegFile* GetRegFile() {
        return regfile.get();
    }

    const std::vector<ArchBasicBlock*>& Predecessors() const {
        return predecessors;
    }
    const std::vector<ArchBasicBlock*>& Successors() const {
        return successors;
    }

    llvm::BasicBlock* BeginBlock() {
        return llvm_block;
    }
    llvm::BasicBlock* EndBlock() {
        return regfile->GetInsertBlock();
    }

private:
    /// First LLVM basic block for the x86 basic block.
    llvm::BasicBlock* llvm_block;

    /// The register file for the basic block
    std::unique_ptr<RegFile> regfile;

    size_t max_preds;
    std::vector<ArchBasicBlock*> predecessors;
    std::vector<ArchBasicBlock*> successors;
    std::vector<std::tuple<ArchReg, Facet, llvm::PHINode*>> empty_phis;
};

}

#endif
