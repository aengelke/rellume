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
    BasicBlock(llvm::Function* fn, Kind kind = DEFAULT,
               llvm::Value* mem_arg = nullptr);

    BasicBlock(BasicBlock&& rhs);
    BasicBlock& operator=(BasicBlock&& rhs);

    BasicBlock(const BasicBlock&) = delete;
    BasicBlock& operator=(const BasicBlock&) = delete;

    void AddInst(const LLInstr& inst, const LLConfig& cfg);
    void BranchTo(BasicBlock& next);
    void BranchTo(llvm::Value* cond, BasicBlock& then, BasicBlock& other);
    bool FillPhis();

    void RemoveUnmodifiedStores(const BasicBlock& entry);

    llvm::Value* NextRip() {
        return regfile.GetReg(LLReg(LL_RT_IP, 0), Facet::I64);
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
};

}

#endif
