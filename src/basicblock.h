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
#include <tuple>
#include <vector>


namespace rellume {

class BasicBlock
{
public:
    BasicBlock(llvm::BasicBlock* llvm);

    BasicBlock(BasicBlock&& rhs);
    BasicBlock& operator=(BasicBlock&& rhs);

    BasicBlock(const BasicBlock&) = delete;
    BasicBlock& operator=(const BasicBlock&) = delete;

    void AddInst(const LLInstr& inst, LLConfig& cfg);
    void AddToPhis(BasicBlock& pred) {
        AddToPhis(pred.llvmBB, pred.regfile);
    }
    void AddToPhis(llvm::BasicBlock* pred, RegFile& pred_rf);
    bool FillPhis();

    llvm::BasicBlock* Llvm() {
        return llvmBB;
    }

private:
    /// The LLVM basic block
    llvm::BasicBlock* llvmBB;

public:
    /// The register file for the basic block
    RegFile regfile;

private:
    std::vector<std::pair<llvm::BasicBlock*, RegFile&>> predecessors;
    std::vector<std::tuple<LLReg, Facet, llvm::PHINode*>> empty_phis;

    /// The phi nodes for the flags
    llvm::PHINode* phiFlags[RFLAG_Max];
};

}

#endif
