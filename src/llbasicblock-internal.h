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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include <llvm/IR/BasicBlock.h>
#include <llvm-c/Core.h>

#include <llcommon-internal.h>
#include <llregfile-internal.h>
#include <llstate-internal.h>


namespace rellume
{

class BasicBlock
{
public:
    BasicBlock(llvm::BasicBlock* block, LLState& state);
    ~BasicBlock();

    BasicBlock(BasicBlock&& rhs);
    BasicBlock& operator=(BasicBlock&& rhs);

    BasicBlock(const BasicBlock&) = delete;
    BasicBlock& operator=(const BasicBlock&) = delete;

    void SetCurrent();
    void AddPhis();
    void AddInst(LLInstr* inst);
    void AddBranches(BasicBlock*, BasicBlock*);
    void Terminate();
    void FillPhis();

private:
    LLState& state;

    /// The branch basic block, or NULL
    BasicBlock* nextBranch;
    /// The fall-through basic block, or NULL
    BasicBlock* nextFallThrough;

    /// Preceding basic blocks
    std::vector<BasicBlock*> preds;

    /// The LLVM basic block
    llvm::BasicBlock* llvmBB;

    /// The register file for the basic block
    LLRegisterFile* regfile;

    struct RegisterPhis {
        llvm::PHINode* facets[FACET_COUNT];
    };
    /// The phi nodes for the registers
    RegisterPhis phiGpRegs[LL_RI_GPMax];
    /// The phi nodes for the registers
    RegisterPhis phiVRegs[LL_RI_XMMMax];

    /// The phi nodes for the flags
    llvm::PHINode* phiFlags[RFLAG_Max];

    LLInstrType endType;
};

}

#endif
