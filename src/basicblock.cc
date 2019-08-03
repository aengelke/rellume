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
#include "lifter.h"
#include "regfile.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Metadata.h>
#include <cstdio>
#include <vector>



/**
 * \defgroup LLBasicBlock Basic Block
 * \brief Representation of a basic block
 *
 * @{
 **/

namespace rellume {

BasicBlock::BasicBlock(llvm::BasicBlock* llvm) : llvmBB(llvm), regfile(llvm) {
    // Initialize all registers with a generator which adds a PHI node when the
    // value-facet combination is requested.
    regfile.InitAll([this](const LLReg reg, const Facet facet) {
        return [this, reg, facet]() {
            llvm::IRBuilder<> irb(llvmBB, llvmBB->begin());
            llvm::PHINode* phi = irb.CreatePHI(facet.Type(irb.getContext()), 4);
            empty_phis.push_back(std::make_tuple(reg, facet, phi));
            return phi;
        };
    });
}

void BasicBlock::AddInst(const LLInstr& inst, LLConfig& cfg)
{
    Lifter state(cfg, regfile, llvmBB);

    // Set new instruction pointer register
    llvm::Value* ripValue = state.irb.getInt64(inst.addr + inst.len);
    regfile.SetReg(LLReg(LL_RT_IP, 0), Facet::I64, ripValue, true);

    // Add separator for debugging.
    llvm::Function* intrinsicDoNothing = llvm::Intrinsic::getDeclaration(llvmBB->getModule(), llvm::Intrinsic::donothing, {});
    state.irb.CreateCall(intrinsicDoNothing);

    switch (inst.type)
    {
#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include "rellume/opcodes.inc"
#undef DEF_IT

        default:
            printf("Could not handle instruction at %#zx\n", inst.addr);
            assert(0);
            break;
    }
}

void BasicBlock::BranchTo(BasicBlock& next) {
    llvm::IRBuilder<> irb(llvmBB);
    irb.CreateBr(next.llvmBB);
    next.predecessors.push_back(this);
}

void BasicBlock::BranchTo(llvm::Value* cond, BasicBlock& then,
                             BasicBlock& other) {
    // In case both blocks are the same create a single branch only.
    if (std::addressof(then) == std::addressof(other)) {
        BranchTo(then);
        return;
    }

    llvm::IRBuilder<> irb(llvmBB);
    irb.CreateCondBr(cond, then.llvmBB, other.llvmBB);
    then.predecessors.push_back(this);
    other.predecessors.push_back(this);
}

bool BasicBlock::FillPhis() {
    if (empty_phis.empty())
        return false;

    for (auto& item : empty_phis) {
        LLReg reg = std::get<0>(item);
        Facet facet = std::get<1>(item);
        llvm::PHINode* phi = std::get<2>(item);
        for (BasicBlock* pred : predecessors) {
            llvm::Value* value = pred->regfile.GetReg(reg, facet);
            phi->addIncoming(value, pred->llvmBB);
        }
    }
    empty_phis.clear();

    return true;
}

} // namespace

/**
 * @}
 **/
