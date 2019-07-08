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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Metadata.h>
#include <llvm-c/Core.h>

#include <llbasicblock-internal.h>

#include <llcommon-internal.h>
#include <llflags-internal.h>
#include <llinstr-internal.h>
#include <llinstruction-internal.h>
#include <llregfile-internal.h>

/**
 * \defgroup LLBasicBlock Basic Block
 * \brief Representation of a basic block
 *
 * @{
 **/

namespace rellume
{

void BasicBlock::AddPhis()
{
    llvm::IRBuilder<>* builder = llvm::unwrap(state.builder);
    SetCurrent();

    for (int i = 0; i < LL_RI_GPMax; i++)
    {
        for (auto facet : phis_gp[i].facets())
        {
            llvm::Type* ty = Facet::Type(facet, state.irb.getContext());
            llvm::PHINode* phiNode = builder->CreatePHI(ty, 0);

            regfile.SetReg(LLReg(LL_RT_GP64, i), facet, phiNode, false);
            phis_gp[i].at(facet) = phiNode;
        }
    }

    for (int i = 0; i < LL_RI_XMMMax; i++)
    {
        for (auto facet : phis_sse[i].facets())
        {
            llvm::Type* ty = Facet::Type(facet, state.irb.getContext());
            llvm::PHINode* phiNode = builder->CreatePHI(ty, 0);

            regfile.SetReg(LLReg(LL_RT_XMM, i), facet, phiNode, false);
            phis_sse[i].at(facet) = phiNode;
        }
    }

    for (int i = 0; i < RFLAG_Max; i++)
    {
        llvm::PHINode* phiNode = builder->CreatePHI(builder->getInt1Ty(), 0);

        regfile.SetFlag(i, phiNode);
        phiFlags[i] = phiNode;
    }
}

/**
 * Add branches to the basic block. This also registers them as predecessors.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param branch The active branch, or NULL
 * \param fallThrough The fall-through branch, or NULL
 **/
void BasicBlock::AddBranches(BasicBlock* branch, BasicBlock* fallThrough)
{
    if (branch != NULL)
    {
        branch->preds.push_back(this);
        nextBranch = branch;
    }

    if (fallThrough != NULL)
    {
        fallThrough->preds.push_back(this);
        nextFallThrough = fallThrough;
    }
}

#define instrIsJcc(instr) ( \
    (instr) == LL_INS_JO || \
    (instr) == LL_INS_JNO || \
    (instr) == LL_INS_JC || \
    (instr) == LL_INS_JNC || \
    (instr) == LL_INS_JZ || \
    (instr) == LL_INS_JNZ || \
    (instr) == LL_INS_JBE || \
    (instr) == LL_INS_JA || \
    (instr) == LL_INS_JS || \
    (instr) == LL_INS_JNS || \
    (instr) == LL_INS_JP || \
    (instr) == LL_INS_JNP || \
    (instr) == LL_INS_JL || \
    (instr) == LL_INS_JGE || \
    (instr) == LL_INS_JLE || \
    (instr) == LL_INS_JG \
)

void BasicBlock::AddInst(LLInstr* instr)
{
    SetCurrent();

    llvm::IRBuilder<>* builder = llvm::unwrap(state.builder);

    // Set new instruction pointer register
    uintptr_t rip = instr->addr + instr->len;
    llvm::Value* ripValue = llvm::ConstantInt::get(builder->getInt64Ty(), rip);
    regfile.SetReg(LLReg(LL_RT_IP, 0), Facet::I64, ripValue, true);

    // Add separator for debugging.
    llvm::Function* intrinsicDoNothing = llvm::Intrinsic::getDeclaration(llvmBB->getModule(), llvm::Intrinsic::donothing, {});
    builder->CreateCall(intrinsicDoNothing);

    switch (instr->type)
    {
#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include "rellume/opcodes.inc"
#undef DEF_IT

        default:
            printf("Could not handle instruction at %#zx\n", instr->addr);
            warn_if_reached();
            break;
    }

    endType = instr->type;
}

/**
 * Build the LLVM IR.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param state The module state
 **/
void
BasicBlock::Terminate()
{
    SetCurrent();

    llvm::IRBuilder<>* builder = llvm::unwrap(state.builder);

    if (instrIsJcc(endType))
    {
        llvm::Value* cond = state.FlagCond(endType, LL_INS_JO);
        builder->CreateCondBr(cond, nextBranch->llvmBB, nextFallThrough->llvmBB);
    }
    else if (endType == LL_INS_JMP)
        builder->CreateBr(nextBranch->llvmBB);
    else if (endType != LL_INS_RET && endType != LL_INS_Invalid) // Any other instruction which is not a terminator
        builder->CreateBr(nextFallThrough->llvmBB);
}

/**
 * Fill PHI nodes after the IR for all basic blocks of the function is
 * generated.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 **/
void BasicBlock::FillPhis()
{
    state.regfile = NULL;

    for (auto pred_it = preds.begin(); pred_it != preds.end(); ++pred_it)
    {
        BasicBlock* pred = *pred_it;

        for (int j = 0; j < LL_RI_GPMax; j++)
        {
            for (auto facet : phis_gp[j].facets())
            {
                auto phi = llvm::cast<llvm::PHINode>(phis_gp[j].at(facet));
                llvm::Value* value = pred->regfile.GetReg(LLReg(LL_RT_GP64, j), facet);
                phi->addIncoming(value, pred->llvmBB);
            }
        }

        for (int j = 0; j < LL_RI_XMMMax; j++)
        {
            for (auto facet : phis_sse[j].facets())
            {
                auto phi = llvm::cast<llvm::PHINode>(phis_sse[j].at(facet));
                llvm::Value* value = pred->regfile.GetReg(LLReg(LL_RT_XMM, j), facet);
                phi->addIncoming(value, pred->llvmBB);
            }
        }

        for (int j = 0; j < RFLAG_Max; j++)
        {
            llvm::Value* value = pred->regfile.GetFlag(j);
            phiFlags[j]->addIncoming(value, pred->llvmBB);
        }
    }
}

} // namespace



/**
 * @}
 **/
