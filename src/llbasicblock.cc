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

#include "llbasicblock-internal.h"

#include "config.h"
#include "facet.h"
#include "llregfile-internal.h"
#include "llstate-internal.h"
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

namespace rellume
{

BasicBlock::BasicBlock(llvm::BasicBlock* llvm) : llvmBB(llvm), regfile(llvm) {
    llvm::IRBuilder<> irb(llvm);

    phi_rip = irb.CreatePHI(irb.getInt64Ty(), 0);
    regfile.SetReg(LLReg(LL_RT_IP, 0), Facet::I64, phi_rip, true);

    for (int i = 0; i < LL_RI_GPMax; i++)
    {
        for (Facet facet : phis_gp[i].facets())
        {
            llvm::Type* ty = facet.Type(irb.getContext());
            llvm::PHINode* phiNode = irb.CreatePHI(ty, 0);

            regfile.SetReg(LLReg(LL_RT_GP64, i), facet, phiNode, false);
            phis_gp[i].at(facet) = phiNode;
        }
    }

    for (int i = 0; i < LL_RI_XMMMax; i++)
    {
        for (Facet facet : phis_sse[i].facets())
        {
            llvm::Type* ty = facet.Type(irb.getContext());
            llvm::PHINode* phiNode = irb.CreatePHI(ty, 0);

            regfile.SetReg(LLReg(LL_RT_XMM, i), facet, phiNode, false);
            phis_sse[i].at(facet) = phiNode;
        }
    }

    for (int i = 0; i < RFLAG_Max; i++)
    {
        llvm::PHINode* phiNode = irb.CreatePHI(irb.getInt1Ty(), 0);

        regfile.SetFlag(i, phiNode);
        phiFlags[i] = phiNode;
    }
}

void BasicBlock::AddInst(LLInstr* instr, LLConfig& cfg)
{
    LLState state(cfg, regfile, llvmBB);

    // Set new instruction pointer register
    llvm::Value* ripValue = state.irb.getInt64(instr->addr + instr->len);
    regfile.SetReg(LLReg(LL_RT_IP, 0), Facet::I64, ripValue, true);

    // Add separator for debugging.
    llvm::Function* intrinsicDoNothing = llvm::Intrinsic::getDeclaration(llvmBB->getModule(), llvm::Intrinsic::donothing, {});
    state.irb.CreateCall(intrinsicDoNothing);

    switch (instr->type)
    {
#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include "rellume/opcodes.inc"
#undef DEF_IT

        default:
            printf("Could not handle instruction at %#zx\n", instr->addr);
            assert(0);
            break;
    }
}

void BasicBlock::AddToPhis(llvm::BasicBlock* pred, RegFile& pred_rf)
{
    phi_rip->addIncoming(pred_rf.GetReg(LLReg(LL_RT_IP, 0), Facet::I64), pred);

    for (int j = 0; j < LL_RI_GPMax; j++)
    {
        for (Facet facet : phis_gp[j].facets())
        {
            auto phi = llvm::cast<llvm::PHINode>(phis_gp[j].at(facet));
            llvm::Value* value = pred_rf.GetReg(LLReg(LL_RT_GP64, j), facet);
            phi->addIncoming(value, pred);
        }
    }

    for (int j = 0; j < LL_RI_XMMMax; j++)
    {
        for (Facet facet : phis_sse[j].facets())
        {
            auto phi = llvm::cast<llvm::PHINode>(phis_sse[j].at(facet));
            llvm::Value* value = pred_rf.GetReg(LLReg(LL_RT_XMM, j), facet);
            phi->addIncoming(value, pred);
        }
    }

    for (int j = 0; j < RFLAG_Max; j++)
    {
        llvm::Value* value = pred_rf.GetFlag(j);
        phiFlags[j]->addIncoming(value, pred);
    }
}

} // namespace



/**
 * @}
 **/
