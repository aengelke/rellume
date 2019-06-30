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
#include <llsupport-internal.h>

/**
 * \defgroup LLBasicBlock Basic Block
 * \brief Representation of a basic block
 *
 * @{
 **/

namespace rellume
{

/**
 * Create a new basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param address The address of the basic block
 * \returns The new basic block
 **/
BasicBlock::BasicBlock(llvm::BasicBlock* llvm, LLState* state)
{
    this->state = state;
    this->llvmBB = llvm;
    nextBranch = NULL;
    nextFallThrough = NULL;
    endType = LL_INS_None;
    regfile = ll_regfile_new(llvm::wrap(llvmBB));
}

BasicBlock::~BasicBlock()
{
    ll_regfile_dispose(regfile);
}

void BasicBlock::SetCurrent()
{
    state->regfile = regfile;

    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(llvmBB);
}

void BasicBlock::AddPhis()
{
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(llvmBB);

    state->regfile = regfile;

    for (int i = 0; i < LL_RI_GPMax; i++)
    {
        for (size_t k = 0; k < FACET_COUNT; k++)
        {
            LLVMTypeRef ty = ll_register_facet_type((RegisterFacet) k, state->context);
            llvm::PHINode* phiNode = builder->CreatePHI(llvm::unwrap(ty), 0);

            ll_regfile_set(regfile, (RegisterFacet) k, ll_reg(LL_RT_GP64, i), llvm::wrap(phiNode), false, state->builder);
            phiGpRegs[i].facets[k] = phiNode;
        }
    }

    for (int i = 0; i < LL_RI_XMMMax; i++)
    {
        for (size_t k = 0; k < FACET_COUNT; k++)
        {
            LLVMTypeRef ty = ll_register_facet_type((RegisterFacet) k, state->context);
            llvm::PHINode* phiNode = builder->CreatePHI(llvm::unwrap(ty), 0);

            ll_regfile_set(regfile, (RegisterFacet) k, ll_reg(LL_RT_XMM, i), llvm::wrap(phiNode), false, state->builder);
            phiVRegs[i].facets[k] = phiNode;
        }
    }

    for (int i = 0; i < RFLAG_Max; i++)
    {
        llvm::PHINode* phiNode = builder->CreatePHI(builder->getInt1Ty(), 0);

        ll_regfile_set_flag(regfile, i, llvm::wrap(phiNode), state->context);
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
    state->regfile = regfile;

    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(llvmBB);

    // Set new instruction pointer register
    uintptr_t rip = instr->addr + instr->len;
    llvm::Value* ripValue = llvm::ConstantInt::get(builder->getInt64Ty(), rip);
    ll_set_register(ll_reg(LL_RT_IP, 0), FACET_I64, llvm::wrap(ripValue), true, state);

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
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(llvmBB);

    if (instrIsJcc(endType))
    {
        state->regfile = regfile;
        llvm::Value* cond = llvm::unwrap(ll_flags_condition(endType, LL_INS_JO, state));
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
    state->regfile = NULL;

    for (auto pred_it = preds.begin(); pred_it != preds.end(); ++pred_it)
    {
        BasicBlock* pred = *pred_it;

        for (int j = 0; j < LL_RI_GPMax; j++)
        {
            for (size_t k = 0; k < FACET_COUNT; k++)
            {
                llvm::Value* value = llvm::unwrap(ll_regfile_get(pred->regfile, (RegisterFacet)k, ll_reg(LL_RT_GP64, j), state->builder));
                phiGpRegs[j].facets[k]->addIncoming(value, pred->llvmBB);
            }
        }

        for (int j = 0; j < LL_RI_XMMMax; j++)
        {
            for (size_t k = 0; k < FACET_COUNT; k++)
            {
                llvm::Value* value = llvm::unwrap(ll_regfile_get(pred->regfile, (RegisterFacet)k, ll_reg(LL_RT_XMM, j), state->builder));
                phiVRegs[j].facets[k]->addIncoming(value, pred->llvmBB);
            }
        }

        for (int j = 0; j < RFLAG_Max; j++)
        {
            llvm::Value* value = llvm::unwrap(ll_regfile_get_flag(pred->regfile, j));
            phiFlags[j]->addIncoming(value, pred->llvmBB);
        }
    }
}

} // namespace


LLBasicBlock*
ll_basic_block_new(LLVMBasicBlockRef llvm, LLState* state)
{
    return reinterpret_cast<LLBasicBlock*>(new rellume::BasicBlock(llvm::unwrap(llvm), state));
}

void
ll_basic_block_dispose(LLBasicBlock* bb)
{
    delete reinterpret_cast<rellume::BasicBlock*>(bb);
}
void
ll_basic_block_set_current(LLBasicBlock* bb)
{
    reinterpret_cast<rellume::BasicBlock*>(bb)->SetCurrent();
}
void
ll_basic_block_add_phis(LLBasicBlock* bb)
{
    reinterpret_cast<rellume::BasicBlock*>(bb)->AddPhis();
}
void
ll_basic_block_terminate(LLBasicBlock* bb)
{
    reinterpret_cast<rellume::BasicBlock*>(bb)->Terminate();
}
void
ll_basic_block_fill_phis(LLBasicBlock* bb)
{
    reinterpret_cast<rellume::BasicBlock*>(bb)->FillPhis();
}

void
ll_basic_block_add_branches(LLBasicBlock* bb, LLBasicBlock* a, LLBasicBlock* b)
{
    reinterpret_cast<rellume::BasicBlock*>(bb)->AddBranches(reinterpret_cast<rellume::BasicBlock*>(a), reinterpret_cast<rellume::BasicBlock*>(b));
}
void
ll_basic_block_add_inst(LLBasicBlock* bb, LLInstr* instr)
{
    reinterpret_cast<rellume::BasicBlock*>(bb)->AddInst(instr);
}

/**
 * @}
 **/
