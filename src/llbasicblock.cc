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

struct LLRegister {
    llvm::PHINode* facets[FACET_COUNT];
};

typedef struct LLRegister LLRegister;

struct LLBasicBlock {
    LLState* state;

    /**
     * \brief The branch basic block, or NULL
     **/
    LLBasicBlock* nextBranch;
    /**
     * \brief The fall-through basic block, or NULL
     **/
    LLBasicBlock* nextFallThrough;

    /**
     * \brief Preceding basic blocks
     **/
    std::vector<LLBasicBlock*> preds;

    /**
     * \brief The LLVM basic block
     **/
    llvm::BasicBlock* llvmBB;

    /**
     * \brief The register file for the basic block
     **/
    LLRegisterFile* regfile;

    /**
     * \brief The phi nodes for the registers
     **/
    LLRegister phiGpRegs[LL_RI_GPMax];

    /**
     * \brief The phi nodes for the registers
     **/
    LLRegister phiVRegs[LL_RI_XMMMax];

    /**
     * \brief The phi nodes for the flags
     **/
    llvm::PHINode* phiFlags[RFLAG_Max];

    LLInstrType endType;
};

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
LLBasicBlock*
ll_basic_block_new(LLVMBasicBlockRef llvmBB, LLState* state)
{
    LLBasicBlock* bb;

    bb = (LLBasicBlock*) calloc(sizeof(LLBasicBlock), 1);
    bb->state = state;
    bb->llvmBB = llvm::unwrap(llvmBB);
    bb->nextBranch = NULL;
    bb->nextFallThrough = NULL;
    bb->endType = LL_INS_None;
    bb->regfile = ll_regfile_new(llvmBB);

    return bb;
}

void
ll_basic_block_set_current(LLBasicBlock* bb)
{
    bb->state->regfile = bb->regfile;

    llvm::IRBuilder<>* builder = llvm::unwrap(bb->state->builder);
    builder->SetInsertPoint(bb->llvmBB);
}

void
ll_basic_block_add_phis(LLBasicBlock* bb)
{
    LLState* state = bb->state;

    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(bb->llvmBB);

    state->regfile = bb->regfile;

    for (int i = 0; i < LL_RI_GPMax; i++)
    {
        for (size_t k = 0; k < FACET_COUNT; k++)
        {
            LLVMTypeRef ty = ll_register_facet_type((RegisterFacet) k, state->context);
            llvm::PHINode* phiNode = builder->CreatePHI(llvm::unwrap(ty), 0);

            ll_regfile_set(bb->regfile, (RegisterFacet) k, ll_reg(LL_RT_GP64, i), llvm::wrap(phiNode), false, state->builder);
            bb->phiGpRegs[i].facets[k] = phiNode;
        }
    }

    for (int i = 0; i < LL_RI_XMMMax; i++)
    {
        for (size_t k = 0; k < FACET_COUNT; k++)
        {
            LLVMTypeRef ty = ll_register_facet_type((RegisterFacet) k, state->context);
            llvm::PHINode* phiNode = builder->CreatePHI(llvm::unwrap(ty), 0);

            ll_regfile_set(bb->regfile, (RegisterFacet) k, ll_reg(LL_RT_XMM, i), llvm::wrap(phiNode), false, state->builder);
            bb->phiVRegs[i].facets[k] = phiNode;
        }
    }

    for (int i = 0; i < RFLAG_Max; i++)
    {
        llvm::PHINode* phiNode = builder->CreatePHI(builder->getInt1Ty(), 0);

        ll_regfile_set_flag(bb->regfile, i, llvm::wrap(phiNode), state->context);
        bb->phiFlags[i] = phiNode;
    }
}

/**
 * Dispose a basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 **/
void
ll_basic_block_dispose(LLBasicBlock* bb)
{
    ll_regfile_dispose(bb->regfile);

    bb->preds.~vector();

    free(bb);
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
void
ll_basic_block_add_branches(LLBasicBlock* bb, LLBasicBlock* branch, LLBasicBlock* fallThrough)
{
    if (branch != NULL)
    {
        branch->preds.push_back(bb);
        bb->nextBranch = branch;
    }

    if (fallThrough != NULL)
    {
        fallThrough->preds.push_back(bb);
        bb->nextFallThrough = fallThrough;
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

void
ll_basic_block_add_inst(LLBasicBlock* bb, LLInstr* instr)
{
    LLState* state = bb->state;
    state->regfile = bb->regfile;

    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(bb->llvmBB);

    // Set new instruction pointer register
    uintptr_t rip = instr->addr + instr->len;
    llvm::Value* ripValue = llvm::ConstantInt::get(builder->getInt64Ty(), rip);
    ll_set_register(ll_reg(LL_RT_IP, 0), FACET_I64, llvm::wrap(ripValue), true, state);

    // Add separator for debugging.
    llvm::Function* intrinsicDoNothing = llvm::Intrinsic::getDeclaration(bb->llvmBB->getModule(), llvm::Intrinsic::donothing, {});
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

    bb->endType = instr->type;
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
ll_basic_block_terminate(LLBasicBlock* bb)
{
    LLState* state = bb->state;
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    builder->SetInsertPoint(bb->llvmBB);

    LLInstrType endType = bb->endType;
    if (instrIsJcc(endType))
    {
        state->regfile = bb->regfile;
        llvm::Value* cond = llvm::unwrap(ll_flags_condition(endType, LL_INS_JO, state));
        builder->CreateCondBr(cond, bb->nextBranch->llvmBB, bb->nextFallThrough->llvmBB);
    }
    else if (endType == LL_INS_JMP)
        builder->CreateBr(bb->nextBranch->llvmBB);
    else if (endType != LL_INS_RET && endType != LL_INS_Invalid) // Any other instruction which is not a terminator
        builder->CreateBr(bb->nextFallThrough->llvmBB);
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
void
ll_basic_block_fill_phis(LLBasicBlock* bb)
{
    LLState* state = bb->state;
    state->regfile = NULL;

    for (auto pred_it = bb->preds.begin(); pred_it != bb->preds.end(); ++pred_it)
    {
        LLBasicBlock* pred = *pred_it;

        for (int j = 0; j < LL_RI_GPMax; j++)
        {
            for (size_t k = 0; k < FACET_COUNT; k++)
            {
                llvm::Value* value = llvm::unwrap(ll_regfile_get(pred->regfile, (RegisterFacet)k, ll_reg(LL_RT_GP64, j), state->builder));
                bb->phiGpRegs[j].facets[k]->addIncoming(value, pred->llvmBB);
            }
        }

        for (int j = 0; j < LL_RI_XMMMax; j++)
        {
            for (size_t k = 0; k < FACET_COUNT; k++)
            {
                llvm::Value* value = llvm::unwrap(ll_regfile_get(pred->regfile, (RegisterFacet)k, ll_reg(LL_RT_XMM, j), state->builder));
                bb->phiVRegs[j].facets[k]->addIncoming(value, pred->llvmBB);
            }
        }

        for (int j = 0; j < RFLAG_Max; j++)
        {
            llvm::Value* value = llvm::unwrap(ll_regfile_get_flag(pred->regfile, j));
            bb->phiFlags[j]->addIncoming(value, pred->llvmBB);
        }
    }
}

/**
 * @}
 **/
