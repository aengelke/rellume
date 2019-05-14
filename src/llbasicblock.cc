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

#include <llvm/IR/Instructions.h>
#include <llvm-c/Core.h>

#include <llbasicblock-internal.h>

#include <llcommon-internal.h>
#include <llflags-internal.h>
#include <llinstr-internal.h>
#include <llinstruction-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLBasicBlock Basic Block
 * \brief Representation of a basic block
 *
 * @{
 **/

struct LLRegister {
    LLVMValueRef facets[FACET_COUNT];
};

typedef struct LLRegister LLRegister;
// typedef LLVMValueRef LLRegister[FACET_COUNT];

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

    // Predecessors needed for phi nodes
    /**
     * \brief The predecessor count
     **/
    size_t predCount;
    /**
     * \brief The number predecessors allocated
     **/
    size_t predsAllocated;
    /**
     * \brief The preceding basic blocks
     **/
    LLBasicBlock** preds;

    /**
     * \brief The LLVM basic block
     **/
    LLVMBasicBlockRef llvmBB;

    /**
     * \brief The register file for the basic block
     **/
    LLRegisterFile* regfile;

    /**
     * \brief The phi nodes for the registers
     **/
    LLRegister phiNodesGpRegisters[LL_RI_GPMax];

    /**
     * \brief The phi nodes for the registers
     **/
    LLRegister phiNodesSseRegisters[LL_RI_XMMMax];

    /**
     * \brief The phi nodes for the flags
     **/
    LLVMValueRef phiNodesFlags[RFLAG_Max];

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

    bb = (LLBasicBlock*) malloc(sizeof(LLBasicBlock));
    bb->state = state;
    bb->llvmBB = llvmBB;
    bb->nextBranch = NULL;
    bb->nextFallThrough = NULL;
    bb->predCount = 0;
    bb->predsAllocated = 0;
    bb->endType = LL_INS_None;
    bb->regfile = ll_regfile_new(bb);

    return bb;
}

void
ll_basic_block_add_phis(LLBasicBlock* bb)
{
    LLVMValueRef phiNode;
    LLState* state = bb->state;

    state->currentBB = bb;
    LLVMPositionBuilderAtEnd(state->builder, bb->llvmBB);

    for (int i = 0; i < LL_RI_GPMax; i++)
    {
        for (size_t k = 0; k < FACET_COUNT; k++)
        {
            phiNode = LLVMBuildPhi(state->builder, ll_register_facet_type((RegisterFacet) k, state), "");

            ll_regfile_set(bb->regfile, (RegisterFacet) k, ll_reg(LL_RT_GP64, i), phiNode, false, state);
            bb->phiNodesGpRegisters[i].facets[k] = phiNode;
        }
    }

    for (int i = 0; i < LL_RI_XMMMax; i++)
    {
        for (size_t k = 0; k < FACET_COUNT; k++)
        {
            phiNode = LLVMBuildPhi(state->builder, ll_register_facet_type((RegisterFacet) k, state), "");

            ll_regfile_set(bb->regfile, (RegisterFacet) k, ll_reg(LL_RT_XMM, i), phiNode, false, state);
            bb->phiNodesSseRegisters[i].facets[k] = phiNode;
        }
    }

    for (int i = 0; i < RFLAG_Max; i++)
    {
        phiNode = LLVMBuildPhi(state->builder, LLVMInt1TypeInContext(state->context), "");

        ll_regfile_set_flag(bb->regfile, i, phiNode);
        bb->phiNodesFlags[i] = phiNode;
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

    if (bb->predsAllocated != 0)
        free(bb->preds);

    free(bb);
}

/**
 * Add a predecessor.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param pred The preceding basic block
 **/
void
ll_basic_block_add_predecessor(LLBasicBlock* bb, LLBasicBlock* pred)
{
    if (bb->predsAllocated == 0)
    {
        bb->preds = (LLBasicBlock**) malloc(sizeof(LLBasicBlock*) * 10);
        bb->predsAllocated = 10;

        if (bb->preds == NULL)
            warn_if_reached();
    }
    else if (bb->predsAllocated == bb->predCount)
    {
        bb->preds = (LLBasicBlock**) realloc(bb->preds, sizeof(LLBasicBlock*) * bb->predsAllocated * 2);
        bb->predsAllocated *= 2;

        if (bb->preds == NULL)
            warn_if_reached();
    }

    bb->preds[bb->predCount] = pred;
    bb->predCount++;
}

/**
 * Get the LLVM value of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \returns The LLVM basic block
 **/
LLVMBasicBlockRef
ll_basic_block_llvm(LLBasicBlock* bb)
{
    return bb->llvmBB;
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
        ll_basic_block_add_predecessor(branch, bb);
        bb->nextBranch = branch;
    }

    if (fallThrough != NULL)
    {
        ll_basic_block_add_predecessor(fallThrough, bb);
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
    state->currentBB = bb;
    LLVMPositionBuilderAtEnd(state->builder, bb->llvmBB);

    // Set new instruction pointer register
    uintptr_t rip = instr->addr + instr->len;
    LLVMValueRef ripValue = LLVMConstInt(LLVMInt64TypeInContext(state->context), rip, false);
    ll_set_register(ll_reg(LL_RT_IP, 0), FACET_I64, ripValue, true, state);

    // Add Metadata for debugging.
    LLVMValueRef intrinsicDoNothing = ll_support_get_intrinsic(state->module, LL_INTRINSIC_DO_NOTHING, NULL, 0);
    const char* instructionName = instr2string(instr, 0, NULL);
    LLVMValueRef mdCall = LLVMBuildCall(state->builder, intrinsicDoNothing, NULL, 0, "");
    LLVMValueRef mdNode = LLVMMDStringInContext(state->context, instructionName, strlen(instructionName));
    LLVMSetMetadata(mdCall, LLVMGetMDKindIDInContext(state->context, "asm.instr", 9), mdNode);

    switch (instr->type)
    {
#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include <opcodes.inc>
#undef DEF_IT

        default:
            printf("Could not handle instruction: %s\n", instructionName);
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
    LLVMValueRef branch = NULL;
    state->currentBB = bb;
    LLVMPositionBuilderAtEnd(state->builder, bb->llvmBB);

    LLInstrType endType = bb->endType;
    if (instrIsJcc(endType))
    {
        LLVMValueRef cond = ll_flags_condition(endType, LL_INS_JO, state);
        branch = LLVMBuildCondBr(state->builder, cond, bb->nextBranch->llvmBB, bb->nextFallThrough->llvmBB);
    }
    else if (endType == LL_INS_JMP)
        branch = LLVMBuildBr(state->builder, bb->nextBranch->llvmBB);
    else if (endType != LL_INS_RET && endType != LL_INS_Invalid) // Any other instruction which is not a terminator
        branch = LLVMBuildBr(state->builder, bb->nextFallThrough->llvmBB);

    if (state->cfg.enableFullLoopUnroll && branch != NULL)
        LLVMSetMetadata(branch, LLVMGetMDKindIDInContext(state->context, "llvm.loop", 9), state->unrollMD);
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
    if (bb->predCount == 0)
        return;

    LLState* state = bb->state;
    state->currentBB = NULL;

    for (size_t i = 0; i < bb->predCount; i++)
    {
        for (int j = 0; j < LL_RI_GPMax; j++)
        {
            for (size_t k = 0; k < FACET_COUNT; k++)
            {
                llvm::PHINode* phi = llvm::unwrap<llvm::PHINode>(bb->phiNodesGpRegisters[j].facets[k]);
                llvm::Value* value = llvm::unwrap(ll_basic_block_get_register(bb->preds[i], (RegisterFacet)k, ll_reg(LL_RT_GP64, j), state));
                phi->addIncoming(value, llvm::unwrap(bb->preds[i]->llvmBB));
            }
        }

        for (int j = 0; j < LL_RI_XMMMax; j++)
        {
            for (size_t k = 0; k < FACET_COUNT; k++)
            {
                llvm::PHINode* phi = llvm::unwrap<llvm::PHINode>(bb->phiNodesSseRegisters[j].facets[k]);
                llvm::Value* value = llvm::unwrap(ll_basic_block_get_register(bb->preds[i], (RegisterFacet)k, ll_reg(LL_RT_XMM, j), state));
                phi->addIncoming(value, llvm::unwrap(bb->preds[i]->llvmBB));
            }
        }

        for (int j = 0; j < RFLAG_Max; j++)
        {
            llvm::PHINode* phi = llvm::unwrap<llvm::PHINode>(bb->phiNodesFlags[j]);
            llvm::Value* value = llvm::unwrap(ll_regfile_get_flag(bb->preds[i]->regfile, j));
            phi->addIncoming(value, llvm::unwrap(bb->preds[i]->llvmBB));
        }
    }
}

/**
 * Get a register value of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param reg The register
 * \returns The register value in the given facet
 **/
LLVMValueRef
ll_basic_block_get_register(LLBasicBlock* bb, RegisterFacet facet, LLReg reg, LLState* state)
{
    return ll_regfile_get(bb->regfile, facet, reg, state);
}

/**
 * Clear a register to undefined of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param reg The register
 * \param value The new value
 **/
void
ll_basic_block_clear_register(LLBasicBlock* bb, LLReg reg, LLState* state)
{
    ll_regfile_clear(bb->regfile, reg, state);
}

/**
 * Set a register in all facets to zero within the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param reg The name of the new register
 * \param state The state
 **/
void
ll_basic_block_zero_register(LLBasicBlock* bb, LLReg reg, LLState* state)
{
    ll_regfile_zero(bb->regfile, reg, state);
}

/**
 * Rename a register to another register of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param reg The name of the new register
 * \param current The name of the current register
 * \param state The state
 **/
void
ll_basic_block_rename_register(LLBasicBlock* bb, LLReg reg, LLReg current, LLState* state)
{
    ll_regfile_rename(bb->regfile, reg, current, state);
}

/**
 * Set a register value of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param reg The register
 * \param value The new value
 **/
void
ll_basic_block_set_register(LLBasicBlock* bb, RegisterFacet facet, LLReg reg, LLVMValueRef value, bool clearOthers, LLState* state)
{
    ll_regfile_set(bb->regfile, facet, reg, value, clearOthers, state);
}

/**
 * Get a flag value of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param flag The flag
 * \returns The current flag value
 **/
LLVMValueRef
ll_basic_block_get_flag(LLBasicBlock* bb, int flag)
{
    return ll_regfile_get_flag(bb->regfile, flag);
}

/**
 * Set a flag value of the basic block.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \param flag The flag
 * \param value The new value
 **/
void
ll_basic_block_set_flag(LLBasicBlock* bb, int flag, LLVMValueRef value)
{
    ll_regfile_set_flag(bb->regfile, flag, value);
}

/**
 * Get the flag cache.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param bb The basic block
 * \returns The flag cache
 **/
LLFlagCache*
ll_basic_block_get_flag_cache(LLBasicBlock* bb)
{
    return ll_regfile_get_flag_cache(bb->regfile);
}

/**
 * @}
 **/
