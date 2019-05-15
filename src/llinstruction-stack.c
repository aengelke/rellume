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
#include <llvm-c/Core.h>

#include <llinstruction-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llinstr-internal.h>
#include <lloperand-internal.h>
#include <llregfile-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLInstructionStack Push/Pop/Leave Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

static LLVMValueRef
ll_instruction_get_flags(bool fullSized, LLState* state)
{
    static const int flags[] = {
        RFLAG_CF,       -1, RFLAG_PF,       -1,
        RFLAG_AF,       -1, RFLAG_ZF, RFLAG_SF,
              -1,       -1,       -1, RFLAG_OF,
              -1,       -1,       -1,       -1,
    };

    LLVMTypeRef intType = LLVMIntTypeInContext(state->context, fullSized ? 64 : 16);
    LLVMValueRef flagRegister = LLVMConstNull(intType);
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++)
    {
        if (flags[i] < 0)
            continue;

        LLVMValueRef flag = LLVMBuildZExt(state->builder, ll_get_flag(flags[i], state), intType, "");
        flag = LLVMBuildShl(state->builder, flag, LLVMConstInt(intType, i, false), "");
        flagRegister = LLVMBuildOr(state->builder, flagRegister, flag, "");
    }

    return flagRegister;
}

void
ll_instruction_stack(LLInstr* instr, LLState* state)
{
    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
    LLVMTypeRef pi8 = LLVMPointerType(i8, 0);
    LLVMTypeRef pi64 = LLVMPointerType(i64, 0);

    // In case of a leave instruction, we basically pop the new base pointer
    // from RBP and store the new value as stack pointer.
    int spRegIndex = instr->type == LL_INS_LEAVE ? LL_RI_BP : LL_RI_SP;
    LLVMValueRef spReg = ll_get_register(ll_reg(LL_RT_GP64, spRegIndex), FACET_PTR, state);
    LLVMValueRef sp = LLVMBuildPointerCast(state->builder, spReg, pi64, "");
    LLVMValueRef newSp = NULL;
    LLVMValueRef value;

    if (instr->type == LL_INS_PUSH || instr->type == LL_INS_PUSHFQ)
    {
        // Decrement Stack Pointer via a GEP instruction
        LLVMValueRef constSub = LLVMConstInt(i64, -1, false);
        newSp = LLVMBuildGEP(state->builder, sp, &constSub, 1, "");

        if (instr->type == LL_INS_PUSH)
            value = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state);
        else
            value = ll_instruction_get_flags(true, state);

        value = LLVMBuildSExtOrBitCast(state->builder, value, i64, "");
        LLVMBuildStore(state->builder, value, newSp);
    }
    else if (instr->type == LL_INS_POP || instr->type == LL_INS_LEAVE)
    {
        LLInstrOp* operand = instr->type == LL_INS_LEAVE
            ? getRegOp(ll_reg(LL_RT_GP64, LL_RI_BP))
            : &instr->dst;

        value = LLVMBuildLoad(state->builder, sp, "");
        ll_operand_store(OP_SI, ALIGN_MAXIMUM, operand, REG_DEFAULT, value, state);

        // Advance Stack pointer via a GEP
        LLVMValueRef constAdd = LLVMConstInt(i64, 1, false);
        newSp = LLVMBuildGEP(state->builder, sp, &constAdd, 1, "");
    }
    else
        warn_if_reached();

    // Cast back to int for register store
    LLVMValueRef newSpReg = LLVMBuildPointerCast(state->builder, newSp, pi8, "");
    LLVMSetMetadata(newSpReg, LLVMGetMDKindIDInContext(state->context, "asm.reg.rsp", 11), state->emptyMD);

    ll_set_register(ll_reg(LL_RT_GP64, LL_RI_SP), FACET_PTR, newSpReg, true, state);
}

/**
 * @}
 **/
