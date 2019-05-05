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
#include <llvm-c/Core.h>

#include <llinstruction-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llflags-internal.h>
#include <llinstr-internal.h>
#include <lloperand-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLInstruction Instruction
 * \brief Handling of X86-64 instructions
 *
 * @{
 **/

/**
 * Handling of an instruction.
 *
 * \todo Support other return types than i64, float and double
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param instr The push instruction
 * \param state The module state
 **/
void
ll_generate_instruction(LLInstr* instr, LLState* state)
{
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
        LLVMValueRef operand1;
        LLVMValueRef operand2;
        LLVMValueRef result;

#define CD_FUNCTION(fn,...) fn(instr, state, ##__VA_ARGS__)
#define CD_NOP()
#define CD_BINARY_INT_LLVM(build_fn,flag_fn) \
            operand1 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->dst, state); \
            operand2 = ll_operand_load(OP_SI, ALIGN_MAXIMUM, &instr->src, state); \
            result = build_fn(state->builder, operand1, operand2, ""); \
            flag_fn(state, result, operand1, operand2); \
            ll_operand_store(OP_SI, ALIGN_MAXIMUM, &instr->dst, REG_DEFAULT, result, state);
#define CD_BINARY_FP_LLVM(data_type,prh,build_fn,fastmath) \
            operand1 = ll_operand_load(data_type, ALIGN_MAXIMUM, &instr->dst, state); \
            operand2 = ll_operand_load(data_type, ALIGN_MAXIMUM, &instr->src, state); \
            result = build_fn(state->builder, operand1, operand2, ""); \
            if (fastmath && state->cfg.enableFastMath) \
                ll_support_enable_fast_math(result); \
            ll_operand_store(data_type, ALIGN_MAXIMUM, &instr->dst, prh, result, state);

#define DEF_IT(opc,handler) case LL_INS_ ## opc : handler; break;
#include <opcodes.inc>
#undef DEF_IT

        default:
            printf("Could not handle instruction: %s\n", instr2string(instr, 0, NULL));
            warn_if_reached();
            break;
    }
}

/**
 * @}
 **/
