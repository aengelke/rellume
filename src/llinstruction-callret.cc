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
#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>

#include <llinstruction-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llinstr-internal.h>
#include <lloperand-internal.h>
#include <llregfile-internal.h>
#include <llstate-internal.h>
#include <llsupport-internal.h>

/**
 * \defgroup LLInstructionCall Call/Ret Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

void
ll_instruction_call(LLInstr* instr, LLState* state)
{
    (void) instr;
    (void) state;

    warn_if_reached();
#if 0
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);

    uintptr_t address = 0;

    if (instr->dst.type == LL_OP_IMM)
        address = instr->dst.val;
    else if (instr->dst.type == LL_OP_REG && instr->dst.reg.rt == LL_RT_GP64)
    {
        LLVMValueRef value = ll_get_register(instr->dst.reg, FACET_I64, state);

        if (ll_support_is_constant_int(value))
            address = LLVMConstIntGetZExtValue(value);
    }

    if (address == 0)
        warn_if_reached();

    // Find function with corresponding address.
    LLFunction* function = NULL;

    for (size_t i = 0; i < state->functionCount; i++)
        if (state->functions[i]->address == address)
            function = state->functions[i];

    if (function == NULL)
        return;
        // warn_if_reached();

    LLVMValueRef llvmFunction = function->llvmFunction;
    LLVMAddAttributeAtIndex(llvmFunction, -1, ll_support_get_enum_attr(state->context, "inlinehint"));

    // Construct arguments.
    LLVMTypeRef fnType = LLVMGetElementType(LLVMTypeOf(llvmFunction));
    size_t argCount = LLVMCountParamTypes(fnType);

    LLVMValueRef args[argCount];
    ll_operand_construct_args(fnType, args, state);

    LLVMValueRef result = LLVMBuildCall(state->builder, llvmFunction, args, argCount, "");

    if (LLVMGetTypeKind(LLVMTypeOf(result)) == LLVMPointerTypeKind)
        result = LLVMBuildPtrToInt(state->builder, result, i64, "");
    if (LLVMTypeOf(result) != i64)
        warn_if_reached();

    // TODO: Handle return values except for i64!
    ll_set_register(ll_reg(LL_RT_GP64, LL_RI_A), FACET_I64, result, true, state);

    // Clobber registers.
    ll_clear_register(ll_reg(LL_RT_GP64, LL_RI_C), state);
    ll_clear_register(ll_reg(LL_RT_GP64, LL_RI_D), state);
    ll_clear_register(ll_reg(LL_RT_GP64, LL_RI_SI), state);
    ll_clear_register(ll_reg(LL_RT_GP64, LL_RI_DI), state);
    ll_clear_register(ll_reg(LL_RT_GP64, 8), state);
    ll_clear_register(ll_reg(LL_RT_GP64, 9), state);
    ll_clear_register(ll_reg(LL_RT_GP64, 10), state);
    ll_clear_register(ll_reg(LL_RT_GP64, 11), state);
#endif
}

void
ll_instruction_ret(LLInstr* instr, LLState* state)
{
    llvm::IRBuilder<>* builder = llvm::unwrap(state->builder);
    llvm::Value* param = builder->GetInsertBlock()->getParent()->arg_begin();
    llvm::Type* cpu_type = llvm::cast<llvm::PointerType>(param->getType())->getElementType();
    llvm::Value* result = llvm::UndefValue::get(cpu_type);

    // Set all registers to undef first.
    for (unsigned i = 0; i < LL_RI_GPMax; i++)
    {
        LLVMValueRef value = ll_get_register(ll_reg(LL_RT_GP64, i), FACET_I64, state);
        result = builder->CreateInsertValue(result, llvm::unwrap(value), {1, i});
    }

    for (unsigned i = 0; i < LL_RI_XMMMax; i++)
    {
        LLVMValueRef value = ll_get_register(ll_reg(LL_RT_XMM, i), FACET_IVEC, state);
        result = builder->CreateInsertValue(result, llvm::unwrap(value), {3, i});
    }

    for (unsigned i = 0; i < RFLAG_Max; i++)
    {
        LLVMValueRef value = ll_get_flag(i, state);
        result = builder->CreateInsertValue(result, llvm::unwrap(value), {2, i});
    }

    builder->CreateStore(result, param);

    builder->CreateRetVoid();

    (void) instr;
}

/**
 * @}
 **/
