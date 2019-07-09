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

#include <cstdbool>
#include <cstdint>

#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>

#include <llinstruction-internal.h>
#include <llstate-internal.h>

#include <llcommon-internal.h>
#include <rellume/instr.h>

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

llvm::Value* LLState::LiftJmp(const LLInstr& inst) {
    llvm::Value* taken = OpLoad(inst.ops[0], Facet::I64);
    // TODO: no longer require this.
    return irb.Insert(llvm::SelectInst::Create(irb.getTrue(), taken, taken));
}
llvm::Value* LLState::LiftJcc(const LLInstr& inst) {
    llvm::Value* cond = FlagCond(inst.type, LL_INS_JO);
    llvm::Value* taken = OpLoad(inst.ops[0], Facet::I64);
    llvm::Value* nottaken = irb.getInt64(inst.addr + inst.len);
    return irb.Insert(llvm::SelectInst::Create(cond, taken, nottaken));
}

llvm::Value* LLState::LiftRet(const LLInstr& inst)
{
    llvm::Value* param = irb.GetInsertBlock()->getParent()->arg_begin();
    llvm::Type* cpu_type = param->getType()->getPointerElementType();
    llvm::Value* result = llvm::UndefValue::get(cpu_type);

    for (unsigned i = 0; i < LL_RI_GPMax; i++)
    {
        llvm::Value* value = GetReg(LLReg(LL_RT_GP64, i), Facet::I64);
        result = irb.CreateInsertValue(result, value, {1, i});
    }

    for (unsigned i = 0; i < LL_RI_XMMMax; i++)
    {
        llvm::Value* value = GetReg(LLReg(LL_RT_XMM, i), Facet::IVEC);
        result = irb.CreateInsertValue(result, value, {3, i});
    }

    for (unsigned i = 0; i < RFLAG_Max; i++)
    {
        llvm::Value* value = GetFlag(i);
        result = irb.CreateInsertValue(result, value, {2, i});
    }

    irb.CreateStore(result, param);

    irb.CreateRetVoid();

    (void) inst;

    return llvm::UndefValue::get(irb.getInt64Ty());
}

/**
 * @}
 **/
