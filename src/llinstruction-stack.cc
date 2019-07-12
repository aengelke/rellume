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
#include <llstate-internal.h>

#include <llcommon-internal.h>
#include <rellume/instr.h>

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

    LLVMTypeRef intType = llvm::wrap(state->irb.getIntNTy(fullSized ? 64 : 16));
    LLVMValueRef flagRegister = LLVMConstNull(intType);
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++)
    {
        if (flags[i] < 0)
            continue;

        LLVMValueRef flag = llvm::wrap(state->GetFlag(flags[i]));
        flag = LLVMBuildZExt(state->builder, flag, intType, "");
        flag = LLVMBuildShl(state->builder, flag, LLVMConstInt(intType, i, false), "");
        flagRegister = LLVMBuildOr(state->builder, flagRegister, flag, "");
    }

    return flagRegister;
}

void LLStateBase::StackPush(llvm::Value* value) {
    llvm::Value* rsp = GetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, value->getType()->getPointerTo());
    rsp = irb.CreateConstGEP1_64(rsp, -1);
    irb.CreateStore(value, rsp);

    rsp = irb.CreatePointerCast(rsp, irb.getInt8PtrTy());
    SetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR, rsp);
}

llvm::Value* LLStateBase::StackPop(const LLReg sp_src_reg) {
    llvm::Value* rsp = GetReg(sp_src_reg, Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, irb.getInt64Ty()->getPointerTo());

    llvm::Value* new_rsp = irb.CreateConstGEP1_64(rsp, 1);
    new_rsp = irb.CreatePointerCast(new_rsp, irb.getInt8PtrTy());
    SetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR, new_rsp);

    return irb.CreateLoad(rsp);
}

void LLState::LiftPushf(const LLInstr& inst) {
    StackPush(llvm::unwrap(ll_instruction_get_flags(true, this)));
}

/**
 * @}
 **/
