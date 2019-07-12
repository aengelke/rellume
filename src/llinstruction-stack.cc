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

void LLState::LiftPushPushf(const LLInstr& inst) {
    llvm::Value* op1;
    if (inst.type == LL_INS_PUSHFQ)
        op1 = llvm::unwrap(ll_instruction_get_flags(true, this));
    else
        op1 = OpLoad(inst.ops[0], Facet::I);

    llvm::Value* rsp = GetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, op1->getType()->getPointerTo());
    rsp = irb.CreateConstGEP1_64(rsp, -1);
    irb.CreateStore(op1, rsp);

    rsp = irb.CreatePointerCast(rsp, irb.getInt8PtrTy());
    SetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR, rsp);
}

void LLState::LiftPopLeaveRet(const LLInstr& inst) {
    LLInstrOp tgt_op;
    if (inst.type == LL_INS_LEAVE)
        tgt_op = LLInstrOp::Reg(LLReg(LL_RT_GP64, LL_RI_BP));
    else if (inst.type == LL_INS_RET)
        tgt_op = LLInstrOp::Reg(LLReg(LL_RT_IP, 0));
    else
        tgt_op = inst.ops[0];

    // A LEAVE is essentially a POP RBP from RBP.
    LLReg src_reg(LL_RT_GP64, inst.type == LL_INS_LEAVE ? LL_RI_BP : LL_RI_SP);
    llvm::Value* rsp = GetReg(src_reg, Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, irb.getInt64Ty()->getPointerTo());
    OpStoreGp(tgt_op, irb.CreateLoad(rsp));

    rsp = irb.CreateConstGEP1_64(rsp, 1);
    rsp = irb.CreatePointerCast(rsp, irb.getInt8PtrTy());
    SetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR, rsp);
}

/**
 * @}
 **/
