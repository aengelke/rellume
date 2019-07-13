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

/**
 * @}
 **/
