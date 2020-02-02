/**
 * This file is part of Rellume.
 *
 * (c) 2019, Alexis Engelke <alexis.engelke@googlemail.com>
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

#include "callconv.h"

#include "function-info.h"
#include "regfile.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <cassert>


namespace rellume {

llvm::FunctionType* CallConv::FnType(llvm::LLVMContext& ctx,
                                     unsigned sptr_addrspace) const {
    llvm::Type* void_ty = llvm::Type::getVoidTy(ctx);
    llvm::Type* i8p = llvm::Type::getInt8PtrTy(ctx, sptr_addrspace);
    llvm::Type* i64 = llvm::Type::getInt64Ty(ctx);

    switch (*this) {
    default:
        return nullptr;
    case CallConv::SPTR:
        return llvm::FunctionType::get(void_ty, {i8p}, false);
    case CallConv::HHVM: {
        auto ret_ty = llvm::StructType::get(i64, i64, i64, i64, i64, i64, i64,
                                            i64, i64, i64, i64, i64, i64, i64);
        return llvm::FunctionType::get(ret_ty, {i64, i8p, i64, i64, i64, i64,
                                                i64, i64, i64, i64, i64, i64,
                                                i64, i64}, false);
    }
    }
}

llvm::CallingConv::ID CallConv::FnCallConv() const {
    switch (*this) {
    default: return llvm::CallingConv::C;
    case CallConv::SPTR: return llvm::CallingConv::C;
    case CallConv::HHVM: return llvm::CallingConv::HHVM;
    }
}

unsigned CallConv::CpuStructParamIdx() const {
    switch (*this) {
    default: return 0;
    case CallConv::SPTR: return 0;
    case CallConv::HHVM: return 1;
    }
}

static const std::tuple<unsigned, X86Reg, Facet> cpu_struct_entries[] = {
#define RELLUME_MAPPED_REG(nameu,off,reg,facet) std::make_tuple(SptrIdx::nameu, reg, facet),
#include <rellume/cpustruct-private.inc>
#undef RELLUME_MAPPED_REG
};

llvm::Value* CallConv::Pack(RegFile& regfile, FunctionInfo& fi) const {
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    llvm::Value* ret_val = nullptr;
    if (*this == CallConv::HHVM)
        ret_val = llvm::UndefValue::get(fi.fn->getReturnType());

    for (const auto& [sptr_idx, reg, facet] : cpu_struct_entries) {
        if (*this == CallConv::HHVM) {
            int ins_idx = -1;
            if (reg.IsGP() && reg.Index() < 12) {
                // RAX->RAX; RCX->RCX; RDX->RDX; RBX->RBP; RSP->R15; RBP->R13;
                // RSI->RSI; RDI->RDI; R8->R8;   R9->R9;   R10->R10; R11->R11;
                static const uint8_t str_idx[] = {8,5,4,1,13,11,3,2,6,7,9,10};
                ins_idx = str_idx[reg.Index()];
            } else if (reg == X86Reg::IP) {
                ins_idx = 0; // RIP is stored in RBX
            }

            if (ins_idx >= 0) {
                unsigned ins_idx_u = static_cast<unsigned>(ins_idx);
                llvm::Value* reg_val = regfile.GetReg(reg, facet);
                ret_val = irb.CreateInsertValue(ret_val, reg_val, {ins_idx_u});
                continue;
            }
        }

        if (!fi.modified_regs_final || fi.modified_regs[sptr_idx]) {
            // GetReg moved in here to avoid generating dozens of dead PHI nodes
            irb.CreateStore(regfile.GetReg(reg, facet), fi.sptr[sptr_idx]);
        }
    }

    return ret_val;
}

void CallConv::Unpack(RegFile& regfile, FunctionInfo& fi) const {
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    for (const auto& [sptr_idx, reg, facet] : cpu_struct_entries) {
        llvm::Value* reg_val = nullptr;
        if (*this == CallConv::HHVM && reg.IsGP() && reg.Index() < 12) {
            // RAX->RAX; RCX->RCX; RDX->RDX; RBX->RBP; RSP->R15; RBP->R13;
            // RSI->RSI; RDI->RDI; R8->R8;   R9->R9;   R10->R10; R11->R11;
            static const uint8_t arg_idx[] = {10,7,6,2,3,13,5,4,8,9,11,12};
            reg_val = &fi.fn->arg_begin()[arg_idx[reg.Index()]];
        }

        if (reg_val == nullptr)
            reg_val = irb.CreateLoad(fi.sptr[sptr_idx]);

        fi.modified_regs.set(sptr_idx);
        regfile.SetReg(reg, facet, reg_val, false);
    }
}

} // namespace
