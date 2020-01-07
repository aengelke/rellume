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
    llvm::Type* i8p = llvm::Type::getInt8Ty(ctx)->getPointerTo(sptr_addrspace);
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

static const std::tuple<unsigned, size_t, X86Reg, Facet> cpu_struct_entries[] = {
#define RELLUME_MAPPED_REG(nameu,off,reg,facet) std::make_tuple(SptrIdx::nameu, off, reg, facet),
#include <rellume/cpustruct-private.inc>
#undef RELLUME_MAPPED_REG
};

llvm::Value* CallConv::Pack(RegFile& regfile, FunctionInfo& fi) const {
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    llvm::Value* ret_val = nullptr;
    if (*this == CallConv::HHVM)
        ret_val = llvm::UndefValue::get(fi.fn->getReturnType());

    for (auto& entry : cpu_struct_entries) {
        unsigned sptr_idx; size_t offset; X86Reg reg; Facet facet;
        std::tie(sptr_idx, offset, reg, facet) = entry;

        bool store_in_sptr = true;
        if (*this == CallConv::HHVM) {
            int ins_idx = -1;
            if (reg == X86Reg(X86Reg::IP))
                ins_idx = 0; // RIP is stored in RBX
            else if (reg == X86Reg(X86Reg::GP, 0))
                ins_idx = 8; // RAX is stored in RAX
            else if (reg == X86Reg(X86Reg::GP, 1))
                ins_idx = 5; // RCX is stored in RCX
            else if (reg == X86Reg(X86Reg::GP, 2))
                ins_idx = 4; // RDX is stored in RDX
            else if (reg == X86Reg(X86Reg::GP, 3))
                ins_idx = 1; // RBX is stored in RBP
            else if (reg == X86Reg(X86Reg::GP, 4))
                ins_idx = 13; // RSP is stored in R15
            else if (reg == X86Reg(X86Reg::GP, 5))
                ins_idx = 11; // RBP is stored in R13
            else if (reg == X86Reg(X86Reg::GP, 6))
                ins_idx = 3; // RSI is stored in RSI
            else if (reg == X86Reg(X86Reg::GP, 7))
                ins_idx = 2; // RDI is stored in RDI
            else if (reg == X86Reg(X86Reg::GP, 8))
                ins_idx = 6; // R8 is stored in R8
            else if (reg == X86Reg(X86Reg::GP, 9))
                ins_idx = 7; // R9 is stored in R9
            else if (reg == X86Reg(X86Reg::GP, 10))
                ins_idx = 9; // R10 is stored in R10
            else if (reg == X86Reg(X86Reg::GP, 11))
                ins_idx = 10; // R11 is stored in R11

            if (ins_idx >= 0) {
                unsigned ins_idx_u = static_cast<unsigned>(ins_idx);
                llvm::Value* reg_val = regfile.GetReg(reg, facet);
                ret_val = irb.CreateInsertValue(ret_val, reg_val, {ins_idx_u});
                store_in_sptr = false;
            }
        }

        store_in_sptr &= !fi.modified_regs_final || fi.modified_regs.count(reg);
        if (store_in_sptr) {
            // GetReg moved in here to avoid generating dozens of dead PHI nodes
            irb.CreateStore(regfile.GetReg(reg, facet), fi.sptr[sptr_idx]);
        }
    }

    return ret_val;
}

void CallConv::Unpack(RegFile& regfile, FunctionInfo& fi) const {
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    for (auto& entry : cpu_struct_entries) {
        unsigned sptr_idx; size_t offset; X86Reg reg; Facet facet;
        std::tie(sptr_idx, offset, reg, facet) = entry;

        llvm::Value* reg_val = nullptr;
        if (*this == CallConv::HHVM) {
            int arg_idx = -1;
            if (reg == X86Reg(X86Reg::GP, 0))
                arg_idx = 10; // RAX is stored in RAX
            else if (reg == X86Reg(X86Reg::GP, 1))
                arg_idx = 7; // RCX is stored in RCX
            else if (reg == X86Reg(X86Reg::GP, 2))
                arg_idx = 6; // RDX is stored in RDX
            else if (reg == X86Reg(X86Reg::GP, 3))
                arg_idx = 2; // RBX is stored in RBP
            else if (reg == X86Reg(X86Reg::GP, 4))
                arg_idx = 3; // RSP is stored in R15
            else if (reg == X86Reg(X86Reg::GP, 5))
                arg_idx = 13; // RBP is stored in R13
            else if (reg == X86Reg(X86Reg::GP, 6))
                arg_idx = 5; // RSI is stored in RSI
            else if (reg == X86Reg(X86Reg::GP, 7))
                arg_idx = 4; // RDI is stored in RDI
            else if (reg == X86Reg(X86Reg::GP, 8))
                arg_idx = 8; // R8 is stored in R8
            else if (reg == X86Reg(X86Reg::GP, 9))
                arg_idx = 9; // R9 is stored in R9
            else if (reg == X86Reg(X86Reg::GP, 10))
                arg_idx = 11; // R10 is stored in R10
            else if (reg == X86Reg(X86Reg::GP, 11))
                arg_idx = 12; // R11 is stored in R11

            if (arg_idx >= 0)
                reg_val = &fi.fn->arg_begin()[arg_idx];
        }

        if (reg_val == nullptr)
            reg_val = irb.CreateLoad(fi.sptr[sptr_idx]);

        fi.modified_regs.insert(reg);
        regfile.SetReg(reg, facet, reg_val, false);
    }
}

} // namespace
