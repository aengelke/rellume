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

#include "basicblock.h"
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

llvm::Value* CallConv::Pack(BasicBlock* bb, FunctionInfo& fi) const {
    RegFile& regfile = *bb->GetRegFile();
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    llvm::Value* ret_val = nullptr;
    if (*this == CallConv::HHVM)
        ret_val = llvm::UndefValue::get(fi.fn->getReturnType());

    CallConvPack& pack_info = fi.call_conv_packs.emplace_back();
    pack_info.block_dirty_regs = regfile.DirtyRegs();
    pack_info.bb = bb;

    for (const auto& [sptr_idx, reg, facet] : cpu_struct_entries) {
        llvm::Value* reg_val = regfile.GetReg(reg, facet);

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
                ret_val = irb.CreateInsertValue(ret_val, reg_val, {ins_idx_u});
                continue;
            }
        }

        unsigned regset_idx = RegisterSetBitIdx(reg, facet);
        regfile.DirtyRegs()[regset_idx] = false;
        regfile.CleanedRegs()[regset_idx] = true;
        pack_info.stores[sptr_idx] = irb.CreateStore(reg_val, fi.sptr[sptr_idx]);
    }

    return ret_val;
}

void CallConv::Unpack(BasicBlock* bb, FunctionInfo& fi) const {
    RegFile& regfile = *bb->GetRegFile();
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    for (const auto& [sptr_idx, reg, facet] : cpu_struct_entries) {
        llvm::Value* reg_val = nullptr;
        if (*this == CallConv::HHVM) {
            if (reg.IsGP() && reg.Index() < 12) {
                // RAX->RAX; RCX->RCX; RDX->RDX; RBX->RBP; RSP->R15; RBP->R13;
                // RSI->RSI; RDI->RDI; R8->R8;   R9->R9;   R10->R10; R11->R11;
                static const uint8_t arg_idx[] = {10,7,6,2,3,13,5,4,8,9,11,12};
                reg_val = &fi.fn->arg_begin()[arg_idx[reg.Index()]];
            } else if (reg == X86Reg::IP) {
                reg_val = &fi.fn->arg_begin()[0]; // RIP->RBX
            }
        }

        bool load_from_sptr = reg_val == nullptr;
        if (load_from_sptr)
            reg_val = irb.CreateLoad(fi.sptr[sptr_idx]);

        // Mark register only as modified if it didn't come from the sptr.
        regfile.SetReg(reg, facet, reg_val, false);
        if (load_from_sptr)
            regfile.DirtyRegs()[RegisterSetBitIdx(reg, facet)] = false;
    }
}

void CallConv::OptimizePacks(FunctionInfo& fi, BasicBlock* entry) {
    // Map of basic block to dirty register at (beginning, end) of the block.
    llvm::DenseMap<BasicBlock*, std::pair<RegisterSet, RegisterSet>> bb_map;

    llvm::SmallPtrSet<BasicBlock*, 16> queue;
    llvm::SmallVector<BasicBlock*, 16> queue_vec;
    queue.insert(entry);
    queue_vec.push_back(entry);

    while (!queue.empty()) {
        llvm::SmallVector<BasicBlock*, 16> new_queue_vec;
        for (BasicBlock* bb : queue_vec) {
            queue.erase(bb);
            RegisterSet pre;
            for (BasicBlock* pred : bb->Predecessors())
                pre |= bb_map.lookup(pred).second;

            RegFile* rf = bb->GetRegFile();
            RegisterSet post = (pre & ~rf->CleanedRegs()) | rf->DirtyRegs();
            auto new_regsets = std::make_pair(pre, post);

            auto [it, inserted] = bb_map.try_emplace(bb, new_regsets);
            // If it is the first time we look at bb, or the set of dirty
            // registers changed, look at successors (again).
            if (inserted || it->second.second != post) {
                for (BasicBlock* succ : bb->Successors()) {
                    // If user not in the set, then add it to the vector.
                    if (queue.insert(succ).second)
                        new_queue_vec.push_back(succ);
                }
            }

            // Ensure that we store the new value.
            if (!inserted)
                it->second = new_regsets;
        }
        queue_vec = std::move(new_queue_vec);
    }

    for (const auto& pack : fi.call_conv_packs) {
        RegisterSet regset = bb_map.lookup(pack.bb).first | pack.block_dirty_regs;
        for (const auto& [sptr_idx, reg, facet] : cpu_struct_entries) {
            if (pack.stores[sptr_idx] && !regset[RegisterSetBitIdx(reg, facet)]) {
                pack.stores[sptr_idx]->eraseFromParent();
            }
        }
    }
}

} // namespace
