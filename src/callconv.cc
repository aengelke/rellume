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

CallConv CallConv::FromFunction(llvm::Function* fn, Arch arch) {
    auto fn_cconv = fn->getCallingConv();
    auto fn_ty = llvm::cast<llvm::FunctionType>(fn->getType()->getPointerElementType());
    CallConv hunch = INVALID;
    switch (arch) {
    case Arch::X86_64:
        hunch = fn_cconv == llvm::CallingConv::HHVM ? X86_64_HHVM : X86_64_SPTR;
        break;
    default:
        return INVALID;
    }

    // Verify hunch.
    if (hunch.FnCallConv() != fn_cconv)
        return INVALID;
    unsigned sptr_idx = hunch.CpuStructParamIdx();
    if (sptr_idx >= fn->arg_size())
        return INVALID;
    llvm::Type* sptr_ty = fn->arg_begin()[sptr_idx].getType();
    if (!sptr_ty->isPointerTy())
        return INVALID;
    unsigned sptr_addrspace = sptr_ty->getPointerAddressSpace();
    if (fn_ty != hunch.FnType(fn->getContext(), sptr_addrspace))
        return INVALID;
    return hunch;
}

llvm::FunctionType* CallConv::FnType(llvm::LLVMContext& ctx,
                                     unsigned sptr_addrspace) const {
    llvm::Type* void_ty = llvm::Type::getVoidTy(ctx);
    llvm::Type* i8p = llvm::Type::getInt8PtrTy(ctx, sptr_addrspace);
    llvm::Type* i64 = llvm::Type::getInt64Ty(ctx);

    switch (*this) {
    default:
        return nullptr;
    case CallConv::X86_64_SPTR:
        return llvm::FunctionType::get(void_ty, {i8p}, false);
    case CallConv::X86_64_HHVM: {
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
    case CallConv::X86_64_SPTR: return llvm::CallingConv::C;
    case CallConv::X86_64_HHVM: return llvm::CallingConv::HHVM;
    }
}

unsigned CallConv::CpuStructParamIdx() const {
    switch (*this) {
    default: return 0;
    case CallConv::X86_64_SPTR: return 0;
    case CallConv::X86_64_HHVM: return 1;
    }
}

using CPUStructEntry = std::tuple<unsigned, unsigned, ArchReg, Facet>;

// Note: replace with C++20 std::span.
template<typename T>
class span {
    T* ptr;
    std::size_t len;
public:
    constexpr span() : ptr(nullptr), len(0) {}
    template<std::size_t N>
    constexpr span(T (&arr)[N]) : ptr(arr), len(N) {}
    constexpr std::size_t size() const { return len; }
    constexpr T* begin() const { return &ptr[0]; }
    constexpr T* end() const { return &ptr[len]; }
};

static span<const CPUStructEntry> CPUStructEntries(CallConv cconv) {
    static const CPUStructEntry cpu_struct_entries_x86_64[] = {
#define RELLUME_MAPPED_REG(nameu,off,reg,facet) \
            std::make_tuple(SptrIdx::x86_64::nameu, off, reg, facet),
#include <rellume/cpustruct-x86_64-private.inc>
#undef RELLUME_MAPPED_REG
    };

    switch (cconv) {
    default:
        return span<const CPUStructEntry>();
    case CallConv::X86_64_SPTR:
    case CallConv::X86_64_HHVM:
        return cpu_struct_entries_x86_64;
    }
}


void CallConv::InitSptrs(BasicBlock* bb, FunctionInfo& fi) {
    llvm::IRBuilder<> irb(bb->GetRegFile()->GetInsertBlock());
    unsigned as = fi.sptr_raw->getType()->getPointerAddressSpace();

    const auto& cpu_struct_entries = CPUStructEntries(*this);
    fi.sptr.resize(cpu_struct_entries.size());
    for (const auto& [sptr_idx, off, reg, facet] : cpu_struct_entries) {
        llvm::Value* ptr = irb.CreateConstGEP1_64(fi.sptr_raw, off);
        llvm::Type* ty = facet.Type(irb.getContext())->getPointerTo(as);
        fi.sptr[sptr_idx] = irb.CreatePointerCast(ptr, ty);
    }
}

// Mapping of GP registers to HHVM parameters and return struct indices.
//     RAX->RAX; RCX->RCX; RDX->RDX; RBX->RBP; RSP->R15; RBP->R13;
//     RSI->RSI; RDI->RDI; R8->R8;   R9->R9;   R10->R10; R11->R11;
//     RIP->RBX; (not encoded here)
static bool hhvm_is_host_reg(ArchReg reg) {
    return reg == ArchReg::IP || (reg.IsGP() && reg.Index() < 12);
}
static uint8_t hhvm_arg_index(ArchReg reg) {
    static const uint8_t indices[] = {10, 7, 6, 2, 3, 13, 5, 4, 8, 9, 11, 12};
    assert(hhvm_is_host_reg(reg));
    return (reg.IsGP() && reg.Index() < 12) ? indices[reg.Index()] : 0;
}
static uint8_t hhvm_ret_index(ArchReg reg) {
    static const uint8_t indices[] = {8, 5, 4, 1, 13, 11, 3, 2, 6, 7, 9, 10};
    assert(hhvm_is_host_reg(reg));
    return (reg.IsGP() && reg.Index() < 12) ? indices[reg.Index()] : 0;
}

template<typename F>
static void Pack(CallConv cconv, BasicBlock* bb, FunctionInfo& fi, F hhvm_fn) {
    RegFile& regfile = *bb->GetRegFile();
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    const auto& cpu_struct_entries = CPUStructEntries(cconv);

    CallConvPack& pack_info = fi.call_conv_packs.emplace_back();
    pack_info.block_dirty_regs = regfile.DirtyRegs();
    pack_info.bb = bb;
    pack_info.stores.resize(cpu_struct_entries.size());

    for (const auto& [sptr_idx, off, reg, facet] : cpu_struct_entries) {
        if (reg.Kind() == ArchReg::RegKind::INVALID)
            continue;

        llvm::Value* reg_val = regfile.GetReg(reg, facet);

        if (cconv == CallConv::X86_64_HHVM && hhvm_is_host_reg(reg)) {
            hhvm_fn(reg, reg_val);
            continue;
        }

        unsigned regset_idx = RegisterSetBitIdx(reg, facet);
        regfile.DirtyRegs()[regset_idx] = false;
        regfile.CleanedRegs()[regset_idx] = true;
        pack_info.stores[sptr_idx] = irb.CreateStore(reg_val, fi.sptr[sptr_idx]);
    }
}

template<typename F>
static void Unpack(CallConv cconv, BasicBlock* bb, FunctionInfo& fi, F hhvm_fn) {
    RegFile& regfile = *bb->GetRegFile();
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    // Clear all facets before entering new values.
    regfile.Clear();
    for (const auto& [sptr_idx, off, reg, facet] : CPUStructEntries(cconv)) {
        if (reg.Kind() == ArchReg::RegKind::INVALID)
            continue;
        if (cconv == CallConv::X86_64_HHVM && hhvm_is_host_reg(reg)) {
            regfile.SetReg(reg, facet, hhvm_fn(reg), false);
            continue;
        }

        llvm::Value* reg_val = irb.CreateLoad(fi.sptr[sptr_idx]);
        // Mark register as clean if it was loaded from the sptr.
        regfile.SetReg(reg, facet, reg_val, false);
        regfile.DirtyRegs()[RegisterSetBitIdx(reg, facet)] = false;
    }
}

llvm::ReturnInst* CallConv::Return(BasicBlock* bb, FunctionInfo& fi) const {
    llvm::IRBuilder<> irb(bb->GetRegFile()->GetInsertBlock());

    llvm::SmallVector<llvm::Value*, 16> hhvm_ret;
    if (*this == CallConv::X86_64_HHVM) {
        hhvm_ret.resize(14);
        hhvm_ret[12] = llvm::UndefValue::get(irb.getInt64Ty());
    }

    Pack(*this, bb, fi, [&hhvm_ret] (ArchReg reg, llvm::Value* reg_val) {
        hhvm_ret[hhvm_ret_index(reg)] = reg_val;
    });

    if (*this == CallConv::X86_64_HHVM)
        return irb.CreateAggregateRet(hhvm_ret.data(), hhvm_ret.size());
    return irb.CreateRetVoid();
}

void CallConv::UnpackParams(BasicBlock* bb, FunctionInfo& fi) const {
    Unpack(*this, bb, fi, [&fi] (ArchReg reg) {
        return &fi.fn->arg_begin()[hhvm_arg_index(reg)];
    });
}

llvm::CallInst* CallConv::Call(llvm::Function* fn, BasicBlock* bb,
                               FunctionInfo& fi, bool tail_call) {
    llvm::SmallVector<llvm::Value*, 16> call_args;
    call_args.resize(fn->arg_size());
    call_args[CpuStructParamIdx()] = fi.sptr_raw;

    Pack(*this, bb, fi, [&call_args] (ArchReg reg, llvm::Value* reg_val) {
        call_args[hhvm_arg_index(reg)] = reg_val;
    });

    llvm::IRBuilder<> irb(bb->GetRegFile()->GetInsertBlock());

    llvm::CallInst* call = irb.CreateCall(fn->getFunctionType(), fn, call_args);
    call->setCallingConv(fn->getCallingConv());
    call->setAttributes(fn->getAttributes());

    if (tail_call) {
        call->setTailCallKind(llvm::CallInst::TCK_MustTail);
        if (call->getType()->isVoidTy())
            irb.CreateRetVoid();
        else
            irb.CreateRet(call);
        return call;
    }

    llvm::SmallVector<llvm::Value*, 14> hhvm_ret;
    if (*this == CallConv::X86_64_HHVM) {
        for (unsigned i = 0; i < 14; i++)
            hhvm_ret.push_back(irb.CreateExtractValue(call, {i}));
    }

    Unpack(*this, bb, fi, [&hhvm_ret] (ArchReg reg) {
        return hhvm_ret[hhvm_ret_index(reg)];
    });

    return call;
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
        for (const auto& [sptr_idx, off, reg, facet] : CPUStructEntries(*this)) {
            if (reg.Kind() == ArchReg::RegKind::INVALID)
                continue;
            if (pack.stores[sptr_idx] && !regset[RegisterSetBitIdx(reg, facet)]) {
                pack.stores[sptr_idx]->eraseFromParent();
            }
        }
    }
}

} // namespace
