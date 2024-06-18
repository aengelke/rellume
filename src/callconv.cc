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
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <cassert>


namespace rellume {

CallConv CallConv::FromFunction(llvm::Function* fn, Arch arch) {
    auto fn_cconv = fn->getCallingConv();
    auto fn_ty = fn->getFunctionType();
    CallConv hunch = INVALID;
    switch (arch) {
#ifdef RELLUME_WITH_X86_64
    case Arch::X86_64: hunch = X86_64_SPTR; break;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
    case Arch::RV64: hunch = RV64_SPTR; break;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
    case Arch::AArch64: hunch = AArch64_SPTR; break;
#endif // RELLUME_WITH_AARCH64
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
    llvm::Type* ptrTy = llvm::PointerType::get(ctx, sptr_addrspace);

    switch (*this) {
    default:
        return nullptr;
    case CallConv::X86_64_SPTR:
    case CallConv::RV64_SPTR:
    case CallConv::AArch64_SPTR:
        return llvm::FunctionType::get(void_ty, {ptrTy}, false);
    }
}

llvm::CallingConv::ID CallConv::FnCallConv() const {
    switch (*this) {
    default: return llvm::CallingConv::C;
    case CallConv::X86_64_SPTR: return llvm::CallingConv::C;
    case CallConv::RV64_SPTR: return llvm::CallingConv::C;
    case CallConv::AArch64_SPTR: return llvm::CallingConv::C;
    }
}

unsigned CallConv::CpuStructParamIdx() const {
    switch (*this) {
    default: return 0;
    case CallConv::X86_64_SPTR:  return 0;
    case CallConv::RV64_SPTR:    return 0;
    case CallConv::AArch64_SPTR: return 0;
    }
}

Arch CallConv::ToArch() const {
    switch (*this) {
    default: return Arch::INVALID;
    case CallConv::X86_64_SPTR:  return Arch::X86_64;
    case CallConv::RV64_SPTR:    return Arch::RV64;
    case CallConv::AArch64_SPTR: return Arch::AArch64;
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
#ifdef RELLUME_WITH_X86_64
    static const CPUStructEntry cpu_struct_entries_x86_64[] = {
#define RELLUME_MAPPED_REG(nameu,off,reg,facet) \
            std::make_tuple(SptrIdx::x86_64::nameu, off, reg, facet),
#include <rellume/cpustruct-x86_64-private.inc>
#undef RELLUME_MAPPED_REG
    };
#endif // RELLUME_WITH_X86_64

#ifdef RELLUME_WITH_RV64
    static const CPUStructEntry cpu_struct_entries_rv64[] = {
#define RELLUME_MAPPED_REG(nameu,off,reg,facet) \
            std::make_tuple(SptrIdx::rv64::nameu, off, reg, facet),
#include <rellume/cpustruct-rv64-private.inc>
#undef RELLUME_MAPPED_REG
    };
#endif // RELLUME_WITH_RV64

#ifdef RELLUME_WITH_AARCH64
    static const CPUStructEntry cpu_struct_entries_aarch64[] = {
#define RELLUME_MAPPED_REG(nameu,off,reg,facet) \
            std::make_tuple(SptrIdx::aarch64::nameu, off, reg, facet),
#include <rellume/cpustruct-aarch64-private.inc>
#undef RELLUME_MAPPED_REG
    };
#endif // RELLUME_WITH_AARCH64

    switch (cconv) {
    default:
        return span<const CPUStructEntry>();
#ifdef RELLUME_WITH_X86_64
    case CallConv::X86_64_SPTR:
        return cpu_struct_entries_x86_64;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
    case CallConv::RV64_SPTR:
        return cpu_struct_entries_rv64;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
    case CallConv::AArch64_SPTR:
        return cpu_struct_entries_aarch64;
#endif // RELLUME_WITH_AARCH64
    }
}

static const char ** CPURegisterNameMap(CallConv cconv) {
#ifdef RELLUME_WITH_X86_64
    static const char * x86_64_reg_index_to_name[] = {
#define RELLUME_MAPPED_REG(nameu,...) #nameu,
#include <rellume/cpustruct-x86_64-private.inc>
#undef RELLUME_MAPPED_REG
    };
#endif // RELLUME_WITH_X86_64

#ifdef RELLUME_WITH_RV64
    static const char * rv64_reg_index_to_name[] = {
#define RELLUME_MAPPED_REG(nameu,...) #nameu,
#include <rellume/cpustruct-rv64-private.inc>
#undef RELLUME_MAPPED_REG
    };
#endif // RELLUME_WITH_RV64

#ifdef RELLUME_WITH_AARCH64
    static const char * aarch64_reg_index_to_name[] = {
#define RELLUME_MAPPED_REG(nameu,...) #nameu,
#include <rellume/cpustruct-aarch64-private.inc>
#undef RELLUME_MAPPED_REG
    };
#endif // RELLUME_WITH_AARCH64

    switch (cconv) {
    default: 
        return NULL;

#ifdef RELLUME_WITH_X86_64
    case CallConv::X86_64_SPTR:
        return x86_64_reg_index_to_name;
#endif // RELLUME_WITH_X86_64

#ifdef RELLUME_WITH_RV64
    case CallConv::RV64_SPTR:
        return rv64_reg_index_to_name;
#endif // RELLUME_WITH_RV64

#ifdef RELLUME_WITH_AARCH64
    case CallConv::AArch64_SPTR:
        return aarch64_reg_index_to_name;
#endif // RELLUME_WITH_AARCH64
    }

    return NULL;
}


void CallConv::InitSptrs(ArchBasicBlock* bb, FunctionInfo& fi) {
    llvm::IRBuilder<> irb(bb->BeginBlock());
    llvm::Type* i8 = irb.getInt8Ty();

    const auto& cpu_struct_entries = CPUStructEntries(*this);
    const char ** reg_map = CPURegisterNameMap(*this);
    
    fi.sptr.resize(cpu_struct_entries.size());
    for (const auto& [sptr_idx, off, reg, facet] : cpu_struct_entries) {
        fi.sptr[sptr_idx] = irb.CreateConstGEP1_64(i8, fi.sptr_raw, off, reg_map[sptr_idx]);
    }
}

static void Pack(ArchBasicBlock* bb, FunctionInfo& fi, llvm::Instruction* before) {
    CallConvPack& pack_info = fi.call_conv_packs.emplace_back();
    pack_info.regfile = bb->TakeRegFile();
    pack_info.packBefore = before;
    pack_info.bb = bb;
}

template<typename F>
static void Unpack(CallConv cconv, ArchBasicBlock* bb, llvm::BasicBlock* llvmBlock, FunctionInfo& fi, F get_from_reg) {
    bb->InitEmpty(cconv.ToArch(), llvmBlock);
    // New regfile with everything cleared
    RegFile& regfile = *bb->GetRegFile();
    llvm::IRBuilder<> irb(regfile.GetInsertBlock());

    regfile.SetPC(irb.CreateLoad(irb.getInt64Ty(), fi.sptr_raw));
    for (const auto& [sptr_idx, off, reg, facet] : CPUStructEntries(cconv)) {
        if (reg.Kind() == ArchReg::RegKind::INVALID)
            continue;
        if (llvm::Value* reg_val = get_from_reg(reg)) {
            regfile.Set(reg, reg_val);
            continue;
        }

        llvm::Type* reg_ty = facet.Type(irb.getContext());
        auto typeVal = llvm::Constant::getNullValue(reg_ty);
        // Mark register as clean if it was loaded from the sptr.
        regfile.Set(reg, RegFile::Transform::Load, fi.sptr[sptr_idx], typeVal);
        regfile.DirtyRegs()[RegisterSetBitIdx(reg)] = false;
    }
}

llvm::ReturnInst* CallConv::Return(ArchBasicBlock* bb, FunctionInfo& fi) const {
    llvm::IRBuilder<> irb(bb->GetRegFile()->GetInsertBlock());
    llvm::ReturnInst* ret = irb.CreateRetVoid();
    Pack(bb, fi, ret);
    return ret;
}

void CallConv::UnpackParams(ArchBasicBlock* bb, FunctionInfo& fi) const {
    Unpack(*this, bb, bb->BeginBlock(), fi, [&fi] (ArchReg reg) {
        return nullptr;
    });
}

llvm::CallInst* CallConv::Call(llvm::Function* fn, ArchBasicBlock* bb,
                               FunctionInfo& fi, bool tail_call) {
    llvm::SmallVector<llvm::Value*, 16> call_args;
    call_args.resize(fn->arg_size());
    call_args[CpuStructParamIdx()] = fi.sptr_raw;

    llvm::IRBuilder<> irb(bb->EndBlock());

    llvm::CallInst* call = irb.CreateCall(fn->getFunctionType(), fn, call_args);
    call->setCallingConv(fn->getCallingConv());
    call->setAttributes(fn->getAttributes());

    Pack(bb, fi, call);

    if (tail_call) {
        call->setTailCallKind(llvm::CallInst::TCK_MustTail);
        if (call->getType()->isVoidTy())
            irb.CreateRetVoid();
        else
            irb.CreateRet(call);
        return call;
    }

    Unpack(*this, bb, irb.GetInsertBlock(), fi, [] (ArchReg reg) {
        return nullptr;
    });

    return call;
}

void CallConv::OptimizePacks(FunctionInfo& fi, ArchBasicBlock* entry) {
    // Map of basic block to dirty register at (beginning, end) of the block.
    llvm::DenseMap<ArchBasicBlock*, std::pair<RegisterSet, RegisterSet>> bb_map;

    llvm::SmallPtrSet<ArchBasicBlock*, 16> queue;
    llvm::SmallVector<ArchBasicBlock*, 16> queue_vec;
    queue.insert(entry);
    queue_vec.push_back(entry);

    while (!queue.empty()) {
        llvm::SmallVector<ArchBasicBlock*, 16> new_queue_vec;
        for (ArchBasicBlock* bb : queue_vec) {
            queue.erase(bb);
            RegisterSet pre;
            for (ArchBasicBlock* pred : bb->Predecessors())
                pre |= bb_map.lookup(pred).second;

            RegisterSet post;
            if (RegFile* rf = bb->GetRegFile()){
                if (rf->StartsClean())
                    post = rf->DirtyRegs();
                else
                    post = pre | rf->DirtyRegs();
            }
            auto new_regsets = std::make_pair(pre, post);

            auto [it, inserted] = bb_map.try_emplace(bb, new_regsets);
            // If it is the first time we look at bb, or the set of dirty
            // registers changed, look at successors (again).
            if (inserted || it->second.second != post) {
                for (ArchBasicBlock* succ : bb->Successors()) {
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
        RegFile& regfile = *pack.regfile;
        regfile.SetInsertPoint(pack.packBefore->getIterator());

        RegisterSet regset = regfile.DirtyRegs();
        if (!regfile.StartsClean())
            regset |= bb_map.lookup(pack.bb).first;
        llvm::IRBuilder<> irb(pack.packBefore);
        irb.CreateStore(regfile.GetPCValue(fi.pc_base_value, fi.pc_base_addr), fi.sptr_raw);
        for (const auto& [sptr_idx, off, reg, facet] : CPUStructEntries(*this)) {
            if (reg.Kind() == ArchReg::RegKind::INVALID)
                continue;
            unsigned regidx = RegisterSetBitIdx(reg);
            if (!regset[regidx])
                continue;
            // Find best position for store. Hoist stores up to predecessors
            // where possible to avoid executing stores on code paths that never
            // write the register, but don't hoist them inside loops or similar.
            ArchBasicBlock* bb = pack.bb;
            RegFile* rf = &regfile;
            while (!rf->StartsClean() && !rf->DirtyRegs()[regidx]) {
                // Try to find single predecessor where register is written.
                ArchBasicBlock* dirtyPred = nullptr;
                for (ArchBasicBlock* pred : bb->Predecessors()) {
                    // Ignore predecessors where the register is never written.
                    if (!bb_map.lookup(pred).second[regidx])
                        continue;
                    if (dirtyPred) {
                        dirtyPred = nullptr;
                        break;
                    }
                    dirtyPred = pred;
                }
                // If there is no single dirty predecessor or if that has
                // multiple successors (possibly a loop), abort.
                if (!dirtyPred || dirtyPred->Successors().size() != 1)
                    break;

                bb = dirtyPred;
                rf = bb->GetRegFile();
            }

            llvm::Value* reg_val = rf->GetReg(reg, facet);
            if (llvm::isa<llvm::UndefValue>(reg_val))
                continue; // Just remove stores of undef.
            if (rf != &regfile) {
                auto terminator = rf->GetInsertBlock()->getTerminator();
                new llvm::StoreInst(reg_val, fi.sptr[sptr_idx], terminator);
            } else {
                irb.CreateStore(reg_val, fi.sptr[sptr_idx]);
            }
        }
    }
}

} // namespace
