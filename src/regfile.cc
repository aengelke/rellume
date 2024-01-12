/**
 * This file is part of Rellume.
 *
 * (c) 2016-2019, Alexis Engelke <alexis.engelke@googlemail.com>
 * (c) 2020, Dominik Okwieka <dominik.okwieka@t-online.de>
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

#include "regfile.h"

#include "facet.h"

#include <llvm-c/Core.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>
#include <type_traits>

namespace rellume {

unsigned RegisterSetBitIdx(ArchReg reg, Facet facet) {
    switch (reg.Kind()) {
    case ArchReg::RegKind::IP:
        return 0;
    case ArchReg::RegKind::EFLAGS:
        switch (facet) {
        // clang-format off
        case Facet::OF: return 1; break;
        case Facet::SF: return 2; break;
        case Facet::ZF: return 3; break;
        case Facet::AF: return 4; break;
        case Facet::PF: return 5; break;
        case Facet::CF: return 6; break;
        case Facet::DF: return 7; break;
        default: assert(false && "invalid facet for EFLAGS register");
        }
        // clang-format on
    case ArchReg::RegKind::GP:
        return 8 + reg.Index();
    case ArchReg::RegKind::VEC:
        // Leave space for 32 GP registers
        return 40 + reg.Index();
    default:
        assert(false && "invalid register kind");
    }
    return 0xffffffff;
}

struct Register {
    /// Whether the parts larger than the first value are also updated to zero.
    bool upperZero;
    struct Value {
        /// "Main" value
        llvm::Value* valueA;
        /// Same value, but with alternative type
        llvm::Value* valueB;
        /// Size in bits
        unsigned size;
        /// Value is more recent than larger values in the register. If set,
        /// accessing the larger values must merge this value.
        bool dirty;
    };
    llvm::SmallVector<Value, 2> values;

    Register() : upperZero(false), values() {}

    void clear() {
        upperZero = false;
        values.clear();
    }
};

class RegFile::impl {
public:
    impl(Arch arch, llvm::BasicBlock* bb)
            : irb(bb), reg_ip(), flags(), dirty_regs(), cleaned_regs() {
        unsigned ngp, nvec, nflags = 7;
        switch (arch) {
#ifdef RELLUME_WITH_X86_64
        case Arch::X86_64: ngp = 16; nvec = 16; ivec_facet = Facet::V2I64; break;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        case Arch::RV64: ngp = 32; nvec = 32; nflags = 0; ivec_facet = Facet::I64; break;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        case Arch::AArch64: ngp = 32; nvec = 32; ivec_facet = Facet::V2I64; break;
#endif // RELLUME_WITH_AARCH64
        default: assert(false);
        }
        regs_gp.resize(ngp);
        regs_sse.resize(nvec);
        flags.resize(nflags);
    }

    llvm::BasicBlock* GetInsertBlock() { return irb.GetInsertBlock(); }
    void SetInsertPoint(llvm::BasicBlock::iterator ip) {
        irb.SetInsertPoint(ip->getParent(), ip);
    }

    void Clear();
    void InitWithRegFile(RegFile* parent) {
        Clear();
        this->parent = parent;
    }
    void InitWithPHIs(std::vector<PhiDesc>* desc_vec) {
        Clear();
        phiDescs = desc_vec;
    }

    llvm::Value* GetReg(ArchReg reg, Facet facet);
    void SetReg(ArchReg reg, Facet facet, llvm::Value*, bool clear_facets);

    RegisterSet& DirtyRegs() { return dirty_regs; }
    RegisterSet& CleanedRegs() { return cleaned_regs; }

private:
    llvm::IRBuilder<> irb;
    llvm::SmallVector<Register, 32> regs_gp;
    llvm::SmallVector<Register, 32> regs_sse;
    Register reg_ip;
    llvm::SmallVector<Register, 7> flags;

    RegFile* parent = nullptr;
    std::vector<PhiDesc>* phiDescs = nullptr;

    Facet ivec_facet;

    RegisterSet dirty_regs;
    RegisterSet cleaned_regs;

    Register* AccessReg(ArchReg reg, Facet facet);
    Facet NativeFacet(ArchReg reg, Facet facet);

    Register::Value* GetRegFold(ArchReg reg, Facet facet, unsigned fullSize);
};

void RegFile::impl::Clear() {
    for (auto& reg : regs_gp)
        reg.clear();
    for (auto& reg : regs_sse)
        reg.clear();
    reg_ip.clear();
    for (auto& reg : flags)
        reg.clear();
}

Register* RegFile::impl::AccessReg(ArchReg reg, Facet facet) {
    unsigned idx = reg.Index();
    switch (reg.Kind()) {
    case ArchReg::RegKind::GP:
        return &regs_gp[idx];
    case ArchReg::RegKind::IP:
        return &reg_ip;
    case ArchReg::RegKind::EFLAGS:
        switch (facet) {
        case Facet::OF: return &flags[0]; break;
        case Facet::SF: return &flags[1]; break;
        case Facet::ZF: return &flags[2]; break;
        case Facet::AF: return &flags[3]; break;
        case Facet::PF: return &flags[4]; break;
        case Facet::CF: return &flags[5]; break;
        case Facet::DF: return &flags[6]; break;
        default: assert(false && "invalid facet for EFLAGS register");
        }
        return nullptr;
    case ArchReg::RegKind::VEC:
        return &regs_sse[idx];
    default:
        return nullptr;
    }
}

Facet RegFile::impl::NativeFacet(ArchReg reg, Facet facet) {
    switch (reg.Kind()) {
    case ArchReg::RegKind::GP:
        return Facet::I64;
    case ArchReg::RegKind::IP:
        return Facet::I64;
    case ArchReg::RegKind::EFLAGS:
        return facet;
    case ArchReg::RegKind::VEC:
        return ivec_facet;
    default:
        assert(false);
        return Facet::I64;
    }
}

Register::Value* RegFile::impl::GetRegFold(ArchReg reg, Facet facet, unsigned fullSize) {
    // Goal: make sure that rv->values[<retvalue>] is at least fullSize and
    // that no dirty values follow after that.
    Register* rv = AccessReg(reg, facet);
    if (!rv->upperZero && (rv->values.empty() || rv->values[0].size < fullSize)) {
        // In future, we may want to support this. For now, it shouldn't happen.
        assert(rv->values.empty() && "missing native facet");
        // We need to get the value, so add a PHI node.
        auto nativeFacet = NativeFacet(reg, facet);
        if (parent) {
            SetReg(reg, nativeFacet, parent->GetReg(reg, nativeFacet), true);
        } else if (phiDescs) {
            llvm::IRBuilder<> phiirb(GetInsertBlock(), GetInsertBlock()->begin());
            auto phi = phiirb.CreatePHI(nativeFacet.Type(irb.getContext()), 4);
            phiDescs->push_back(std::make_tuple(reg, nativeFacet, phi));
            SetReg(reg, nativeFacet, phi, true);
        } else {
            assert(false && "accessing unset register in entry block");
            return nullptr;
        }
    }

    if (rv->values.empty() && rv->upperZero) {
        llvm::Type* intTy = llvm::Type::getIntNTy(irb.getContext(), fullSize);
        llvm::Value* zero = llvm::Constant::getNullValue(intTy);
        rv->values.push_back(Register::Value{zero, nullptr, fullSize, true});
        return &rv->values[0];
    }

    assert(!rv->values.empty() && "undefined upper part of register");

    unsigned foldStartIdx = 0;
    for (unsigned i = 0; i < rv->values.size(); i++) {
        if (rv->values[i].size >= fullSize) {
            foldStartIdx = i;
        } else if (rv->values[i].dirty) {
            goto fold;
        }
    }

    return &rv->values[foldStartIdx];

fold:;
    unsigned foldSize = fullSize < rv->values[0].size ? rv->values[0].size : fullSize;
    // Fold values from foldStartIdx
    assert(false && "value merging not implemented");
    return nullptr;
}

llvm::Value* RegFile::impl::GetReg(ArchReg reg, Facet facet) {
    unsigned facetSize = facet.Size();
    if (facet == Facet::I8H)
        facetSize = 16;
    llvm::Type* facetType = facet.Type(irb.getContext());

    Register::Value* rvv = GetRegFold(reg, facet, facetSize);
    if (facet != Facet::I8H && rvv->size == facetSize) {
        if (rvv->valueA->getType() == facetType)
            return rvv->valueA;
        if (rvv->valueB && rvv->valueB->getType() == facetType)
            return rvv->valueB;

        rvv->valueB = rvv->valueA;
        rvv->valueA = irb.CreateBitOrPointerCast(rvv->valueB, facetType);
        return rvv->valueA;
    }
    llvm::Value* superValue = rvv->valueA;
    llvm::Type* superValueTy = superValue->getType();

    // Need to do arbitrary conversion.
    switch (facet) {
    case Facet::I128:
    case Facet::I64:
    case Facet::I32:
    case Facet::I16:
    case Facet::I8:
    case Facet::F64:
    case Facet::F32:
    case Facet::PTR:
        if (superValueTy->isVectorTy()) {
            int nativeBits = superValueTy->getPrimitiveSizeInBits();
            int nativeCnt = nativeBits / facetType->getPrimitiveSizeInBits();
            if (nativeCnt == 1)
                return irb.CreateBitCast(superValue, facetType);
            auto vecTy = llvm::VectorType::get(facetType, nativeCnt,
                                               /*scalable=*/false);
            auto vec = irb.CreateBitCast(superValue, vecTy);
            return irb.CreateExtractElement(vec, uint64_t{0});
        } else {
            auto val = superValue;
            if (!superValueTy->isIntegerTy())
                val = irb.CreateBitOrPointerCast(superValue, irb.getIntNTy(rvv->size));
            if (rvv->size > facetSize)
                val = irb.CreateTrunc(val, irb.getIntNTy(facetSize));
            return irb.CreateBitOrPointerCast(val, facetType);
        }
        assert(false);
        return nullptr;
    case Facet::I8H:
        assert(superValueTy->isIntegerTy() && "I8H from non-integer type");
        return irb.CreateTrunc(irb.CreateLShr(superValue, irb.getInt64(8)), irb.getInt8Ty());
    case Facet::ZF:
    case Facet::SF:
    case Facet::PF:
    case Facet::CF:
    case Facet::OF:
    case Facet::AF:
    case Facet::DF:
        assert(false && "type mismatch for flag facet");
        return nullptr;
    default: {
        assert(facetType->isVectorTy() && "invalid facet for GetReg");

        auto vec_ty = llvm::cast<llvm::VectorType>(facetType);
        llvm::Type* elem_ty = facetType->getScalarType();
        int targetCnt = vec_ty->getElementCount().getFixedValue();

        int elementBits = elem_ty->getPrimitiveSizeInBits();
        int nativeBits = superValueTy->getPrimitiveSizeInBits();
        int nativeCnt = nativeBits / elementBits;

        // Cast native facet to appropriate type
        auto native_vec_ty = llvm::VectorType::get(elem_ty, nativeCnt,
                                                   /*scalable=*/false);
        auto res = irb.CreateBitCast(superValue, native_vec_ty);

        // If a shorter vector is required, use shufflevector.
        if (nativeCnt > targetCnt) {
            llvm::SmallVector<int, 16> mask;
            for (int i = 0; i < targetCnt; i++)
                mask.push_back(i);
            llvm::Value* undef = llvm::UndefValue::get(native_vec_ty);
            res = irb.CreateShuffleVector(res, undef, mask);
        }
        return res;
    }
    }
}

void RegFile::impl::SetReg(ArchReg reg, Facet facet, llvm::Value* value,
                           bool clearOthers) {
    if (facet == Facet::PTR)
        assert(value->getType()->isPointerTy());
    else
        assert(value->getType() == facet.Type(irb.getContext()));

    if (!clearOthers && reg.Kind() != ArchReg::RegKind::EFLAGS)
        return; // this is just an optimization.

    unsigned facetSize = facet.Size();
    Register* rv = AccessReg(reg, facet);

    // Index of first value that is NOT LARGER than the size we overwrite.
    unsigned mergeValuePoint = 0;
    while (mergeValuePoint < rv->values.size()) {
        if (rv->values[mergeValuePoint].size <= facetSize)
            break;
        mergeValuePoint++;
    }
    rv->values.truncate(mergeValuePoint);
    rv->values.push_back(Register::Value{value, nullptr, facetSize, true});

    dirty_regs[RegisterSetBitIdx(reg, facet)] = true;
}

RegFile::RegFile(Arch arch, llvm::BasicBlock* bb) : pimpl{std::make_unique<impl>(arch, bb)} {}
RegFile::~RegFile() {}

llvm::BasicBlock* RegFile::GetInsertBlock() { return pimpl->GetInsertBlock(); }
void RegFile::SetInsertPoint(llvm::BasicBlock::iterator ip) { pimpl->SetInsertPoint(ip); }
void RegFile::Clear() { pimpl->Clear(); }
void RegFile::InitWithRegFile(RegFile* r) { pimpl->InitWithRegFile(r); }
void RegFile::InitWithPHIs(std::vector<PhiDesc>* d) { pimpl->InitWithPHIs(d); }
llvm::Value* RegFile::GetReg(ArchReg r, Facet f) { return pimpl->GetReg(r, f); }
void RegFile::SetReg(ArchReg reg, Facet facet, llvm::Value* value, bool clear) {
    pimpl->SetReg(reg, facet, value, clear);
}
RegisterSet& RegFile::DirtyRegs() { return pimpl->DirtyRegs(); }
RegisterSet& RegFile::CleanedRegs() { return pimpl->CleanedRegs(); }

} // namespace rellume
