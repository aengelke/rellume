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

unsigned RegisterSetBitIdx(ArchReg reg) {
    switch (reg.Kind()) {
    case ArchReg::RegKind::IP:
        return 0;
    case ArchReg::RegKind::FLAG:
        return 1 + reg.Index();
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
        llvm::Value* valueC;
        /// Size in bits
        unsigned size;
        /// Value is more recent than larger values in the register. If set,
        /// accessing the larger values must merge this value.
        bool dirty;
        /// Transformation to perform
        RegFile::Transform transform;

        Value(llvm::Value* value, unsigned size)
            : valueA(value), valueB(nullptr), size(size), dirty(true), transform(RegFile::Transform::None) {}
        Value(RegFile::Transform t, llvm::Value* v1, llvm::Value* v2, llvm::Value* v3, unsigned size)
            : valueA(v1), valueB(v2), valueC(v3), size(size), dirty(true), transform(t) {}

        llvm::Value* value() const {
            assert(transform == RegFile::Transform::None);
            return valueA;
        }
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
            : irb(bb), reg_ip(), flags(), dirty_regs() {
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
    void SetReg(ArchReg reg, llvm::Value*, WriteMode mode);
    void Set(ArchReg reg, Transform transform, llvm::Value* v1, llvm::Value* v2, llvm::Value* v3);

    RegisterSet& DirtyRegs() { return dirty_regs; }
    bool StartsClean() { return !parent && !phiDescs; }

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

    Register* AccessReg(ArchReg reg);
    Facet NativeFacet(ArchReg reg);

    Register::Value* GetRegFold(ArchReg reg, unsigned fullSize);

    void CanonicalizeRegisterValue(Register::Value& rvv);
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

Register* RegFile::impl::AccessReg(ArchReg reg) {
    unsigned idx = reg.Index();
    switch (reg.Kind()) {
    case ArchReg::RegKind::GP:
        return &regs_gp[idx];
    case ArchReg::RegKind::IP:
        return &reg_ip;
    case ArchReg::RegKind::FLAG:
        return &flags[idx];
    case ArchReg::RegKind::VEC:
        return &regs_sse[idx];
    default:
        return nullptr;
    }
}

Facet RegFile::impl::NativeFacet(ArchReg reg) {
    switch (reg.Kind()) {
    case ArchReg::RegKind::GP:
        return Facet::I64;
    case ArchReg::RegKind::IP:
        return Facet::I64;
    case ArchReg::RegKind::FLAG:
        if (reg == ArchReg::PF)
            return Facet::I8;
        return Facet::I1;
    case ArchReg::RegKind::VEC:
        return ivec_facet;
    default:
        assert(false);
        return Facet::I64;
    }
}

void RegFile::impl::CanonicalizeRegisterValue(Register::Value& rvv) {
    switch (rvv.transform) {
    case Transform::None:
        break;
    case Transform::IsZero:
        rvv.valueA = irb.CreateIsNull(rvv.valueA);
        rvv.valueB = nullptr;
        rvv.valueC = nullptr;
        break;
    case Transform::IsNeg:
        rvv.valueA = irb.CreateIsNeg(rvv.valueA);
        rvv.valueB = nullptr;
        rvv.valueC = nullptr;
        break;
    case Transform::TruncI8:
        rvv.valueA = irb.CreateTrunc(rvv.valueA, irb.getInt8Ty());
        rvv.valueB = nullptr;
        rvv.valueC = nullptr;
        break;
    case Transform::Load:
        rvv.valueA = irb.CreateLoad(rvv.valueB->getType(), rvv.valueA);
        rvv.valueB = nullptr;
        rvv.valueC = nullptr;
        break;
    case Transform::X86AuxFlag: {
        llvm::Value* tmp = irb.CreateXor(irb.CreateXor(rvv.valueB, rvv.valueC), rvv.valueA);
        llvm::Value* masked = irb.CreateAnd(tmp, llvm::ConstantInt::get(tmp->getType(), 16));
        rvv.valueA = irb.CreateICmpNE(masked, llvm::Constant::getNullValue(tmp->getType()));;
        rvv.valueB = nullptr;
        rvv.valueC = nullptr;
        break;
    }
    default:
        assert(false);
    }
    rvv.transform = Transform::None;
}

Register::Value* RegFile::impl::GetRegFold(ArchReg reg, unsigned fullSize) {
    // Goal: make sure that rv->values[<retvalue>] is at least fullSize and
    // that no dirty values follow after that.
    Register* rv = AccessReg(reg);
    if (!rv->upperZero && (rv->values.empty() || rv->values[0].size < fullSize)) {
        // We need to get the value, so add a PHI node.
        auto nativeFacet = NativeFacet(reg);
        llvm::Value* nativeVal = nullptr;
        if (parent) {
            nativeVal = parent->GetReg(reg, nativeFacet);
        } else if (phiDescs) {
            llvm::IRBuilder<> phiirb(GetInsertBlock(), GetInsertBlock()->begin());
            auto phi = phiirb.CreatePHI(nativeFacet.Type(irb.getContext()), 4);
            phiDescs->push_back(std::make_tuple(reg, nativeFacet, phi));
            nativeVal = phi;
        } else {
            assert(false && "accessing unset register in entry block");
            return nullptr;
        }
        Register::Value nativeRvv(nativeVal, nativeFacet.Size());
        nativeRvv.dirty = false;
        rv->values.insert(rv->values.begin(), std::move(nativeRvv));
    }

    if (rv->values.empty() && rv->upperZero) {
        llvm::Type* intTy = llvm::Type::getIntNTy(irb.getContext(), fullSize);
        llvm::Value* zero = llvm::Constant::getNullValue(intTy);
        rv->values.push_back(Register::Value(zero, fullSize));
        return &rv->values[0];
    }

    assert(!rv->values.empty() && "undefined upper part of register");
    assert(!rv->upperZero || rv->values[0].dirty);

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
    CanonicalizeRegisterValue(rv->values[0]);
    llvm::Value* result = rv->values[0].value();

    // Always convert pointers to integers, first.
    if (result->getType()->isPointerTy())
        result = irb.CreateBitCast(result, irb.getIntNTy(rv->values[0].size));

    if (result->getType()->isIntegerTy()) {
        llvm::Type* targetTy = irb.getIntNTy(foldSize);
        result = irb.CreateZExtOrTrunc(result, targetTy);
        for (unsigned i = 1; i < rv->values.size(); i++) {
            if (!rv->values[i].dirty)
                continue;
            unsigned size = rv->values[i].size;
            CanonicalizeRegisterValue(rv->values[i]);
            auto value = irb.CreateBitCast(rv->values[i].value(), irb.getIntNTy(size));
            value = irb.CreateZExt(value, targetTy);
            auto mask = llvm::APInt::getHighBitsSet(foldSize, foldSize - size);
            result = irb.CreateOr(irb.CreateAnd(result, mask), value);
        }
    } else {
        llvm::errs() << *result << "\n";
        assert(false && "non-integer value merging not implemented");
        return nullptr;
    }

    // TODO: could keep smallest dirty value and all following clean as clean.
    rv->values.clear();
    rv->values.push_back(Register::Value(result, foldSize));

    return &rv->values[0];
}

llvm::Value* RegFile::impl::GetReg(ArchReg reg, Facet facet) {
    unsigned facetSize = facet.Size();
    if (facet == Facet::I8H)
        facetSize = 16;
    llvm::Type* facetType = facet.Type(irb.getContext());

    Register::Value* rvv = GetRegFold(reg, facetSize);
    CanonicalizeRegisterValue(*rvv);
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

void RegFile::impl::SetReg(ArchReg reg, llvm::Value* value, WriteMode mode) {
    if (mode == RegFile::EXTRA_PART)
        return; // this is just an optimization.

    unsigned size;
    if (value->getType()->isPointerTy())
        size = 64;
    else
        size = value->getType()->getPrimitiveSizeInBits();
    assert(size != 0);
    Register* rv = AccessReg(reg);

    switch (mode) {
    case RegFile::INTO_ZERO:
        rv->upperZero = true;
        rv->values.clear();
        rv->values.push_back(Register::Value(value, size));
        break;
    case RegFile::MERGE: {
        // Index of first value that is NOT LARGER than the size we overwrite.
        unsigned mergeValuePoint = 0;
        while (mergeValuePoint < rv->values.size()) {
            if (rv->values[mergeValuePoint].size <= size)
                break;
            mergeValuePoint++;
        }
        rv->values.truncate(mergeValuePoint);
        rv->values.push_back(Register::Value(value, size));
        break;
    }
    default:
        assert(false);
    }

    dirty_regs[RegisterSetBitIdx(reg)] = true;
}

void RegFile::impl::Set(ArchReg reg, Transform transform, llvm::Value* v1,
                        llvm::Value* v2, llvm::Value* v3) {
    unsigned size = 0;
    switch (transform) {
    case Transform::None:
        size = v1->getType()->getPrimitiveSizeInBits();
        break;
    case Transform::IsZero:
    case Transform::IsNeg:
    case Transform::X86AuxFlag:
        size = 1;
        break;
    case Transform::TruncI8:
        size = 8;
        break;
    case Transform::Load:
        size = v2->getType()->getPrimitiveSizeInBits();
        assert(size != 0 && "maybe pointer type for Transform::Load?");
        break;
    default:
        assert(false);
    }

    Register* rv = AccessReg(reg);
    rv->values.clear();
    rv->values.push_back(Register::Value(transform, v1, v2, v3, size));
    dirty_regs[RegisterSetBitIdx(reg)] = true;
}

RegFile::RegFile(Arch arch, llvm::BasicBlock* bb) : pimpl{std::make_unique<impl>(arch, bb)} {}
RegFile::~RegFile() {}

llvm::BasicBlock* RegFile::GetInsertBlock() { return pimpl->GetInsertBlock(); }
void RegFile::SetInsertPoint(llvm::BasicBlock::iterator ip) { pimpl->SetInsertPoint(ip); }
void RegFile::Clear() { pimpl->Clear(); }
void RegFile::InitWithRegFile(RegFile* r) { pimpl->InitWithRegFile(r); }
void RegFile::InitWithPHIs(std::vector<PhiDesc>* d) { pimpl->InitWithPHIs(d); }
llvm::Value* RegFile::GetReg(ArchReg r, Facet f) { return pimpl->GetReg(r, f); }
void RegFile::SetReg(ArchReg reg, llvm::Value* value, WriteMode mode) {
    pimpl->SetReg(reg, value, mode);
}
void RegFile::Set(ArchReg reg, Transform t, llvm::Value* v1, llvm::Value* v2, llvm::Value* v3) {
    pimpl->Set(reg, t, v1, v2, v3);
}
RegisterSet& RegFile::DirtyRegs() { return pimpl->DirtyRegs(); }
bool RegFile::StartsClean() { return pimpl->StartsClean(); }

} // namespace rellume
