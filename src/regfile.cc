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
    /// Transformation to perform
    RegFile::Transform transform;
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

        Value(llvm::Value* value, unsigned size)
            : valueA(value), valueB(nullptr), size(size), dirty(true) {}
        Value(llvm::Value* valueA, llvm::Value* valueB, unsigned size)
            : valueA(valueA), valueB(valueB), size(size), dirty(true) {}

        llvm::Value* value() const {
            return valueA;
        }
    };
    llvm::SmallVector<Value, 2> values;

    Register() : upperZero(false), transform(RegFile::Transform::None), values() {}
    Register(RegFile::Transform t, llvm::Value* v1, llvm::Value* v2, llvm::Value* v3);

    void canonicalize(llvm::IRBuilder<>& irb);

    void clear() {
        upperZero = false;
        transform = RegFile::Transform::None;
        values.clear();
    }
};

Register::Register(RegFile::Transform t, llvm::Value* v1, llvm::Value* v2, llvm::Value* v3) {
    assert(t != RegFile::Transform::None);
    unsigned size = 0;
    switch (t) {
    case RegFile::Transform::IsZero:
    case RegFile::Transform::IsNeg:
    case RegFile::Transform::X86AuxFlag:
        size = 1;
        break;
    case RegFile::Transform::TruncI8:
        size = 8;
        break;
    case RegFile::Transform::Load:
        size = v2->getType()->getPrimitiveSizeInBits();
        assert(size != 0 && "maybe pointer type for Transform::Load?");
        break;
    case RegFile::Transform::None:
        assert(false);
    }

    upperZero = true;
    transform = t;
    values.push_back(Value(v1, v2, size));
    values.push_back(Value(v3, size));
}

void Register::canonicalize(llvm::IRBuilder<>& irb) {
    if (transform == RegFile::Transform::None)
        return;
    assert(upperZero && values.size() == 2);
    auto v1 = values[0].valueA;
    auto v2 = values[0].valueB;
    auto v3 = values[1].valueA;
    values.truncate(1);
    values[0].valueA = nullptr;
    values[0].valueB = nullptr;
    switch (transform) {
    case RegFile::Transform::IsZero:
        values[0].valueA = irb.CreateIsNull(v1);
        break;
    case RegFile::Transform::IsNeg:
        values[0].valueA = irb.CreateIsNeg(v1);
        break;
    case RegFile::Transform::TruncI8:
        values[0].valueA = irb.CreateTrunc(v1, irb.getInt8Ty());
        break;
    case RegFile::Transform::Load:
        values[0].valueA = irb.CreateLoad(v2->getType(), v1);
        break;
    case RegFile::Transform::X86AuxFlag: {
        llvm::Value* tmp = irb.CreateXor(irb.CreateXor(v2, v3), v1);
        llvm::Value* masked = irb.CreateAnd(tmp, llvm::ConstantInt::get(tmp->getType(), 16));
        values[0].valueA = irb.CreateICmpNE(masked, llvm::Constant::getNullValue(tmp->getType()));;
        break;
    }
    case RegFile::Transform::None:
        assert(false);
    }
    transform = RegFile::Transform::None;
}

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

    /// Prepare register for read access of the lowest fullSize bits, merging
    /// dirty and missing parts if necessary. Returns the index into register
    /// values with the same or next larger size.
    std::pair<Register*, unsigned> GetRegFold(ArchReg reg, unsigned fullSize);
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

std::pair<Register*, unsigned> RegFile::impl::GetRegFold(ArchReg reg, unsigned fullSize) {
    // Goal: make sure that rv->values[<retvalue>] is at least fullSize and
    // that no dirty values follow after that.
    Register* rv = AccessReg(reg);
    rv->canonicalize(irb);
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
            return std::make_pair(nullptr, 0);
        }
        Register::Value nativeRvv(nativeVal, nativeFacet.Size());
        nativeRvv.dirty = false;
        rv->values.insert(rv->values.begin(), std::move(nativeRvv));
    }

    if (rv->values.empty() && rv->upperZero) {
        llvm::Type* intTy = llvm::Type::getIntNTy(irb.getContext(), fullSize);
        llvm::Value* zero = llvm::Constant::getNullValue(intTy);
        rv->values.push_back(Register::Value(zero, fullSize));
        return std::make_pair(rv, 0);
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

    return std::make_pair(rv, foldStartIdx);

fold:;
    unsigned foldSize = fullSize < rv->values[0].size ? rv->values[0].size : fullSize;
    llvm::Value* result = rv->values[0].value();

    // Always convert pointers to integers, first.
    // TODO: use vectors for floating-point?
    if (result->getType()->isPointerTy() || result->getType()->isFloatingPointTy())
        result = irb.CreateBitCast(result, irb.getIntNTy(rv->values[0].size));

    if (result->getType()->isIntegerTy()) {
        llvm::Type* targetTy = irb.getIntNTy(foldSize);
        result = irb.CreateZExtOrTrunc(result, targetTy);
        for (unsigned i = 1; i < rv->values.size(); i++) {
            if (!rv->values[i].dirty)
                continue;
            unsigned size = rv->values[i].size;
            auto value = irb.CreateBitCast(rv->values[i].value(), irb.getIntNTy(size));
            value = irb.CreateZExt(value, targetTy);
            auto mask = llvm::APInt::getHighBitsSet(foldSize, foldSize - size);
            result = irb.CreateOr(irb.CreateAnd(result, mask), value);
        }
    } else if (result->getType()->isVectorTy()) {
        unsigned resultSize = result->getType()->getPrimitiveSizeInBits();
        for (unsigned i = 1; i < rv->values.size(); i++) {
            if (!rv->values[i].dirty)
                continue;
            auto value = rv->values[i].value();
            if (!value->getType()->isVectorTy()) {
                unsigned cnt = resultSize / rv->values[i].size;
                auto vecTy = llvm::VectorType::get(value->getType(), cnt, false);
                result = irb.CreateBitCast(result, vecTy);
                result = irb.CreateInsertElement(result, value, uint64_t{0});
            } else {
                auto valueTy = llvm::cast<llvm::VectorType>(value->getType());
                auto elementTy = valueTy->getScalarType();
                unsigned resultCnt = resultSize / elementTy->getPrimitiveSizeInBits();

                // Vector-in-vector insertion require 2 x shufflevector.
                // First, we enlarge the input vector to the full length.
                unsigned valueCnt = valueTy->getElementCount().getFixedValue();
                llvm::SmallVector<int, 16> mask;
                for (unsigned j = 0; j < resultCnt; j++)
                    mask.push_back(j < valueCnt ? j : valueCnt);
                llvm::Value* zero = llvm::Constant::getNullValue(valueTy);
                llvm::Value* ext_vec = irb.CreateShuffleVector(value, zero, mask);

                // Now shuffle the two vectors together
                for (unsigned j = 0; j < resultCnt; j++)
                    mask[j] = j + (j < valueCnt ? 0 : resultCnt);
                auto vecTy = llvm::VectorType::get(elementTy, resultCnt, false);
                result = irb.CreateBitCast(result, vecTy);
                result = irb.CreateShuffleVector(ext_vec, result, mask);
            }
        }

        if (resultSize < foldSize) {
            auto resultTy = llvm::cast<llvm::VectorType>(result->getType());
            unsigned resultCnt = resultTy->getElementCount().getFixedValue();
            auto elemSize = resultTy->getScalarSizeInBits();
            unsigned cnt = foldSize / elemSize;

            llvm::SmallVector<int, 16> mask;
            for (unsigned j = 0; j < cnt; j++)
                mask.push_back(j < resultCnt ? j : resultCnt);
            llvm::Value* zero = llvm::Constant::getNullValue(resultTy);
            result = irb.CreateShuffleVector(result, zero, mask);
        }
    } else {
        llvm::errs() << *result << "\n";
        assert(false && "non-integer/vector value merging not implemented");
        return std::make_pair(nullptr, 0);
    }

    // TODO: could keep smallest dirty value and all following clean as clean.
    rv->values.clear();
    rv->values.push_back(Register::Value(result, foldSize));

    return std::make_pair(rv, 0);
}

llvm::Value* RegFile::impl::GetReg(ArchReg reg, Facet facet) {
    unsigned facetSize = facet.Size();
    if (facet == Facet::I8H)
        facetSize = 16;
    llvm::Type* facetType = facet.Type(irb.getContext());

    auto [rv, rvIdx] = GetRegFold(reg, facetSize);
    Register::Value* rvv = &rv->values[rvIdx];
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
    llvm::Value* res = nullptr;

    // Need to do arbitrary conversion.
    switch (facet) {
    case Facet::PTR: {
        // Weird case, pointer from integer register. Should never happen.
        assert(false && "pointer from non-general-purpose register?");
        auto val = superValue;
        if (!superValueTy->isIntegerTy())
            val = irb.CreateBitOrPointerCast(superValue, irb.getIntNTy(rvv->size));
        val = irb.CreateZExtOrTrunc(val, irb.getIntNTy(facetSize));
        return irb.CreatePointerCast(val, facetType);
    }
    case Facet::I128:
    case Facet::I64:
    case Facet::I32:
    case Facet::I16:
    case Facet::I8:
    case Facet::F64:
    case Facet::F32:
        if (superValueTy->isVectorTy()) {
            int nativeBits = superValueTy->getPrimitiveSizeInBits();
            int nativeCnt = nativeBits / facetType->getPrimitiveSizeInBits();
            if (nativeCnt == 1) {
                res = irb.CreateBitCast(superValue, facetType);
            } else {
                auto vecTy = llvm::VectorType::get(facetType, nativeCnt,
                                                   /*scalable=*/false);
                auto vec = irb.CreateBitCast(superValue, vecTy);
                res = irb.CreateExtractElement(vec, uint64_t{0});
            }
        } else {
            auto val = superValue;
            if (!superValueTy->isIntegerTy())
                val = irb.CreateBitOrPointerCast(superValue, irb.getIntNTy(rvv->size));
            if (rvv->size > facetSize)
                val = irb.CreateTrunc(val, irb.getIntNTy(facetSize));
            res = irb.CreateBitOrPointerCast(val, facetType);
        }
        break;
    case Facet::I8H:
        assert(superValueTy->isIntegerTy() && "I8H from non-integer type");
        // Return directly, don't attempt to cache I8H facet.
        // TODO: caching I8H facets might give performance improvements.
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
        res = irb.CreateBitCast(superValue, native_vec_ty);

        // If a shorter vector is required, use shufflevector.
        if (nativeCnt > targetCnt) {
            llvm::SmallVector<int, 16> mask;
            for (int i = 0; i < targetCnt; i++)
                mask.push_back(i);
            llvm::Value* undef = llvm::UndefValue::get(native_vec_ty);
            res = irb.CreateShuffleVector(res, undef, mask);
        }
        break;
    }
    }

    // Cache value if we can do so without a new heap allocation.
    if (rv->values.size() < rv->values.capacity()) {
        unsigned pos = rvIdx + 1;
        assert(pos == rv->values.size() || rv->values[pos].size < facetSize);
        rv->values.insert(rv->values.begin() + pos, Register::Value(res, facetSize));
        rv->values[pos].dirty = false; // this is the same value
    }

    return res;
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
        rv->clear();
        rv->upperZero = true;
        rv->values.push_back(Register::Value(value, size));
        break;
    case RegFile::MERGE: {
        rv->canonicalize(irb);
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
    *AccessReg(reg) = Register(transform, v1, v2, v3);
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
