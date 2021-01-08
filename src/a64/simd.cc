/**
 * This file is part of Rellume.
 *
 * (c) 2021, Dominik Okwieka <dominik.okwieka@t-online.de>
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

#include "a64/lifter.h"
#include "a64/lifter-private.h"

#include "arch.h"
#include "facet.h"
#include "instr.h"
#include "regfile.h"

#include <cstdint>

#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>

namespace rellume::aarch64 {

bool Lifter::LiftSIMD(farmdec::Inst a64) {
    bool round = a64.flags & farmdec::SIMD_ROUND;
    bool sgn = a64.flags & farmdec::SIMD_SIGNED;
    bool scalar = a64.flags & farmdec::SIMD_SCALAR;
    farmdec::VectorArrangement va = fad_get_vec_arrangement(a64.flags);
    bool w32 = a64.flags & farmdec::W32;

    // In the order of farmdec::Op.
    switch (a64.op) {
    case farmdec::A64_DUP_ELEM: {
        auto elem = GetElem(a64.rn, va, a64.imm);
        if (scalar) {
            SetScalar(a64.rd, elem);
            break;
        }
        Dup(a64.rd, va, elem);
        break;
    }
    case farmdec::A64_DUP_GPR: {
        auto elem = irb.CreateTruncOrBitCast(GetGp(a64.rn, w32), ElemTypeOf(va));
        Dup(a64.rd, va, elem);
        break;
    }
    case farmdec::A64_INS_ELEM: {
        auto elem = GetElem(a64.rn, va, a64.ins_elem.src);
        InsertElem(a64.rd, a64.ins_elem.dst, elem);
        break;
    }
    case farmdec::A64_INS_GPR: {
        auto elem = irb.CreateTruncOrBitCast(GetGp(a64.rn, w32), ElemTypeOf(va));
        InsertElem(a64.rd, a64.imm, elem);
        break;
    }
    case farmdec::A64_SMOV: {
        auto elem = GetElem(a64.rn, va, a64.imm);
        SetGp(a64.rd, w32, irb.CreateSExt(elem, (w32) ? irb.getInt32Ty() : irb.getInt64Ty()));
        break;
    }
    case farmdec::A64_UMOV: {
        auto elem = GetElem(a64.rn, va, a64.imm);
        SetGp(a64.rd, w32, irb.CreateZExt(elem, (w32) ? irb.getInt32Ty() : irb.getInt64Ty()));
        break;
    }
    default:
        return false;
    }

    return true;
}

llvm::Value* Lifter::GetVec(farmdec::Reg r, farmdec::VectorArrangement va, bool fp) {
    return GetReg(ArchReg::VEC(r), FacetOf(va, fp));
}

llvm::Value* Lifter::GetElem(farmdec::Reg r, farmdec::VectorArrangement va, unsigned i, bool fp) {
    return irb.CreateExtractElement(GetVec(r, va, fp), i);
}

void Lifter::SetVec(farmdec::Reg r, llvm::Value* vec) {
    auto vecty = llvm::cast<llvm::VectorType>(vec->getType());
    Facet fc = Facet::FromType(vecty);
    unsigned bits = fc.Size();
    Facet ivec = Facet::V2I64;

    // Full 128-bit vector
    if (bits == ivec.Size()) {
        SetReg(ArchReg::VEC(r), ivec, irb.CreateBitCast(vec, ivec.Type(irb.getContext())));
        SetRegFacet(ArchReg::VEC(r), fc, vec);
        return;
    }

    // Half vector -- upper half is zeroed.
    auto fullty = llvm::VectorType::getDoubleElementsVectorType(vecty);
    auto zero = llvm::Constant::getNullValue(vecty);

    // Shufflevector (zero, vec, {0, 1, ..., n}) simply concatenates zero and vec.
    auto full_nelem = 2 * bits / vecty->getScalarSizeInBits();
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < full_nelem; i++) {
        mask.push_back(i);
    }
    auto fullvec = irb.CreateShuffleVector(vec, zero, mask);

    SetReg(ArchReg::VEC(r), ivec, irb.CreateBitCast(fullvec, ivec.Type(irb.getContext())));
    SetRegFacet(ArchReg::VEC(r), Facet::FromType(fullty), fullvec);
    SetRegFacet(ArchReg::VEC(r), fc, vec);
}

// Vr[i] := elem, l without touching other lanes.
void Lifter::InsertElem(farmdec::Reg r, unsigned i, llvm::Value* elem) {
    auto elemty = elem->getType();

    farmdec::VectorArrangement va;
    switch(elemty->getPrimitiveSizeInBits()) {
    case  8: va = farmdec::VA_16B; break;
    case 16: va = farmdec::VA_8H; break;
    case 32: va = farmdec::VA_4S; break;
    case 64: va = farmdec::VA_2D; break;
    default:
        assert(false && "bad element bit count");
    }

    auto oldvec = GetVec(r, va, elemty->isFPOrFPVectorTy());
    auto newvec = irb.CreateInsertElement(oldvec, elem, i);
    SetVec(r, newvec);
}

// Duplicate elem to all lanes.
void Lifter::Dup(farmdec::Reg r, farmdec::VectorArrangement va, llvm::Value* elem) {
    SetVec(r, irb.CreateVectorSplat(NumElem(va), elem));
}

unsigned Lifter::NumElem(farmdec::VectorArrangement va) {
    switch (va) {
    case farmdec::VA_8B:  return 8;
    case farmdec::VA_16B: return 16;
    case farmdec::VA_4H:  return 4;
    case farmdec::VA_8H:  return 8;
    case farmdec::VA_2S:  return 2;
    case farmdec::VA_4S:  return 4;
    case farmdec::VA_1D:  return 1;
    case farmdec::VA_2D:  return 2;
    }
    assert(false && "invalid vector arrangement");
}

Facet Lifter::FacetOf(farmdec::VectorArrangement va, bool fp) {
    switch (va) {
    case farmdec::VA_8B:  return Facet::Vnt(8, Facet::I8);
    case farmdec::VA_16B: return Facet::Vnt(16, Facet::I8);
    case farmdec::VA_4H:  return Facet::Vnt(4, Facet::I16);
    case farmdec::VA_8H:  return Facet::Vnt(8, Facet::I16);
    case farmdec::VA_2S:  return Facet::Vnt(2, (fp) ? Facet::F32 : Facet::I32);
    case farmdec::VA_4S:  return Facet::Vnt(4, (fp) ? Facet::F32 : Facet::I32);
    case farmdec::VA_1D:  return Facet::Vnt(1, (fp) ? Facet::F64 : Facet::I64);
    case farmdec::VA_2D:  return Facet::Vnt(2, (fp) ? Facet::F64 : Facet::I64);
    }
    assert(false && "invalid vector arrangement");
}

llvm::Type* Lifter::TypeOf(farmdec::VectorArrangement va, bool fp) {
    return FacetOf(va, fp).Type(irb.getContext());
}

llvm::Type* Lifter::ElemTypeOf(farmdec::VectorArrangement va, bool fp) {
    return llvm::cast<llvm::VectorType>(TypeOf(va, fp))->getElementType();
}

} // namespace rellume::aarch64

/**
 * @}
 **/
