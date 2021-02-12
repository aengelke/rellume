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

#include "facet.h"

#include <llvm/IR/Type.h>
#include <cassert>


namespace rellume {

Facet Facet::In(unsigned size) {
    switch (size) {
#define SCALAR_INT_FACET(fc, sz, ty) case sz: return fc;
#include "facet.inc"
#undef SCALAR_INT_FACET
    }
    assert(false && "invalid bits for integer facet");
    return MAX;
}

Facet Facet::Vnt(unsigned num_i, Facet scalar) {
#define VECTOR_FACET(fc, num, sc) if (num == num_i && sc == scalar) return fc;
#include "facet.inc"
#undef VECTOR_FACET
    assert(false && "invalid count/type for vector facet");
    return MAX;
}

Facet Facet::FromType(llvm::Type* type) {
    if (type->isVectorTy()) {
        auto num = llvm::cast<llvm::VectorType>(type)->getElementCount();
        return Vnt(num.Min, FromType(type->getScalarType()));
    } else if (type->isIntegerTy()) {
        return In(type->getIntegerBitWidth());
    } else if (type->isFloatTy()) {
        return F32;
    } else if (type->isDoubleTy()) {
        return F64;
    } else {
        assert(false && "invalid type for facet");
        return MAX;
    }
}

unsigned Facet::Size() const {
    switch (*this) {
#define SCALAR_INT_FACET(fc, sz, ty) case Facet::fc: return sz;
#define SCALAR_FP_FACET(fc, sz, ty) case Facet::fc: return sz;
#define SPECIAL_FACET(fc, sz, ty) case Facet::fc: return sz;
#define VECTOR_FACET(fc, num, sc) case Facet::fc: return num * Facet{sc}.Size();
#include "facet.inc"
#undef SCALAR_INT_FACET
#undef SCALAR_FP_FACET
#undef SPECIAL_FACET
#undef VECTOR_FACET
    default:
        assert(false && "Size() called on pseudo-facet");
        return 0;
    }
}

llvm::Type* Facet::Type(llvm::LLVMContext& ctx) const {
    switch (*this) {
#define SCALAR_INT_FACET(fc, sz, ty) case fc: return ty;
#define SCALAR_FP_FACET(fc, sz, ty) case fc: return ty;
#define SPECIAL_FACET(fc, sz, ty) case fc: return ty;
#define VECTOR_FACET(fc, num, sc) \
        case fc: return llvm::VectorType::get(Facet{sc}.Type(ctx), num, false);
#include "facet.inc"
#undef SCALAR_INT_FACET
#undef SCALAR_FP_FACET
#undef SPECIAL_FACET
#undef VECTOR_FACET
    default:
        assert(false && "Type() called on pseudo-facet");
        return nullptr;
    }
}

Facet Facet::Resolve(unsigned bits) const {
    switch (*this) {
#define PSEUDO_INT_FACET(fc) case fc: return In(bits);
#define PSEUDO_VECTOR_FACET(fc, sc) \
        case fc: return Vnt(bits / Facet{sc}.Size(), sc);
#include "facet.inc"
#undef PSEUDO_INT_FACET
#undef PSEUDO_VECTOR_FACET
    default:
        return *this;
    }
}

} // namespace
