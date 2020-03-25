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

#ifndef RELLUME_FACET_H
#define RELLUME_FACET_H

#include <llvm/IR/Type.h>
#include <cstddef>


namespace rellume {

/**
 * \brief The size of a vector
 **/
#define LL_VECTOR_REGISTER_SIZE 128

class Facet {
public:
    enum Value {
#define SCALAR_INT_FACET(fc, sz, ty) fc,
#define SCALAR_FP_FACET(fc, sz, ty) fc,
#define SPECIAL_FACET(fc, sz, ty) fc,
#define VECTOR_FACET(fc, num, sc) fc,
#define PSEUDO_INT_FACET(fc) fc,
#define PSEUDO_VECTOR_FACET(fc, sc) fc,
#include "facet.inc"
#undef SCALAR_INT_FACET
#undef SCALAR_FP_FACET
#undef SPECIAL_FACET
#undef VECTOR_FACET
#undef PSEUDO_INT_FACET
#undef PSEUDO_VECTOR_FACET

        MAX,
        IVEC = I128,
    };

    static Facet In(unsigned bits);
    static Facet Vnt(unsigned num, Facet scalar);
    static Facet FromType(llvm::Type*);

    unsigned Size() const;
    llvm::Type* Type(llvm::LLVMContext& ctx) const;
    Facet Resolve(unsigned bits) const;

    Facet() = default;
    constexpr Facet(Value value) : value(value) {}
    operator Value() const { return value; }
    explicit operator bool() = delete;
private:
    Value value;
};

} // namespace

#endif
