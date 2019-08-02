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
        I64,
        I32, I16, I8, I8H, PTR,

        I128,
        V1I8, V2I8, V4I8, V8I8, V16I8,
        V1I16, V2I16, V4I16, V8I16,
        V1I32, V2I32, V4I32,
        V1I64, V2I64,
        V1F32, V2F32, V4F32,
        V1F64, V2F64,
        F32, F64,
#if LL_VECTOR_REGISTER_SIZE >= 256
        I256,
#endif

        // Flags
        ZF, SF, PF, CF, OF, AF,

        // Pseudo-facets
        I, VI8, VI16, VI32, VI64, VF32, VF64,
        MAX,

#if LL_VECTOR_REGISTER_SIZE == 128
        IVEC = I128,
#elif LL_VECTOR_REGISTER_SIZE == 256
        IVEC = I256,
#endif
    };

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
