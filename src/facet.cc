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

Facet Facet::Resolve(unsigned bits)
{
    switch (*this)
    {
    case Facet::I:
        if (bits == 8) return Facet::I8;
        if (bits == 16) return Facet::I16;
        if (bits == 32) return Facet::I32;
        if (bits == 64) return Facet::I64;
        assert(false && "invalid bits for integer facet");
        break;
    case Facet::VI8:
        if (bits == 8) return Facet::V1I8;
        if (bits == 16) return Facet::V2I8;
        if (bits == 32) return Facet::V4I8;
        if (bits == 64) return Facet::V8I8;
        if (bits == 128) return Facet::V16I8;
        assert(false && "invalid bits for integer facet");
        break;
    case Facet::VI16:
        if (bits == 16) return Facet::V1I16;
        if (bits == 32) return Facet::V2I16;
        if (bits == 64) return Facet::V4I16;
        if (bits == 128) return Facet::V8I16;
        assert(false && "invalid bits for integer facet");
        break;
    case Facet::VI32:
        if (bits == 32) return Facet::V1I32;
        if (bits == 64) return Facet::V2I32;
        if (bits == 128) return Facet::V4I32;
        assert(false && "invalid bits for integer facet");
        break;
    case Facet::VI64:
        if (bits == 64) return Facet::V1I64;
        if (bits == 128) return Facet::V2I64;
        assert(false && "invalid bits for integer facet");
        break;
    case Facet::VF32:
        if (bits == 32) return Facet::V1F32;
        if (bits == 64) return Facet::V2F32;
        if (bits == 128) return Facet::V4F32;
        assert(false && "invalid bits for integer facet");
        break;
    case Facet::VF64:
        if (bits == 64) return Facet::V1F64;
        if (bits == 128) return Facet::V2F64;
        assert(false && "invalid bits for integer facet");
        break;
    default:
        return *this;
    }
}

llvm::Type* Facet::Type(llvm::LLVMContext& ctx)
{
    switch (*this)
    {
    case Facet::I64: return llvm::Type::getInt64Ty(ctx);
    case Facet::I32: return llvm::Type::getInt32Ty(ctx);
    case Facet::I16: return llvm::Type::getInt16Ty(ctx);
    case Facet::I8: return llvm::Type::getInt8Ty(ctx);
    case Facet::I8H: return llvm::Type::getInt8Ty(ctx);
    case Facet::PTR: return llvm::Type::getInt8PtrTy(ctx);
    case Facet::I128: return llvm::Type::getInt128Ty(ctx);
#if LL_VECTOR_REGISTER_SIZE >= 256
    case Facet::I256: return llvm::Type::getIntNTy(ctx, 256);
#endif
    case Facet::F32: return llvm::Type::getFloatTy(ctx);
    case Facet::F64: return llvm::Type::getDoubleTy(ctx);
    case Facet::V1I8: return llvm::VectorType::get(llvm::Type::getInt8Ty(ctx), 1);
    case Facet::V2I8: return llvm::VectorType::get(llvm::Type::getInt8Ty(ctx), 2);
    case Facet::V4I8: return llvm::VectorType::get(llvm::Type::getInt8Ty(ctx), 4);
    case Facet::V8I8: return llvm::VectorType::get(llvm::Type::getInt8Ty(ctx), 8);
    case Facet::V16I8: return llvm::VectorType::get(llvm::Type::getInt8Ty(ctx), 16);
    case Facet::V1I16: return llvm::VectorType::get(llvm::Type::getInt16Ty(ctx), 1);
    case Facet::V2I16: return llvm::VectorType::get(llvm::Type::getInt16Ty(ctx), 2);
    case Facet::V4I16: return llvm::VectorType::get(llvm::Type::getInt16Ty(ctx), 4);
    case Facet::V8I16: return llvm::VectorType::get(llvm::Type::getInt16Ty(ctx), 8);
    case Facet::V1I32: return llvm::VectorType::get(llvm::Type::getInt32Ty(ctx), 1);
    case Facet::V2I32: return llvm::VectorType::get(llvm::Type::getInt32Ty(ctx), 2);
    case Facet::V4I32: return llvm::VectorType::get(llvm::Type::getInt32Ty(ctx), 4);
    case Facet::V1I64: return llvm::VectorType::get(llvm::Type::getInt64Ty(ctx), 1);
    case Facet::V2I64: return llvm::VectorType::get(llvm::Type::getInt64Ty(ctx), 2);
    case Facet::V1F32: return llvm::VectorType::get(llvm::Type::getFloatTy(ctx), 1);
    case Facet::V2F32: return llvm::VectorType::get(llvm::Type::getFloatTy(ctx), 2);
    case Facet::V4F32: return llvm::VectorType::get(llvm::Type::getFloatTy(ctx), 4);
    case Facet::V1F64: return llvm::VectorType::get(llvm::Type::getDoubleTy(ctx), 1);
    case Facet::V2F64: return llvm::VectorType::get(llvm::Type::getDoubleTy(ctx), 2);
    case Facet::ZF:
    case Facet::SF:
    case Facet::PF:
    case Facet::CF:
    case Facet::OF:
    case Facet::AF: return llvm::Type::getInt1Ty(ctx);
    default: assert(0);
    }

    return nullptr;
}

} // namespace
