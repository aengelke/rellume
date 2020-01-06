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

#ifndef RELLUME_FUNCTION_INFO_H
#define RELLUME_FUNCTION_INFO_H

#include <cstdbool>
#include <cstdint>


namespace llvm {
class Function;
class Value;
}

namespace rellume {

namespace SptrIdx {
    enum Idx {
#define RELLUME_NAMED_REG(name,nameu,sz,off) nameu,
#include <rellume/cpustruct-private.inc>
#undef RELLUME_NAMED_REG
        MAX
    };

}

struct FunctionInfo {
    /// The function itself
    llvm::Function* fn;
    /// The sptr argument, and its elements
    llvm::Value* sptr_raw;
    llvm::Value* sptr[SptrIdx::MAX];

private:
    llvm::Value* GetSptrPtr(llvm::IRBuilder<> irb, size_t offset, size_t size) {
        llvm::Type* ptr_ty = irb.getIntNTy(size)->getPointerTo();
        llvm::Value* ptr = irb.CreateConstGEP1_64(sptr_raw, offset);
        return irb.CreatePointerCast(ptr, ptr_ty);
    }

public:
    void InitSptr(llvm::IRBuilder<> irb) {
#define RELLUME_NAMED_REG(name,nameu,sz,off) \
        sptr[SptrIdx::nameu] = GetSptrPtr(irb, off, sz == 1 ? sz : sz * 8);
#include <rellume/cpustruct-private.inc>
#undef RELLUME_NAMED_REG
    }
};


} // namespace

#endif
