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

#ifndef LL_DEFERRED_VALUE_H
#define LL_DEFERRED_VALUE_H

#include "facet.h"
#include "rellume/instr.h"

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Value.h>
#include <type_traits>


namespace rellume {

class DeferredValueBase {
public:
    using Generator = llvm::Value*(*)(X86Reg, Facet, llvm::BasicBlock*, void*);

protected:
    // If value is nullptr, then the generator (unless that is null as well)
    // is used to get the actual value.
    void* values[3];
    Generator generator;

public:
    DeferredValueBase() {}
    DeferredValueBase(llvm::Value* value) : values{value}, generator(nullptr) {}
    explicit DeferredValueBase(Generator generator) : generator(generator) {
        assert(generator && "deferred value with null generator");
    }

    DeferredValueBase(DeferredValueBase&& rhs) = default;
    DeferredValueBase& operator=(DeferredValueBase&& rhs) = default;

    DeferredValueBase(DeferredValueBase const&) = delete;
    DeferredValueBase& operator=(const DeferredValueBase&) = delete;

    llvm::Value* get(X86Reg reg, Facet facet, llvm::BasicBlock* bb) {
        if (generator) {
            values[0] = generator(reg, facet, bb, values);
            assert(values[0] != nullptr && "generator returned nullptr");
            generator = nullptr;
        }
        return static_cast<llvm::Value*>(values[0]);
    }
    explicit operator bool() const {
        return generator || values[0];
    }
};

template<typename T>
class DeferredValue : public DeferredValueBase {
public:
    using Generator = llvm::Value*(*)(X86Reg, Facet, llvm::BasicBlock*, T*);
    DeferredValue(Generator generator, T data)
            : DeferredValueBase(reinterpret_cast<DeferredValueBase::Generator>(generator)) {
        static_assert(std::is_trivially_copyable<T>::value, "invalid defer arg type");
        static_assert(sizeof(T) <= sizeof(values), "defer arg type too big");
        static_assert(alignof(T) <= alignof(void*), "defer arg type misaligned");
        *reinterpret_cast<T*>(values) = data;
    }
};

} // namespace

#endif
