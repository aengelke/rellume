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

#ifndef LL_REGFILE_H
#define LL_REGFILE_H

#include "deferred-value.h"
#include "facet.h"
#include "rellume/instr.h"

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Value.h>
#include <functional>


namespace rellume {

class RegFile
{
public:
    RegFile();
    ~RegFile();

    RegFile(RegFile&& rhs);
    RegFile& operator=(RegFile&& rhs);

    RegFile(const RegFile&) = delete;
    RegFile& operator=(const RegFile&) = delete;

    llvm::BasicBlock* GetInsertBlock();
    void SetInsertBlock(llvm::BasicBlock* new_block);

    void Clear();
    using InitGenerator = std::function<DeferredValue(const X86Reg, const Facet)>;
    void InitAll(InitGenerator init_gen = nullptr);

    llvm::Value* GetReg(X86Reg reg, Facet facet);
    void SetReg(X86Reg reg, Facet facet, llvm::Value*, bool clear_facets);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

} // namespace

#endif
