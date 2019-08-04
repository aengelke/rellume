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

#include "facet.h"
#include "rellume/instr.h"

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Value.h>
#include <functional>


namespace rellume {

class RegFile
{
public:
    RegFile(llvm::BasicBlock* llvm_block);
    ~RegFile();

    RegFile(RegFile&& rhs);
    RegFile& operator=(RegFile&& rhs);

    RegFile(const RegFile&) = delete;
    RegFile& operator=(const RegFile&) = delete;

    using Generator = std::function<llvm::Value*()>;
    using InitGenerator = std::function<Generator(const LLReg, const Facet)>;
    void InitAll(InitGenerator init_gen = nullptr);

    llvm::Value* GetReg(LLReg reg, Facet facet);
    void SetReg(LLReg reg, Facet facet, llvm::Value*, bool clear_facets);

    void UpdateAllFromMem(llvm::Value* buf_ptr);
    void UpdateAllInMem(llvm::Value* buf_ptr);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

} // namespace

#endif
