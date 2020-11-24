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

#include "regfile.h"
#include <cstdbool>
#include <cstdint>
#include <vector>


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

class BasicBlock;

struct CallConvPack {
    RegisterSet block_dirty_regs;
    BasicBlock* bb;
    llvm::StoreInst* stores[SptrIdx::MAX];
};

struct FunctionInfo {
    /// The function itself
    llvm::Function* fn;
    /// The sptr argument, and its elements
    llvm::Value* sptr_raw;
    std::vector<llvm::Value*> sptr;

    uint64_t entry_ip;
    llvm::Value* entry_ip_value;

    std::vector<CallConvPack> call_conv_packs;
};


} // namespace

#endif
