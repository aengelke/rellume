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

#ifndef LL_FUNCTION_H
#define LL_FUNCTION_H

#include "llcommon-internal.h"
#include "rellume/instr.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Value.h>
#include <cstdint>
#include <unordered_map>


namespace rellume
{

class BasicBlock;

class Function
{
public:
    Function(llvm::Module* mod);
    ~Function();

    Function(Function&& rhs);
    Function& operator=(Function&& rhs);

    Function(const Function&) = delete;
    Function& operator=(const Function&) = delete;

    /// Enable the usage of overflow intrinsics instead of bitwise operations
    /// when setting the overflow flag. For dynamic values this leads to better
    /// code which relies on the overflow flag again. However, immediate values
    /// are not folded when they are guaranteed to overflow.
    void EnableOverflowIntrinsics(bool enable) {
        cfg.enableOverflowIntrinsics = enable;
    }

    /// Enable unsafe floating-point optimizations, similar to -ffast-math.
    void EnableFastMath(bool enable) {
        cfg.enableFastMath = enable;
    }

    void SetGlobalBase(uintptr_t base, llvm::Value* value) {
        cfg.global_base_addr = base;
        cfg.global_base_value = value;
    }

    void AddInst(uint64_t block_addr, const LLInstr& inst);
    llvm::Function* Lift();

    // Implemented in lldecoder.cc
    int Decode(uintptr_t addr);

private:
    void CreateEntry(BasicBlock&);
    std::unique_ptr<BasicBlock> CreateExit();
    BasicBlock& ResolveAddr(llvm::Value* addr, BasicBlock& def);

    LLConfig cfg;

    llvm::Function* llvm;
    uint64_t entry_addr;
    std::unordered_map<uint64_t,std::unique_ptr<BasicBlock>> block_map;
};

}

#endif
