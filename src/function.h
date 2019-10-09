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

#include "callconv.h"
#include "config.h"
#include "rellume/instr.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/Value.h>
#include <cstdint>
#include <functional>
#include <unordered_map>


namespace rellume {

class ArchBasicBlock;

class Function
{
public:
    Function(llvm::Module* mod, CallConv callconv);
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
    void EnableVerifyIR(bool enable) { cfg.verify_ir = enable; }

    void SetGlobalBase(uintptr_t base, llvm::Value* value) {
        cfg.global_base_addr = base;
        cfg.global_base_value = value;
    }

    void SetInstrImpl(LLInstrType type, llvm::Function* override) {
        cfg.instr_overrides[type] = override;
    }

    void AddInst(uint64_t block_addr, const LLInstr& inst);
    llvm::Function* Lift();

    // Implemented in lldecoder.cc
    using MemReader = std::function<size_t(uintptr_t, uint8_t*, size_t)>;
    int Decode(uintptr_t addr, MemReader memacc = nullptr);

private:
    ArchBasicBlock& ResolveAddr(llvm::Value* addr);

    LLConfig cfg;

    llvm::Function* llvm;
    uint64_t entry_addr;
    std::unique_ptr<ArchBasicBlock> entry_block;
    std::unique_ptr<ArchBasicBlock> exit_block;
    std::unordered_map<uint64_t,std::unique_ptr<ArchBasicBlock>> block_map;
};

}

#endif
