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

#include "function-info.h"
#include "instr.h"
#include <llvm/ADT/DenseMap.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Value.h>
#include <cstdint>
#include <functional>
#include <unordered_map>


namespace rellume {

class ArchBasicBlock;
class Instr;
struct LLConfig;

class Function
{
public:
    Function(llvm::Module* mod, LLConfig* cfg);
    ~Function();

    Function(Function&& rhs);
    Function& operator=(Function&& rhs);

    Function(const Function&) = delete;
    Function& operator=(const Function&) = delete;

    int AddInst(uint64_t block_addr, uint64_t addr, size_t bufsz,
                const uint8_t* buf);
    bool AddInst(uint64_t block_addr, const Instr& inst);
    llvm::Function* Lift();

    // Implemented in lldecoder.cc
    enum class DecodeStop {
        INSTR,
        BASICBLOCK,
        ALL,
    };
    using MemReader = std::function<size_t(uintptr_t, uint8_t*, size_t)>;
    int Decode(uintptr_t addr, DecodeStop stop, MemReader memacc = nullptr);

    struct CodeRange {
        uint64_t start, end;
    };
    const CodeRange* CodeRanges() const {
        return code_ranges.data();
    }

private:
    ArchBasicBlock& ResolveAddr(llvm::Value* addr);

    LLConfig* cfg;
    FunctionInfo fi;

    llvm::Function* llvm;
    uint64_t entry_addr;

    struct DecodedInstr {
        Instr inst;
        bool new_block;
    };

    std::vector<DecodedInstr> instrs;
    llvm::DenseMap<uint64_t, size_t> instr_map; // map addr -> instr idx

    std::unique_ptr<ArchBasicBlock> entry_block;
    std::unique_ptr<ArchBasicBlock> exit_block;
    std::unordered_map<uint64_t,std::unique_ptr<ArchBasicBlock>> block_map;
};

}

#endif
