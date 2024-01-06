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

#include "instr.h"
#include <llvm/ADT/DenseMap.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <cstdint>
#include <functional>
#include <vector>


namespace rellume {

class Instr;
struct LLConfig;
class LiftHelper;

class Function
{
public:
    Function(llvm::Module* mod, LLConfig* cfg) : mod(mod), cfg(cfg) {}

    Function(Function&& rhs);
    Function& operator=(Function&& rhs);

    Function(const Function&) = delete;
    Function& operator=(const Function&) = delete;

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
    llvm::Module* mod;
    LLConfig* cfg;

    struct DecodedInstr {
        Instr inst;
        bool new_block;
    };

    std::vector<DecodedInstr> instrs;

    struct InstrMapEntry {
        unsigned preds = 0;
        /// Indicate whether the instruction was successfully decoded
        bool decoded = false;
        /// Index in instrs; only valid if decoded is true
        size_t instr_idx = 0;
    };
    llvm::DenseMap<uint64_t, InstrMapEntry> instr_map; // map addr -> instr

    std::vector<CodeRange> code_ranges = {{0, 0}};

    friend class LiftHelper;
};

}

#endif
