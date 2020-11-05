/**
 * This file is part of Rellume.
 *
 * (c) 2016-2019, Alexis Engelke <alexis.engelke@googlemail.com>
 * (c) 2020, Dominik Okwieka <dominik.okwieka@t-online.de>
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

#include "arch.h"
#include "facet.h"

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Value.h>

#include <bitset>
#include <tuple>
#include <vector>

namespace rellume {

class ArchReg {
public:
    enum class RegKind : uint8_t {
        INVALID = 0,
        GP,     // 64-bit
        IP,     // 64-bit
        EFLAGS, // 7 x 1-bit
        VEC,    // >= 128-bit
    };

private:
    RegKind kind;
    uint8_t index;

public:
    constexpr ArchReg() : kind(RegKind::INVALID), index(0) {}

private:
    constexpr ArchReg(RegKind kind, uint8_t index = 0)
        : kind(kind), index(index) {}

public:
    RegKind Kind() const { return kind; }
    uint8_t Index() const { return index; }

    bool IsGP() const { return kind == RegKind::GP; }

    inline bool operator==(const ArchReg& rhs) const {
        return kind == rhs.kind && index == rhs.index;
    }

    static constexpr ArchReg GP(unsigned idx) {
        return ArchReg(RegKind::GP, idx);
    }
    static constexpr ArchReg VEC(unsigned idx) {
        return ArchReg(RegKind::VEC, idx);
    }

    static const ArchReg INVALID;
    static const ArchReg IP;
    static const ArchReg EFLAGS;
    // x86-64-specific names ignored by other archs
    static const ArchReg RAX, RCX, RDX, RBX, RSP, RBP, RSI, RDI;
};

constexpr const ArchReg ArchReg::INVALID{ArchReg::RegKind::INVALID, 0};
constexpr const ArchReg ArchReg::IP{ArchReg::RegKind::IP, 0};
constexpr const ArchReg ArchReg::EFLAGS{ArchReg::RegKind::EFLAGS, 0};
constexpr const ArchReg ArchReg::RAX = ArchReg::GP(0);
constexpr const ArchReg ArchReg::RCX = ArchReg::GP(1);
constexpr const ArchReg ArchReg::RDX = ArchReg::GP(2);
constexpr const ArchReg ArchReg::RBX = ArchReg::GP(3);
constexpr const ArchReg ArchReg::RSP = ArchReg::GP(4);
constexpr const ArchReg ArchReg::RBP = ArchReg::GP(5);
constexpr const ArchReg ArchReg::RSI = ArchReg::GP(6);
constexpr const ArchReg ArchReg::RDI = ArchReg::GP(7);

// The calling convention code uses RegisterSet to record which registers
// are used by the basic blocks of a function, in order to generate loads
// and stores for calls and returns. Which bit represents which register
// depends on the architecture and is defined by RegisterSetBitIdx.
//
// Unlike vector<bool>, bitset allows helpful bit operations and needs no
// initialisation, but is fixed in size. Many bits are unused (x64 uses
// merely 40 registers, aarch64 uses 68).
using RegisterSet = std::bitset<128>;
unsigned RegisterSetBitIdx(ArchReg reg, Facet facet);

class RegFile {
public:
    RegFile(Arch arch);
    ~RegFile();

    RegFile(RegFile&& rhs);
    RegFile& operator=(RegFile&& rhs);

    RegFile(const RegFile&) = delete;
    RegFile& operator=(const RegFile&) = delete;

    llvm::BasicBlock* GetInsertBlock();
    void SetInsertBlock(llvm::BasicBlock* new_block);

    void Clear();
    using PhiDesc = std::tuple<ArchReg, Facet, llvm::PHINode*>;
    void InitWithPHIs(std::vector<PhiDesc>*, bool all_facets);

    llvm::Value* GetReg(ArchReg reg, Facet facet);
    void SetReg(ArchReg reg, Facet facet, llvm::Value*, bool clear_facets);

    /// Modified registers not yet recorded in a CallConvPack in the FunctionInfo.
    RegisterSet& DirtyRegs();

    /// Registers that have been packed into a CallConvPack.
    RegisterSet& CleanedRegs();

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

} // namespace rellume

#endif
