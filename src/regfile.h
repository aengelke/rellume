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
        FLAG,   // status flag
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
    static constexpr ArchReg FLAG(unsigned idx) {
        return ArchReg(RegKind::FLAG, idx);
    }

    static const ArchReg INVALID;
    static const ArchReg ZF, SF, PF, CF, OF, AF, DF;
    // x86-64-specific names ignored by other archs
    static const ArchReg RAX, RCX, RDX, RBX, RSP, RBP, RSI, RDI;

    // AArch64-specific names
    static const ArchReg A64_SP;
};

constexpr const ArchReg ArchReg::INVALID{ArchReg::RegKind::INVALID, 0};
constexpr const ArchReg ArchReg::RAX = ArchReg::GP(0);
constexpr const ArchReg ArchReg::RCX = ArchReg::GP(1);
constexpr const ArchReg ArchReg::RDX = ArchReg::GP(2);
constexpr const ArchReg ArchReg::RBX = ArchReg::GP(3);
constexpr const ArchReg ArchReg::RSP = ArchReg::GP(4);
constexpr const ArchReg ArchReg::RBP = ArchReg::GP(5);
constexpr const ArchReg ArchReg::RSI = ArchReg::GP(6);
constexpr const ArchReg ArchReg::RDI = ArchReg::GP(7);
constexpr const ArchReg ArchReg::ZF = ArchReg::FLAG(0);
constexpr const ArchReg ArchReg::SF = ArchReg::FLAG(1);
constexpr const ArchReg ArchReg::PF = ArchReg::FLAG(2);
constexpr const ArchReg ArchReg::CF = ArchReg::FLAG(3);
constexpr const ArchReg ArchReg::OF = ArchReg::FLAG(4);
constexpr const ArchReg ArchReg::AF = ArchReg::FLAG(5);
constexpr const ArchReg ArchReg::DF = ArchReg::FLAG(6);
constexpr const ArchReg ArchReg::A64_SP = ArchReg::GP(31);

// The calling convention code uses RegisterSet to record which registers
// are used by the basic blocks of a function, in order to generate loads
// and stores for calls and returns. Which bit represents which register
// depends on the architecture and is defined by RegisterSetBitIdx.
//
// Unlike vector<bool>, bitset allows helpful bit operations and needs no
// initialisation, but is fixed in size. Many bits are unused (x64 uses
// mere 40 registers, aarch64 uses 68).
using RegisterSet = std::bitset<128>;
unsigned RegisterSetBitIdx(ArchReg reg);

class RegFile {
public:
    RegFile(Arch arch, llvm::BasicBlock* bb);
    ~RegFile();

    RegFile(RegFile&& rhs);
    RegFile& operator=(RegFile&& rhs);

    RegFile(const RegFile&) = delete;
    RegFile& operator=(const RegFile&) = delete;

    llvm::BasicBlock* GetInsertBlock();
    void SetInsertPoint(llvm::BasicBlock::iterator ip);

    void InitWithRegFile(RegFile* parent);
    using PhiDesc = std::tuple<ArchReg, Facet, llvm::PHINode*>;
    void InitWithPHIs(std::vector<PhiDesc>*);

    llvm::Value* GetReg(ArchReg reg, Facet facet);

    enum class Transform : uint8_t {
        None,
        IsZero,
        IsNeg,
        IsULT,
        AddOverflowFlag,
        TruncI8,
        /// Create (aligned) load, operand: pointer, some value with load type
        Load,
        /// Auxiliary carry flag, operands: res, lhs, rhs
        X86AuxFlag,
    };

    /// Set full register, insert into zero or sign extend (integer types only)
    void Set(ArchReg reg, llvm::Value* v, bool sext = false);
    /// Merge existing value and only overwrite the lower part
    void Merge(ArchReg reg, llvm::Value*);
    /// Set full register with given, lazily evaluated, transformation
    void Set(ArchReg reg, Transform t, llvm::Value* v1, llvm::Value* v2 = nullptr, llvm::Value* v3 = nullptr);

    void SetPC(uint64_t addr);
    void SetPC(llvm::Value* addr);
    void SetPCCond(llvm::Value* cond, uint64_t addr1, uint64_t addr2);
    void SetPCCallret(llvm::Value* addr, uint64_t check);
    llvm::Value* GetPCValue(llvm::Value* pcBase, uint64_t pcBaseAddr, uint64_t offset = 0);
    /// Tuple of branch cond (or null), then addr (or 0), else addr (or 0)
    std::tuple<llvm::Value*, uint64_t, uint64_t> GetPCBranch(llvm::Value* pcBase, uint64_t pcBaseAddr);

    /// Modified registers not yet recorded in a CallConvPack in the FunctionInfo.
    RegisterSet& DirtyRegs();
    bool StartsClean();

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

} // namespace rellume

#endif
