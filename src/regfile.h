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

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Value.h>
#include <tuple>
#include <vector>


namespace rellume {

class X86Reg {
public:
    enum class RegKind : uint8_t {
        INVALID = 0, GP /*64-bit*/, IP /*64-bit*/, EFLAGS, VEC,
    };
private:
    RegKind kind;
    uint8_t index;
public:
    constexpr X86Reg()
            : kind(RegKind::INVALID), index(0) {}
private:
    constexpr X86Reg(RegKind kind, uint8_t index = 0)
            : kind(kind), index(index) {}
public:
    RegKind Kind() const { return kind; }
    uint8_t Index() const { return index; }

    bool IsGP() const { return kind == RegKind::GP; }

    inline bool operator==(const X86Reg& rhs) const {
        return kind == rhs.kind && index == rhs.index;
    }

    static constexpr X86Reg GP(unsigned idx) { return X86Reg(RegKind::GP, idx); }
    static constexpr X86Reg VEC(unsigned idx) { return X86Reg(RegKind::VEC, idx); }
    static const X86Reg RAX, RCX, RDX, RBX, RSP, RBP, RSI, RDI;
    static const X86Reg IP;
    static const X86Reg EFLAGS;
};

constexpr const X86Reg X86Reg::RAX = X86Reg::GP(0);
constexpr const X86Reg X86Reg::RCX = X86Reg::GP(1);
constexpr const X86Reg X86Reg::RDX = X86Reg::GP(2);
constexpr const X86Reg X86Reg::RBX = X86Reg::GP(3);
constexpr const X86Reg X86Reg::RSP = X86Reg::GP(4);
constexpr const X86Reg X86Reg::RBP = X86Reg::GP(5);
constexpr const X86Reg X86Reg::RSI = X86Reg::GP(6);
constexpr const X86Reg X86Reg::RDI = X86Reg::GP(7);
constexpr const X86Reg X86Reg::IP{X86Reg::RegKind::IP, 0};
constexpr const X86Reg X86Reg::EFLAGS{X86Reg::RegKind::EFLAGS, 0};

class RegisterSet {
private:
    uint16_t gp;
    uint8_t flags;
    uint16_t vec;

public:
    RegisterSet() : gp(0), flags(0), vec(0) {}

private:
    bool TestSet(X86Reg reg, Facet facet, unsigned set);

public:
    bool Get(X86Reg reg, Facet facet) { return TestSet(reg, facet, 0); }
    void Set(X86Reg reg, Facet facet) { TestSet(reg, facet, 1); }

    void Add(const RegisterSet& other) {
        gp |= other.gp;
        flags |= other.flags;
        vec |= other.vec;
    }
};

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
    using PhiDesc = std::tuple<X86Reg, Facet, llvm::PHINode*>;
    void InitWithPHIs(std::vector<PhiDesc>*);

    llvm::Value* GetReg(X86Reg reg, Facet facet);
    void SetReg(X86Reg reg, Facet facet, llvm::Value*, bool clear_facets);

    const RegisterSet& ModifiedRegs() const;

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

} // namespace

#endif
