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

#include "x86-64/lifter-private.h"

#include "instr.h"
#include "regfile.h"
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>


/**
 * \defgroup LLFlags Flags
 * \brief Computation of X86 flags
 *
 * @{
 **/

namespace rellume::x86_64 {

llvm::Value* Lifter::FlagCond(Condition cond) {
    llvm::Value* result = nullptr;
    switch (static_cast<Condition>(static_cast<int>(cond) & ~1)) {
    case Condition::O:  result = GetFlag(Facet::OF); break;
    case Condition::C:  result = GetFlag(Facet::CF); break;
    case Condition::Z:  result = GetFlag(Facet::ZF); break;
    case Condition::BE: result = irb.CreateOr(GetFlag(Facet::CF), GetFlag(Facet::ZF)); break;
    case Condition::S:  result = GetFlag(Facet::SF); break;
    case Condition::P:  result = GetFlag(Facet::PF); break;
    case Condition::L:  result = irb.CreateICmpNE(GetFlag(Facet::SF), GetFlag(Facet::OF)); break;
    case Condition::LE: result = irb.CreateOr(GetFlag(Facet::ZF), irb.CreateICmpNE(GetFlag(Facet::SF), GetFlag(Facet::OF))); break;
    default: assert(0);
    }

    return static_cast<int>(cond) & 1 ? irb.CreateNot(result) : result;
}

static const std::pair<Facet, unsigned> RFLAGS_INDICES[] = {
    {Facet::CF, 0}, {Facet::PF, 2}, {Facet::AF, 4}, {Facet::ZF, 6},
    {Facet::SF, 7}, {Facet::DF, 10}, {Facet::OF, 11},
};

llvm::Value* Lifter::FlagAsReg(unsigned size) {
    llvm::Value* res = irb.getInt64(0x202); // IF
    llvm::Type* ty = res->getType();
    for (auto& kv : RFLAGS_INDICES) {
        llvm::Value* ext_bit = irb.CreateZExt(GetFlag(kv.first), ty);
        res = irb.CreateOr(res, irb.CreateShl(ext_bit, kv.second));
    }
    return irb.CreateTruncOrBitCast(res, irb.getIntNTy(size));
}

void Lifter::FlagFromReg(llvm::Value* val) {
    unsigned sz = val->getType()->getIntegerBitWidth();
    for (auto& kv : RFLAGS_INDICES) {
        if (kv.second >= sz)
            break;
        llvm::Value* bit = irb.CreateLShr(val, kv.second);
        SetFlag(kv.first, irb.CreateTrunc(bit, irb.getInt1Ty()));
    }
}

} // namespace::x86_64

/**
 * @}
 **/
