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

#include "a64/lifter.h"
#include "a64/lifter-private.h"

#include "arch.h"
#include "facet.h"
#include "instr.h"
#include "regfile.h"

#include <llvm/IR/Constants.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>

namespace rellume::aarch64 {

bool LiftInstruction(const Instr& inst, FunctionInfo& fi, const LLConfig& cfg,
                     ArchBasicBlock& ab) noexcept {
    return Lifter(fi, cfg, ab).Lift(inst);
}

bool Lifter::Lift(const Instr& inst) {
    SetIP(inst.start()); // ARM PC points to current instruction.

    // Add instruction marker
    if (cfg.instr_marker) {
        llvm::Value* rip = GetReg(ArchReg::IP, Facet::I64);
        llvm::StringRef str_ref{reinterpret_cast<const char*>(&inst),
                                sizeof(FdInstr)};
        llvm::MDString* md = llvm::MDString::get(irb.getContext(), str_ref);
        llvm::Value* md_val = llvm::MetadataAsValue::get(irb.getContext(), md);
        irb.CreateCall(cfg.instr_marker, {rip, md_val});
    }

    // Check overridden implementations first.
    const auto& override = cfg.instr_overrides.find(inst.type());
    if (override != cfg.instr_overrides.end()) {
        CallExternalFunction(override->second);
        return true;
    }

    const farmdec::Inst* a64p = inst;
    const farmdec::Inst& a64 = *a64p;
    bool w32 = a64.flags & farmdec::W32;
    bool set_flags = a64.flags & farmdec::SET_FLAGS;
    unsigned bits = (w32) ? 32 : 64;

    switch (a64.op) {
    default:
        SetIP(inst.start(), /*nofold=*/true);
        return false;
    case farmdec::A64_ADD_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = irb.getIntN(bits, a64.imm);
        auto val = irb.CreateAdd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcAdd(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_ADD_SHIFTED: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount);
        auto val = irb.CreateAdd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcAdd(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_ADD_EXT: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Extend(GetGp(a64.rm, w32), static_cast<farmdec::ExtendType>(a64.extend.type), a64.extend.lsl);
        auto val = irb.CreateAdd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcAdd(val, lhs, rhs);
        }
        break;
    }
    }

    // For non-branches, continue with the next instruction. The Function::Lift
    // branch-generation code uses the ArchReg::IP in the x86 way, as a pointer
    // to the _next_ instruction. Better handle it here than add special case
    // handling there.
    SetIP(inst.end());
    return true;
}

/// Get the value of an A64 general-purpose register. If w32 is true, the 32-bit facet
/// is used instead of the 64-bit one. Handles ZR (yields zero) and SP (stack ptr).
llvm::Value* Lifter::GetGp(farmdec::Reg r, bool w32) {
    unsigned bits = (w32) ? 32 : 64;
    Facet fc = (w32) ? Facet::I32 : Facet::I64;

    if (r == farmdec::ZERO_REG) {
        return irb.getIntN(bits, 0);
    }
    if (r == farmdec::STACK_POINTER) {
        return GetReg(ArchReg::A64_SP, fc);
    }
    return GetReg(ArchReg::GP(r), fc);
}

/// Set the value of an A64 general-purpose register. If w32 is true, the 32-bit facet
/// is used instead of the 64-bit one. Handles ZR (discards writes) and SP (stack ptr).
void Lifter::SetGp(farmdec::Reg r, bool w32, llvm::Value* val) {
    Facet fc = (w32) ? Facet::I32 : Facet::I64;

    if (r == farmdec::ZERO_REG) {
        return; // discard
    }
    if (r == farmdec::STACK_POINTER) {
        SetReg(ArchReg::A64_SP, Facet::I64, irb.CreateZExt(val, irb.getInt64Ty()));
        SetRegFacet(ArchReg::A64_SP, fc, val);
        return;
    }
    SetReg(ArchReg::GP(r), Facet::I64, irb.CreateZExt(val, irb.getInt64Ty()));
    SetRegFacet(ArchReg::GP(r), fc, val);
}

void Lifter::FlagCalcAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs) {
    auto zero = llvm::Constant::getNullValue(res->getType());
    SetFlag(Facet::ZF, irb.CreateICmpEQ(res, zero));
    SetFlag(Facet::SF, irb.CreateICmpSLT(res, zero));
    SetFlag(Facet::CF, irb.CreateICmpULT(res, lhs));

    if (cfg.enableOverflowIntrinsics) {
        llvm::Intrinsic::ID id = llvm::Intrinsic::sadd_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, lhs, rhs);
        SetFlag(Facet::OF, irb.CreateExtractValue(packed, 1));
    } else {
        llvm::Value* tmp1 = irb.CreateNot(irb.CreateXor(lhs, rhs));
        llvm::Value* tmp2 = irb.CreateAnd(tmp1, irb.CreateXor(res, lhs));
        SetFlag(Facet::OF, irb.CreateICmpSLT(tmp2, zero));
    }
}

// Shift or rotate the value v. No spurious instruction is generated if the shift
// amount is zero.
llvm::Value* Lifter::Shift(llvm::Value* v, farmdec::Shift sh, uint32_t amount) {
    if (amount == 0) {
        return v;
    }
    switch (sh) {
    case farmdec::SH_LSL: return irb.CreateShl(v, (uint64_t) amount);
    case farmdec::SH_LSR: return irb.CreateLShr(v, (uint64_t) amount);
    case farmdec::SH_ASR: return irb.CreateAShr(v, (uint64_t) amount);
    case farmdec::SH_ROR: break; // XXX use fshr intrinsic; only for RORV
    }
    return v; // no change
}

// Zero- or sign-extend the value v, and optionally apply a left shift.
llvm::Value* Lifter::Extend(llvm::Value* v, farmdec::ExtendType ext, uint32_t lsl) {
    llvm::Value* extended = v;
    switch (ext) {
    case farmdec::UXTB: extended = irb.CreateZExt(v,  irb.getInt8Ty()); break;
    case farmdec::UXTH: extended = irb.CreateZExt(v, irb.getInt16Ty()); break;
    case farmdec::UXTW: extended = irb.CreateZExt(v, irb.getInt32Ty()); break;
    case farmdec::UXTX: extended = irb.CreateZExt(v, irb.getInt64Ty()); break;
    case farmdec::SXTB: extended = irb.CreateSExt(v,  irb.getInt8Ty()); break;
    case farmdec::SXTH: extended = irb.CreateSExt(v, irb.getInt16Ty()); break;
    case farmdec::SXTW: extended = irb.CreateSExt(v, irb.getInt32Ty()); break;
    case farmdec::SXTX: extended = irb.CreateSExt(v, irb.getInt64Ty()); break;
    }
    return Shift(extended, farmdec::SH_LSL, lsl);
}

} // namespace rellume::aarch64

/**
 * @}
 **/
