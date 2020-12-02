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
#include <llvm/IR/DerivedTypes.h>
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

    // In the order of farmdec::Op.
    switch (a64.op) {
    default:
        SetIP(inst.start(), /*nofold=*/true);
        return false;
    case farmdec::A64_ADD_IMM:
    case farmdec::A64_CMN_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = irb.getIntN(bits, a64.imm);
        auto val = irb.CreateAdd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcAdd(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_SUB_IMM:
    case farmdec::A64_CMP_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = irb.getIntN(bits, a64.imm);
        auto val = irb.CreateSub(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcSub(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_MOV_IMM:
        SetGp(a64.rd, w32, irb.getIntN(bits, a64.imm));
        break;
    case farmdec::A64_B:
        SetIP(inst.start() + a64.offset);
        return true;
    case farmdec::A64_BR:
        SetReg(ArchReg::IP, Facet::I64, GetGp(a64.rn, false));
        return true;
    case farmdec::A64_BL:
    case farmdec::A64_BLR: {
        if (cfg.call_ret_clobber_flags)
            SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::CF});

        auto ret_addr = PCRel(inst.len()); // _next_ instruction → inst.end()
        SetGp(30, false, ret_addr);        // X30: Link Register (LR)

        if (a64.op == farmdec::A64_BLR)
            SetReg(ArchReg::IP, Facet::I64, GetGp(a64.rn, false));
        else
            SetIP(inst.start() + a64.offset);

        if (cfg.call_function) {
            CallExternalFunction(cfg.call_function);

            // The external function call may manipulate the PC in non-obvious ways (e.g. exceptions).
            // See also the comment in the x86_64 LiftCall method.
            llvm::Value* cont_addr = GetReg(ArchReg::IP, Facet::I64);
            llvm::Value* eq = irb.CreateICmpEQ(cont_addr, ret_addr);
            SetReg(ArchReg::IP, Facet::I64, irb.CreateSelect(eq, ret_addr, cont_addr)); // common case
        }
        return true;
    }
    case farmdec::A64_RET:
        if (cfg.call_ret_clobber_flags)
            SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::CF});

        SetReg(ArchReg::IP, Facet::I64, GetGp(a64.rn, false));

        if (cfg.call_function) {
            // If we are in call-ret-lifting mode, forcefully return. Otherwise, we
            // might end up using tail_function, which we don't want here.
            ForceReturn();
        }
        return true;
    case farmdec::A64_MOV_REG:
        SetGp(a64.rd, w32, GetGp(a64.rm, w32)); // rd := rm
        break;
    case farmdec::A64_ADD_SHIFTED:
    case farmdec::A64_CMN_SHIFTED: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount);
        auto val = irb.CreateAdd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcAdd(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_SUB_SHIFTED:
    case farmdec::A64_NEG:
    case farmdec::A64_CMP_SHIFTED: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount);
        auto val = irb.CreateSub(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcSub(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_ADD_EXT:
    case farmdec::A64_CMN_EXT: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Extend(GetGp(a64.rm, w32), static_cast<farmdec::ExtendType>(a64.extend.type), a64.extend.lsl);
        auto val = irb.CreateAdd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcAdd(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_SUB_EXT:
    case farmdec::A64_CMP_EXT: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Extend(GetGp(a64.rm, w32), static_cast<farmdec::ExtendType>(a64.extend.type), a64.extend.lsl);
        auto val = irb.CreateSub(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcSub(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_LDR: {
        farmdec::AddrMode mode = fad_get_addrmode(a64.flags);
        farmdec::ExtendType ext = fad_get_mem_extend(a64.flags);
        farmdec::Size sz = static_cast<farmdec::Size>(ext & 3); // lower two bits
        bool sign_extend = (ext & 4) != 0;

        auto srcty = TypeOf(sz); // type of data to load
        auto dstty = TypeOf((w32) ? farmdec::SZ_W : farmdec::SZ_X); // type of the register the result is stored to
        auto addr = Addr(srcty, a64);

        llvm::LoadInst* load = irb.CreateLoad(srcty, addr);
        if (mode == farmdec::AM_SIMPLE) {
            // XXX AM_SIMPLE → set AtomicOrdering according to Inst.ldst_order
        }
        llvm::Value* val = load;
        if (srcty != dstty) { // sign- or zero-extend unless a copy is sufficient (i.e. srcty==dstty)
            val = (sign_extend) ? irb.CreateSExt(val, dstty) : irb.CreateZExt(val, dstty);
        }

        SetGp(a64.rt, w32, val);
        break;
    }
    case farmdec::A64_STR: {
        farmdec::AddrMode mode = fad_get_addrmode(a64.flags);
        farmdec::ExtendType ext = fad_get_mem_extend(a64.flags);
        farmdec::Size sz = static_cast<farmdec::Size>(ext & 3); // lower two bits

        auto val = GetGp(a64.rt, w32);
        auto srcty = TypeOf((w32) ? farmdec::SZ_W : farmdec::SZ_X);
        auto dstty = TypeOf(sz);
        auto addr = Addr(dstty, a64);

        if (srcty != dstty) {
            val = irb.CreateTrunc(val, dstty);
        }

        llvm::StoreInst* store = irb.CreateStore(val, addr);
        if (mode == farmdec::AM_SIMPLE) {
            // XXX AM_SIMPLE → set AtomicOrdering according to Inst.ldst_order
        }
        break;
    }
    case farmdec::A64_LDR_FP:
        break;
    case farmdec::A64_STR_FP:
        break;
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
llvm::Value* Lifter::GetGp(farmdec::Reg r, bool w32, bool ptr) {
    unsigned bits = (w32) ? 32 : 64;
    Facet fc = (w32) ? Facet::I32 : Facet::I64;
    if (ptr) {
        bits = 64; // all ptrs are 64 bits
        fc = Facet::PTR;
    }

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

void Lifter::FlagCalcSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs) {
    auto zero = llvm::Constant::getNullValue(res->getType());
    llvm::Value* sf = irb.CreateICmpSLT(res, zero);  // also used for OF
    SetFlag(Facet::ZF, irb.CreateICmpEQ(lhs, rhs));
    SetFlag(Facet::SF, sf);
    SetFlag(Facet::CF, irb.CreateICmpUGE(lhs, rhs));
    SetFlag(Facet::OF, irb.CreateICmpNE(sf, irb.CreateICmpSLT(lhs, rhs)));
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

llvm::IntegerType* Lifter::TypeOf(farmdec::Size sz) {
    switch (sz) {
    case farmdec::SZ_B: return irb.getInt8Ty();
    case farmdec::SZ_H: return irb.getInt16Ty();
    case farmdec::SZ_W: return irb.getInt32Ty();
    case farmdec::SZ_X: return irb.getInt64Ty();
    }
    assert(false && "invalid size");
}

llvm::Type* Lifter::TypeOf(farmdec::FPSize fsz) {
    switch (fsz) {
    case farmdec::FSZ_B: return irb.getInt8Ty();
    case farmdec::FSZ_H: return irb.getHalfTy();
    case farmdec::FSZ_S: return irb.getFloatTy();
    case farmdec::FSZ_D: return irb.getDoubleTy();
    case farmdec::FSZ_Q: return llvm::ArrayType::get(irb.getDoubleTy(), 2);
    }
    assert(false && "invalid FP size");
}

// Returns PC-relative address as i64, suitable for storing into PC again.
llvm::Value* Lifter::PCRel(uint64_t off) {
    return irb.CreateAdd(GetReg(ArchReg::IP, Facet::I64), irb.getInt64(off));
}

// Dispatches to the correct addressing mode handler.
llvm::Value* Lifter::Addr(llvm::Type* elemty, farmdec::Inst inst) {
    farmdec::AddrMode mode = fad_get_addrmode(inst.flags);

    // XXX Move the variants in here if that significantly increases performance.
    // XXX But: might lead to clutter, and I'd like to keep the extensive comments.

    switch (mode) {
    case farmdec::AM_SIMPLE:
        return Addr(elemty, inst.rn);
    case farmdec::AM_OFF_IMM:
        return Addr(elemty, inst.rn, inst.offset);
    case farmdec::AM_OFF_REG:
        return Addr(elemty, inst.rn, inst.rm, inst.shift.amount);
    case farmdec::AM_OFF_EXT:
        return Addr(elemty, inst.rn, inst.rm, static_cast<farmdec::ExtendType>(inst.extend.type), inst.extend.lsl);
    case farmdec::AM_PRE:
        SetGp(inst.rn, /*w32=*/false, irb.CreateAdd(GetGp(inst.rn, /*w32=*/false), irb.getIntN(64, inst.imm))); // rn += imm
        return Addr(elemty, inst.rn);
    case farmdec::AM_POST: {
        auto addr = Addr(elemty, inst.rn);
        SetGp(inst.rn, /*w32=*/false, irb.CreateAdd(GetGp(inst.rn, /*w32=*/false), irb.getIntN(64, inst.imm))); // rn += imm
        return addr;
    }
    case farmdec::AM_LITERAL:
        return irb.CreateIntToPtr(PCRel(inst.offset), elemty->getPointerTo());
    }

    assert(false && "invalid addrmode");
}

// AM_SIMPLE addressing mode: [base].
// AM_PRE and AM_POST also make use of this, but add an immediate to base
// before or after this call.
llvm::Value* Lifter::Addr(llvm::Type* elemty, farmdec::Reg base) {
    return irb.CreatePointerCast(GetGp(base, false, /*ptr=*/true), elemty->getPointerTo());
}

// AM_OFF_IMM addressing mode: [base, #imm].
llvm::Value* Lifter::Addr(llvm::Type* elemty, farmdec::Reg base, uint64_t off) {
    auto ptr = GetGp(base, false, /*ptr=*/true);
    auto byteaddr = irb.CreateConstGEP1_64(irb.getInt8Ty(), ptr, off);
    return irb.CreatePointerCast(byteaddr, elemty->getPointerTo());
}

// AM_OFF_REG addressing mode: [base, Xoff {, LSL #imm}]. #imm is log2(size in bytes) or #0.
llvm::Value* Lifter::Addr(llvm::Type* elemty, farmdec::Reg base, farmdec::Reg off, uint32_t lsl) {
    // Uncommon: Offset register holds a byte offset instead of an index.
    // So do a bit of "manual" pointer arithmetic:
    //
    //     uintptr_t baseval = (uintptr_t) base;
    //     uintptr_t byteaddr = base + off;
    //     return (elemty*)byteaddr;
    //
    if (lsl == 0 && elemty != irb.getInt8Ty()) {
        auto baseval = GetGp(base, false);
        auto byteaddr = irb.CreateAdd(baseval, GetGp(off, false));
        return irb.CreateIntToPtr(byteaddr, elemty->getPointerTo());
    }

    // Usual case of lsl == log2(size in bytes), e.g. log2(4)==2 for i32.
    //
    // Here, the offset register actually holds an _index_ into an array at base.
    // Like in C pointer arithmetic, the GEP instruction automatically converts
    // the index into a byte offset appropriate for the type.
    //
    //     elemty *elemptr = (elemty*)base
    //     return elemptr + idx;
    //
    auto elemptr = irb.CreatePointerCast(GetGp(base, false, /*ptr=*/true),  elemty->getPointerTo());
    return irb.CreateGEP(elemty, elemptr, GetGp(off, false));
}

// AM_OFF_EXT addressing mode: [base, Woff, (U|S)XTW {#imm}].
// As for AM_OFF_REG, #imm is log2(size in bytes) or #0.
llvm::Value* Lifter::Addr(llvm::Type* elemty, farmdec::Reg base, farmdec::Reg off, farmdec::ExtendType ext, uint32_t lsl) {
    auto extended_off = GetGp(off, true);
    switch (ext) {
    case farmdec::UXTW: extended_off = irb.CreateZExt(extended_off, irb.getInt64Ty()); break;
    case farmdec::SXTW: extended_off = irb.CreateSExt(extended_off, irb.getInt64Ty()); break;
    default:
        assert(false && "bad extend type for AM_OFF_EXT: only UXTW and SXTW are allowed");
    }

    // An exact copy of the AM_OFF_REG case, sans the explanations.
    if (lsl == 0 && elemty != irb.getInt8Ty()) {
        auto baseval = GetGp(base, false);
        auto byteaddr = irb.CreateAdd(baseval, extended_off);
        return irb.CreateIntToPtr(byteaddr, elemty->getPointerTo());
    }
    auto elemptr = irb.CreatePointerCast(GetGp(base, false, /*ptr=*/true),  elemty->getPointerTo());
    return irb.CreateGEP(elemty, elemptr, extended_off);
}

} // namespace rellume::aarch64

/**
 * @}
 **/
