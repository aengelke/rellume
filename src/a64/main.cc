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

#include <cstdint>

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

static uint64_t ones(int n);

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

    case farmdec::A64_ADR:
        SetGp(a64.rd, /*w32=*/false, PCRel(a64.offset));
        break;
    case farmdec::A64_ADRP: {
        // Scaling to page granularity is already handled in farmdec, but we need to mask out
        // the bottom 12 bits.
        auto masked_pc = irb.CreateAnd(GetReg(ArchReg::IP, Facet::I64), irb.getInt64(~(uint64_t)4095));
        SetGp(a64.rd, /*w32=*/false, irb.CreateAdd(masked_pc, irb.getInt64(a64.offset)));
        break;
    }

    case farmdec::A64_AND_IMM:
    case farmdec::A64_TST_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto val = irb.CreateAnd(lhs, a64.imm);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcLogic(val);
        }
        break;
    }
    case farmdec::A64_ORR_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto val = irb.CreateOr(lhs, a64.imm);
        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_EOR_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto val = irb.CreateXor(lhs, a64.imm);
        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_ADD_IMM:
    case farmdec::A64_MOV_SP:
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
    case farmdec::A64_MOVK: {
        uint64_t clrmask = ~ (0xffffuL << a64.movk.lsl);
        uint64_t shiftedimm = static_cast<uint64_t>(a64.movk.imm16) << a64.movk.lsl;
        auto val = GetGp(a64.rd, w32);
        val = irb.CreateAnd(val, irb.getIntN(bits, clrmask));   // clear 16 bits at point of insertion...
        val = irb.CreateOr(val, irb.getIntN(bits, shiftedimm)); // ...and fill in the immediate
        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_MOV_IMM:
        SetGp(a64.rd, w32, irb.getIntN(bits, a64.imm));
        break;
    case farmdec::A64_SBFM:
        assert(false && "SBFM should only appear as one of its aliases ASR_IMM, SBFIZ, SBFX");
        break;
    case farmdec::A64_ASR_IMM:
        SetGp(a64.rd, w32, Shift(GetGp(a64.rn, w32), farmdec::SH_ASR, a64.imm));
        break;
    case farmdec::A64_SBFIZ: {
        auto field = MoveField(GetGp(a64.rn, w32), w32, a64.bfm.lsb, a64.bfm.width);

        // sext_mask: ones in the upper bits, including the MSB of the field.
        //
        //     msb_set = (field & sext_mask) != 0;
        //     extended = sext_mask | field;
        //     rd = (msb_set) ? extended : field;
        //
        uint64_t sext_mask = ~0uL << (a64.bfm.lsb + a64.bfm.width - 1);
        auto sext_mask_value = irb.getIntN(bits, sext_mask); // truncates to 32-bit if required
        auto msb_set = irb.CreateICmpNE(irb.CreateAnd(field, sext_mask_value), irb.getIntN(bits, 0));
        auto extended = irb.CreateOr(irb.CreateOr(field, sext_mask_value));
        auto val = irb.CreateSelect(msb_set, extended, field);

        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_SBFX: {
        auto field = Extract(GetGp(a64.rn, w32), w32, a64.bfm.lsb, a64.bfm.width);

        // See above.
        uint64_t sext_mask = ~0uL << (a64.bfm.width - 1);
        auto sext_mask_value = irb.getIntN(bits, sext_mask); // truncates to 32-bit if required
        auto msb_set = irb.CreateICmpNE(irb.CreateAnd(field, sext_mask_value), irb.getIntN(bits, 0));
        auto extended = irb.CreateOr(irb.CreateOr(field, sext_mask_value));
        auto val = irb.CreateSelect(msb_set, extended, field);

        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_BFM:
        assert(false && "BFM should only appear as one of its aliases BFC, BFI, BFXIL");
        break;
    case farmdec::A64_BFC: {
        uint64_t clrmask = ~(ones(a64.bfm.width) << a64.bfm.lsb);
        SetGp(a64.rd, w32, irb.CreateAnd(GetGp(a64.rd, w32), irb.getIntN(bits, clrmask)));
        break;
    }
    case farmdec::A64_BFI: {
        auto src = MoveField(GetGp(a64.rn, w32), w32, a64.bfm.lsb, a64.bfm.width);

        uint64_t clrmask = ~(ones(a64.bfm.width) << a64.bfm.lsb);
        auto dst = irb.CreateAnd(GetGp(a64.rd, w32), irb.getIntN(bits, clrmask));

        SetGp(a64.rd, w32, irb.CreateOr(src, dst));
        break;
    }
    case farmdec::A64_BFXIL: {
        auto src = Extract(GetGp(a64.rn, w32), w32, a64.bfm.lsb, a64.bfm.width);

        uint64_t clrmask = ~ones(a64.bfm.width);
        auto dst = irb.CreateAnd(GetGp(a64.rd, w32), irb.getIntN(bits, clrmask));

        SetGp(a64.rd, w32, irb.CreateOr(src, dst));
        break;
    }
    case farmdec::A64_UBFM:
        assert(false && "UBFM should only appear as one of its aliases LSL_IMM, LSR_IMM, UBFIZ, UBFX");
        break;
    case farmdec::A64_LSL_IMM:
        SetGp(a64.rd, w32, Shift(GetGp(a64.rn, w32), farmdec::SH_LSL, a64.imm));
        break;
    case farmdec::A64_LSR_IMM:
        SetGp(a64.rd, w32, Shift(GetGp(a64.rn, w32), farmdec::SH_LSR, a64.imm));
        break;
    case farmdec::A64_UBFIZ:
        SetGp(a64.rd, w32, MoveField(GetGp(a64.rn, w32), w32, a64.bfm.lsb, a64.bfm.width));
        break;
    case farmdec::A64_UBFX:
        SetGp(a64.rd, w32, Extract(GetGp(a64.rn, w32), w32, a64.bfm.lsb, a64.bfm.width));
        break;
    case farmdec::A64_EXTEND:
        SetGp(a64.rd, w32, Extend(GetGp(a64.rn, w32), w32, static_cast<farmdec::ExtendType>(a64.extend.type), 0));
        break;
    case farmdec::A64_ROR_IMM:
        SetGp(a64.rd, w32, Shift(GetGp(a64.rn, w32), farmdec::SH_ROR, a64.imm));
        break;
    case farmdec::A64_BCOND:
        SetReg(ArchReg::IP, Facet::I64, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), PCRel(a64.offset), PCRel(inst.len())));
        return true;
    case farmdec::A64_SVC:
        // SVC has an immediate, but cfg.syscall_implementation takes only a CPU state pointer.
        // This seems dire, but Linux and the BSDs and probably most Unixes only ever use svc #0.
        if (cfg.syscall_implementation)
            CallExternalFunction(cfg.syscall_implementation);
        break;
/*
    Intentionally unimplemented because they are either for debugging or use in the kernel
    while we focus on userspace lifting.

    case farmdec::A64_HVC:
    case farmdec::A64_SMC:
    case farmdec::A64_BRK:
    case farmdec::A64_HLT:
    case farmdec::A64_DCPS1:
    case farmdec::A64_DCPS2:
    case farmdec::A64_DCPS3:
*/
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
    case farmdec::A64_CBZ:
    case farmdec::A64_CBNZ: {
        // CBZ: rt == 0; CBNZ: rt != 0
        auto pred = (a64.op == farmdec::A64_CBZ) ? llvm::CmpInst::Predicate::ICMP_EQ : llvm::CmpInst::Predicate::ICMP_NE;
        auto do_branch = irb.CreateICmp(pred, GetGp(a64.rt, w32), irb.getIntN(bits, 0));
        auto on_true = PCRel(a64.offset);
        auto on_false = PCRel(inst.len()); // next instr
        SetReg(ArchReg::IP, Facet::I64, irb.CreateSelect(do_branch, on_true, on_false));
        return true;
    }
    case farmdec::A64_TBZ:
    case farmdec::A64_TBNZ: {
        assert(a64.tbz.bit < bits);
        uint64_t mask = ((uint64_t)1) << a64.tbz.bit;
        auto bit = irb.CreateAnd(GetGp(a64.rt, w32), irb.getIntN(bits, mask)); // bit := rt & (1 << bit)

        // TBZ: bit == 0; TBNZ: bit != 0
        auto pred = (a64.op == farmdec::A64_TBZ) ? llvm::CmpInst::Predicate::ICMP_EQ : llvm::CmpInst::Predicate::ICMP_NE;
        auto do_branch = irb.CreateICmp(pred, bit, irb.getIntN(bits, 0));
        auto on_true = PCRel(a64.tbz.offset);
        auto on_false = PCRel(inst.len()); // next instr
        SetReg(ArchReg::IP, Facet::I64, irb.CreateSelect(do_branch, on_true, on_false));
        return true;
    }
    case farmdec::A64_UDIV: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = GetGp(a64.rm, w32);

        // div(a,0) => 0. We need to make sure the division does not fault
        // _before_ we even arrive at the final select where we discard it.
        // Hence safe_rhs.
        auto is_zero = irb.CreateICmpEQ(rhs, irb.getIntN(bits, 0));
        auto safe_rhs = irb.CreateSelect(is_zero, irb.getIntN(bits, 1), rhs);
        auto val = irb.CreateSelect(is_zero, irb.getIntN(bits, 0), irb.CreateUDiv(lhs, safe_rhs));

        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_SDIV: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = GetGp(a64.rm, w32);

        // See above
        auto is_zero = irb.CreateICmpEQ(rhs, irb.getIntN(bits, 0));
        auto safe_rhs = irb.CreateSelect(is_zero, irb.getIntN(bits, 1), rhs);

        // sdiv(INT_MIN,-1) => INT_MIN. sdiv overflow is also UB, so we need
        // the same precautions as for the div-by-zero case.
        auto int_min = irb.getIntN(bits, (w32) ? INT32_MIN : INT64_MIN);
        auto is_min = irb.CreateAnd(irb.CreateICmpEQ(lhs, int_min), irb.CreateICmpEQ(rhs, irb.getIntN(bits, -1)));
        safe_rhs = irb.CreateSelect(is_min, irb.getIntN(bits, 1), safe_rhs);
        auto val = irb.CreateSelect(is_min, int_min, irb.CreateSDiv(lhs, safe_rhs));
        val = irb.CreateSelect(is_zero, irb.getIntN(bits, 0), val);
        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_LSLV: {
        auto lhs = GetGp(a64.rn, w32);
        auto amount = irb.CreateAnd(GetGp(a64.rm, w32), (w32) ? 0x1f : 0x3f); // lowest 5 or 6 bit
        SetGp(a64.rd, w32, irb.CreateShl(lhs, amount));
        break;
    }
    case farmdec::A64_LSRV: {
        auto lhs = GetGp(a64.rn, w32);
        auto amount = irb.CreateAnd(GetGp(a64.rm, w32), (w32) ? 0x1f : 0x3f); // lowest 5 or 6 bit
        SetGp(a64.rd, w32, irb.CreateLShr(lhs, amount));
        break;
    }
    case farmdec::A64_ASRV: {
        auto lhs = GetGp(a64.rn, w32);
        auto amount = irb.CreateAnd(GetGp(a64.rm, w32), (w32) ? 0x1f : 0x3f); // lowest 5 or 6 bit
        SetGp(a64.rd, w32, irb.CreateAShr(lhs, amount));
        break;
    }
    case farmdec::A64_RORV: {
        auto lhs = GetGp(a64.rn, w32);
        auto amount = irb.CreateAnd(GetGp(a64.rm, w32), (w32) ? 0x1f : 0x3f); // lowest 5 or 6 bit
        auto mod = irb.GetInsertBlock()->getModule();
        auto fn = llvm::Intrinsic::getDeclaration(mod, llvm::Intrinsic::fshr, {lhs->getType()});
        SetGp(a64.rd, w32, irb.CreateCall(fn, {lhs, lhs, amount}));
        break;
    }
    case farmdec::A64_AND_SHIFTED:
    case farmdec::A64_TST_SHIFTED:
    case farmdec::A64_BIC: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount);
        if (a64.op == farmdec::A64_BIC) {
            rhs = irb.CreateNot(rhs);
        }
        auto val = irb.CreateAnd(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcLogic(val);
        }
        break;
    }
    case farmdec::A64_ORR_SHIFTED:
    case farmdec::A64_ORN: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount);
        if (a64.op == farmdec::A64_ORN) {
            rhs = irb.CreateNot(rhs);
        }
        auto val = irb.CreateOr(lhs, rhs);
        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_MOV_REG:
        SetGp(a64.rd, w32, GetGp(a64.rm, w32)); // rd := rm
        break;
    case farmdec::A64_MVN:
        SetGp(a64.rd, w32, irb.CreateNot(GetGp(a64.rm, w32))); // rd := ~rm
        break;
    case farmdec::A64_EOR_SHIFTED:
    case farmdec::A64_EON: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount);
        if (a64.op == farmdec::A64_EON) {
            rhs = irb.CreateNot(rhs);
        }
        auto val = irb.CreateXor(lhs, rhs);
        SetGp(a64.rd, w32, val);
        break;
    }
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
        auto rhs = Extend(GetGp(a64.rm, w32), w32, static_cast<farmdec::ExtendType>(a64.extend.type), a64.extend.lsl);
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
        auto rhs = Extend(GetGp(a64.rm, w32), w32, static_cast<farmdec::ExtendType>(a64.extend.type), a64.extend.lsl);
        auto val = irb.CreateSub(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags) {
            FlagCalcSub(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_CSEL: { // rd := (cond) ? rn : rm;
        auto on_true = GetGp(a64.rn, w32);
        auto on_false = GetGp(a64.rm, w32);
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), on_true, on_false));
        break;
    }
    case farmdec::A64_CSINC: { // rd := (cond) ? rn : rm+1;
        auto on_true = GetGp(a64.rn, w32);
        auto on_false = irb.CreateAdd(GetGp(a64.rm, w32), irb.getIntN(bits, 1));
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), on_true, on_false));
        break;
    }
    case farmdec::A64_CINC: { // rd := (cond) ? rn+1 : rn;
        auto val = GetGp(a64.rn, w32);
        auto on_true = irb.CreateAdd(val, irb.getIntN(bits, 1));
        auto on_false = val;
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), on_true, on_false));
        break;
    }
    case farmdec::A64_CSET: { // rd := (cond) ? 1 : 0;
        SetGp(a64.rd, w32, irb.CreateZExt(IsTrue(fad_get_cond(a64.flags)), (w32) ? irb.getInt32Ty() : irb.getInt64Ty()));
        break;
    }
    case farmdec::A64_CSINV: { // rd := (cond) ? rn : ~rm;
        auto on_true = GetGp(a64.rn, w32);
        auto on_false = irb.CreateNot(GetGp(a64.rm, w32));
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), on_true, on_false));
        break;
    }
    case farmdec::A64_CINV: { // rd := (cond) ? ~rn : rn;
        auto val = GetGp(a64.rn, w32);
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), irb.CreateNot(val), val));
        break;
    }
    case farmdec::A64_CSETM: { // rd := (cond) ? -1 : 0;
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), irb.getIntN(bits, -1), irb.getIntN(bits, 0)));
        break;
    }
    case farmdec::A64_CSNEG: { // rd := (cond) ? rn : -rm;
        auto on_true = GetGp(a64.rn, w32);
        auto on_false = irb.CreateNeg(GetGp(a64.rm, w32));
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), on_true, on_false));
        break;
    }
    case farmdec::A64_CNEG: { // rd := (cond) ? -rn : rn;
        auto val = GetGp(a64.rn, w32);
        SetGp(a64.rd, w32, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), irb.CreateNeg(val), val));
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
        if (mode == farmdec::AM_SIMPLE) { // AM_SIMPLE → LDAR, LDLAR
            load->setOrdering(Ordering(static_cast<farmdec::MemOrdering>(a64.ldst_order.load)));
            load->setAlignment(llvm::Align(srcty->getPrimitiveSizeInBits() / 8));
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
        if (mode == farmdec::AM_SIMPLE) { // AM_SIMPLE → STLR, STLLR
            store->setOrdering(Ordering(static_cast<farmdec::MemOrdering>(a64.ldst_order.store)));
            store->setAlignment(llvm::Align(dstty->getPrimitiveSizeInBits() / 8));
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

void Lifter::FlagCalcLogic(llvm::Value* res) {
    auto zero = llvm::Constant::getNullValue(res->getType());
    SetFlag(Facet::ZF, irb.CreateICmpEQ(res, zero));
    SetFlag(Facet::SF, irb.CreateICmpSLT(res, zero));
    SetFlag(Facet::CF, irb.getFalse());
    SetFlag(Facet::OF, irb.getFalse());
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
    case farmdec::SH_ROR:
        auto mod = irb.GetInsertBlock()->getModule();
        auto fn = llvm::Intrinsic::getDeclaration(mod, llvm::Intrinsic::fshr, {v->getType()});
        return irb.CreateCall(fn, {v, v, irb.getIntN(v->getType()->getIntegerBitWidth(), amount)});
    }
    return v; // no change
}

// Zero- or sign-extend the lowest 8, 16, 32, or 64 bits of the value v
// to 32 or 64 bits depending on w32 and optionally apply a left shift.
llvm::Value* Lifter::Extend(llvm::Value* v, bool w32, farmdec::ExtendType ext, uint32_t lsl) {
    llvm::Type* srcty = nullptr;
    llvm::Type* dstty = (w32) ? irb.getInt32Ty() : irb.getInt64Ty();
    bool sign_extend = false;
    switch (ext) {
    case farmdec::UXTB: srcty = irb.getInt8Ty();  sign_extend = false; break;
    case farmdec::UXTH: srcty = irb.getInt16Ty(); sign_extend = false; break;
    case farmdec::UXTW: srcty = irb.getInt32Ty(); sign_extend = false; break;
    case farmdec::UXTX: srcty = irb.getInt64Ty(); sign_extend = false; break;
    case farmdec::SXTB: srcty = irb.getInt8Ty();  sign_extend = true; break;
    case farmdec::SXTH: srcty = irb.getInt16Ty(); sign_extend = true; break;
    case farmdec::SXTW: srcty = irb.getInt32Ty(); sign_extend = true; break;
    case farmdec::SXTX: srcty = irb.getInt64Ty(); sign_extend = true; break;
    }
    v = irb.CreateTruncOrBitCast(v, srcty);
    auto extended = (sign_extend) ? irb.CreateSExt(v, dstty) : irb.CreateZExt(v, dstty);
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

// Returns a i1 that is 1 if the A64 condition is true, 0 otherwise.
// Look into any intro of the A64 instruction set for reference.
llvm::Value* Lifter::IsTrue(farmdec::Cond cond) {
    llvm::Value* res = nullptr; // positive result, inverted iff LSB(cond) == 1
    switch (cond) {
    case farmdec::COND_EQ: case farmdec::COND_NE: res = GetFlag(Facet::ZF); break;
    case farmdec::COND_HS: case farmdec::COND_LO: res = GetFlag(Facet::CF); break;
    case farmdec::COND_MI: case farmdec::COND_PL: res = GetFlag(Facet::SF); break;
    case farmdec::COND_VS: case farmdec::COND_VC: res = GetFlag(Facet::OF); break;
    case farmdec::COND_HI: case farmdec::COND_LS:
        res = irb.CreateAnd(GetFlag(Facet::CF), irb.CreateNot(GetFlag(Facet::ZF)));
        break;
    case farmdec::COND_GE: case farmdec::COND_LT:
        res = irb.CreateICmpEQ(GetFlag(Facet::SF), GetFlag(Facet::OF));
        break;
    case farmdec::COND_GT: case farmdec::COND_LE:
        res = irb.CreateAnd(irb.CreateNot(GetFlag(Facet::ZF)), irb.CreateICmpEQ(GetFlag(Facet::SF), GetFlag(Facet::OF)));
        break;
    case farmdec::COND_AL: case farmdec::COND_NV:
        return irb.getTrue(); // Both AL and NV yield true, so return directly without inverting.
    default:
        assert(false && "invalid condition code");
    }
    return (cond & 1) ? irb.CreateNot(res) : res;
}

llvm::AtomicOrdering Lifter::Ordering(farmdec::MemOrdering mo) {
    switch (mo) {
    case farmdec::MO_NONE:       return llvm::AtomicOrdering::NotAtomic;
    case farmdec::MO_ACQUIRE:    return llvm::AtomicOrdering::Acquire;
    case farmdec::MO_LO_ACQUIRE: return llvm::AtomicOrdering::Acquire; // Stronger than LOAcquire
    case farmdec::MO_ACQUIRE_PC: return llvm::AtomicOrdering::Acquire; // XXX is this correct? RCpc is weaker than normal Acq, right?
    case farmdec::MO_RELEASE:    return llvm::AtomicOrdering::Release;
    case farmdec::MO_LO_RELEASE: return llvm::AtomicOrdering::Release; // Stronger than LORelease
    }
    assert(false && "invalid memory ordering");
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

// Return up to 64 ones.
static uint64_t ones(int n) {
    return (n >= 64) ? ~((uint64_t)0) : ((uint64_t) 1 << n) - 1;
}

// Extract a bitfield of that width starting at bit position lsb. Essentially
// the standard C idiom
//
//     return (v >> lsb) & ones(width);
//
llvm::Value* Lifter::Extract(llvm::Value* v, bool w32, unsigned lsb, unsigned width) {
    int bits = (w32) ? 32 : 64;
    v = irb.CreateLShr(v, irb.getIntN(bits, lsb));
    return irb.CreateAnd(v, irb.getIntN(bits, ones(width)));
}

// Move the #width least significant bits to the bit position lsb.
//
//     return (v & ones(width)) << lsb;
//
llvm::Value* Lifter::MoveField(llvm::Value* v, bool w32, unsigned lsb, unsigned width) {
    int bits = (w32) ? 32 : 64;
    uint64_t mask = ones(width);
    v = irb.CreateAnd(v, irb.getIntN(bits, mask));
    return irb.CreateShl(v, irb.getIntN(bits, lsb));
}

} // namespace rellume::aarch64

/**
 * @}
 **/
