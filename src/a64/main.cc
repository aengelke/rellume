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
        // See if it's a valid SIMD instruction before giving up.
        if (LiftSIMD(a64)) {
            break;
        }

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
    case farmdec::A64_TST_IMM:
        LiftBinOp(a64, w32, llvm::Instruction::And, BinOpKind::IMM, set_flags);
        break;
    case farmdec::A64_ORR_IMM: LiftBinOp(a64, w32, llvm::Instruction::Or, BinOpKind::IMM, set_flags); break;
    case farmdec::A64_EOR_IMM: LiftBinOp(a64, w32, llvm::Instruction::Xor, BinOpKind::IMM, set_flags); break;
    case farmdec::A64_ADD_IMM:
    case farmdec::A64_MOV_SP:
    case farmdec::A64_CMN_IMM:
        LiftBinOp(a64, w32, llvm::Instruction::Add, BinOpKind::IMM, set_flags);
        break;
    case farmdec::A64_SUB_IMM:
    case farmdec::A64_CMP_IMM:
        LiftBinOp(a64, w32, llvm::Instruction::Sub, BinOpKind::IMM, set_flags);
        break;
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
    case farmdec::A64_EXTR: {
        auto longty = irb.getIntNTy(bits*2);
        auto high = irb.CreateShl(irb.CreateZExt(GetGp(a64.rn, w32), longty), bits);
        auto low = irb.CreateZExt(GetGp(a64.rm, w32), longty);
        auto long_val = irb.CreateOr(high, low); // long_val = high:low

        auto mask = irb.getIntN(bits*2, ones(bits)); // mask: 00000...(bits x 0):11111...(bits x 1)
        long_val = irb.CreateLShr(long_val, irb.getIntN(bits*2, a64.imm));
        long_val = irb.CreateAnd(long_val, mask);

        SetGp(a64.rd, w32, irb.CreateTrunc(long_val, (w32) ? irb.getInt32Ty() : irb.getInt64Ty()));
        break;
    }
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

    case farmdec::A64_UDF:
    case farmdec::A64_HVC:
    case farmdec::A64_SMC:
    case farmdec::A64_BRK:
    case farmdec::A64_HLT:
    case farmdec::A64_DCPS1:
    case farmdec::A64_DCPS2:
    case farmdec::A64_DCPS3:
*/
    case farmdec::A64_HINT:
        break; // All Hints can be treated as no-ops.
    case farmdec::A64_DMB:
        switch (a64.imm) {
        case 0xf: case 0xb: case 0x7: case 0x3: // SY, ISH, NSH, OSH (reads and writes)
            irb.CreateFence(llvm::AtomicOrdering::SequentiallyConsistent); break;
        case 0xe: case 0xa: case 0x6: case 0x2: // ST, ISHST, NSHST, OSHST (writes)
            irb.CreateFence(llvm::AtomicOrdering::Release); break;
        case 0xd: case 0x9: case 0x5: case 0x1: // LD, ISHLD, NSHLD, OSHLD (reads)
            irb.CreateFence(llvm::AtomicOrdering::Acquire); break;
        default:
            assert(false && "bad DMB CRm value");
        }
        break;
/*
    Intentionally unimplemented since it can only affect PSTATE.D, .A, .I, .F, (DAIF),
    .SP, .SSBS, .PAN, .UAO, .DIT, all of which concern system decisions outside of
    LLVM/Rellume scope (e.g. exception masking, timing control).

    case farmdec::A64_MSR_IMM:
*/
    case farmdec::A64_MSR_REG: {
        // a64.imm is the encoded system register (op0:op1:CRn:CRm:op2).
        switch (a64.imm) {
        case 0xde82: {// TPIDR_EL0
            unsigned idx = SptrIdx::aarch64::TPIDR_EL0;
            irb.CreateStore(GetGp(a64.rt, /*w32=*/false), fi.sptr[idx]);
            break;
        }
        case 0xda10: {// NZCV (bits 31-28)
            auto nzcv = GetGp(a64.rt, /*w32=*/false);
            SetFlag(Facet::SF, irb.CreateTrunc(irb.CreateLShr(nzcv, 31), irb.getInt1Ty()));
            SetFlag(Facet::ZF, irb.CreateTrunc(irb.CreateLShr(nzcv, 30), irb.getInt1Ty()));
            SetFlag(Facet::CF, irb.CreateTrunc(irb.CreateLShr(nzcv, 29), irb.getInt1Ty()));
            SetFlag(Facet::OF, irb.CreateTrunc(irb.CreateLShr(nzcv, 28), irb.getInt1Ty()));
            break; // XXX
        }
        case 0xda20: // FPCR
            break; // XXX no meaningful re-configuration possible at the moment
        case 0xda21: // FPSR
            break; // XXX reset QC if overwritten (once it exists as a flag)
        default:
            return false;
        }
        break;
    }
    case farmdec::A64_CFINV:
        SetFlag(Facet::CF, irb.CreateNot(GetFlag(Facet::CF)));
        break;
    case farmdec::A64_SYS: {
        // DC ZVA -- zeroes a block of bytes, with the size stored in DCZID_EL0. Both
        // glibc and musl want 64-byte blocks, so do exactly that.
        if (a64.sys.op1 == 3 && a64.sys.op2 == 1 && a64.sys.crn == 7 && a64.sys.crm == 4) {
            auto addr = GetGp(a64.rt, /*w32=*/false);                  // may point anywhere into the block
            auto start = irb.CreateAnd(addr, irb.getInt64(~0x3fuL)); // actual start address of block
            auto ptr = irb.CreateIntToPtr(start, irb.getInt8PtrTy());
            irb.CreateMemSet(ptr, irb.getInt8(0), irb.getInt32(64), llvm::Align(64));
        } else {
            return false;
        }
        break;
    }
    case farmdec::A64_MRS: {
        // a64.imm is the encoded system register (op0:op1:CRn:CRm:op2).
        switch (a64.imm) {
        case 0xde82: {// TPIDR_EL0
            unsigned idx = SptrIdx::aarch64::TPIDR_EL0;
            auto res = irb.CreateLoad(irb.getInt64Ty(), fi.sptr[idx]);
            SetGp(a64.rt, /*w32=*/false, res);
            break;
        }
        case 0xda10: {// NZCV (bits 31-28)
            llvm::Value* nzcv = irb.getIntN(64, 0);
            nzcv = irb.CreateOr(nzcv, irb.CreateShl(irb.CreateZExt(GetFlag(Facet::SF), irb.getInt64Ty()), 31)); // nzcv |= n << 31
            nzcv = irb.CreateOr(nzcv, irb.CreateShl(irb.CreateZExt(GetFlag(Facet::ZF), irb.getInt64Ty()), 30)); // nzcv |= z << 30
            nzcv = irb.CreateOr(nzcv, irb.CreateShl(irb.CreateZExt(GetFlag(Facet::CF), irb.getInt64Ty()), 29)); // nzcv |= c << 29
            nzcv = irb.CreateOr(nzcv, irb.CreateShl(irb.CreateZExt(GetFlag(Facet::OF), irb.getInt64Ty()), 28)); // nzcv |= v << 28
            SetGp(a64.rt, /*w32=*/false, nzcv);
            break;
        }
        case 0xda20: // FPCR
            // Bits: AHP(26), DN(25), FZ(24), RMode(23:22), IDE(15), IXE(12), UFE(11), OFE(10), DZE(9), IOE(8)
            // All bits = 0 indicates IEEE 754 with round to nearest and no exceptions.
            SetGp(a64.rt, /*w32=*/false, irb.getIntN(64, 0));
            break;
        case 0xda21: // FPSR
            // XXX We cannot check for "normal" FP errors, but we should store QC in the CPU state and output it here.
            // Bits: QC(27), IDC(7), IXC(4), UFC(3), OFC(2), DZC(1), IOC(0).
            SetGp(a64.rt, /*w32=*/false, irb.getIntN(64, 0));
            break;
        case 0xd807: // DCZID_EL0
            SetGp(a64.rt, /*w32=*/false, irb.getIntN(64, 0x04)); // 4 → 64-byte block size wanted by musl and glibc
            break;
        case 0xc000: // MIDR_EL1
            // Bits: Implementer(31:24), Variant(23:20), Architecture(19:16), PartNum(15:4), Revision(3:0).
            // We set Architecture to 0b1111 to indicate ARMv8, but we do not implement the feature registers
            // ID_AA64* because they are not commonly read in userspace (the compiler knows the features of
            // the target, so why would the generated code check them?).
            SetGp(a64.rt, /*w32=*/false, irb.getIntN(64, 0xfuL << 16));
            break;
        default:
            return false;
        }
        break;
    }
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
    case farmdec::A64_RBIT:
        SetGp(a64.rd, w32, irb.CreateUnaryIntrinsic(llvm::Intrinsic::bitreverse, GetGp(a64.rn, w32)));
        break;
    case farmdec::A64_REV16: { // swap bytes of every halfword independently
        auto vecty = llvm::VectorType::get(irb.getInt16Ty(), (w32) ? 2 : 4, false); // i32 → <2xi16>, i64 → <4xi16>
        auto invec = irb.CreateBitCast(GetGp(a64.rn, w32), vecty);
        auto outvec = irb.CreateUnaryIntrinsic(llvm::Intrinsic::bswap, invec);
        SetGp(a64.rd, w32, irb.CreateBitCast(outvec, irb.getIntNTy(bits)));
        break;
    }
    case farmdec::A64_REV: // swap order of entire (double)word
        SetGp(a64.rd, w32, irb.CreateUnaryIntrinsic(llvm::Intrinsic::bswap, GetGp(a64.rn, w32)));
        break;
    case farmdec::A64_REV32: { // swap bytes of every word independently
        auto vecty = llvm::VectorType::get(irb.getInt32Ty(), (w32) ? 1 : 2, false); // i32 → <1xi32>, i64 → <2xi32>
        auto invec = irb.CreateBitCast(GetGp(a64.rn, w32), vecty);
        auto outvec = irb.CreateUnaryIntrinsic(llvm::Intrinsic::bswap, invec);
        SetGp(a64.rd, w32, irb.CreateBitCast(outvec, irb.getIntNTy(bits)));
        break;
    }
    case farmdec::A64_CLZ: {
        auto val = GetGp(a64.rn, w32);
        auto intr = llvm::Intrinsic::ctlz;
        SetGp(a64.rd, w32, irb.CreateBinaryIntrinsic(intr, val,
                                                    /*zeroundef=*/irb.getFalse()));
        break;
    }
    case farmdec::A64_AND_SHIFTED:
    case farmdec::A64_TST_SHIFTED:
        LiftBinOp(a64, w32, llvm::Instruction::And, BinOpKind::SHIFT, set_flags);
        break;
    case farmdec::A64_BIC:         LiftBinOp(a64, w32, llvm::Instruction::And, BinOpKind::SHIFT, set_flags, /*invert_rhs=*/true); break;
    case farmdec::A64_ORR_SHIFTED: LiftBinOp(a64, w32, llvm::Instruction::Or, BinOpKind::SHIFT, set_flags); break;
    case farmdec::A64_ORN:         LiftBinOp(a64, w32, llvm::Instruction::Or, BinOpKind::SHIFT, set_flags, /*invert_rhs=*/true); break;
    case farmdec::A64_MOV_REG:
        SetGp(a64.rd, w32, GetGp(a64.rm, w32)); // rd := rm
        break;
    case farmdec::A64_MVN:
        SetGp(a64.rd, w32, irb.CreateNot(GetGp(a64.rm, w32))); // rd := ~rm
        break;
    case farmdec::A64_EOR_SHIFTED: LiftBinOp(a64, w32, llvm::Instruction::Xor, BinOpKind::SHIFT, set_flags); break;
    case farmdec::A64_EON:         LiftBinOp(a64, w32, llvm::Instruction::Xor, BinOpKind::SHIFT, set_flags, /*invert_rhs=*/true); break;
    case farmdec::A64_ADD_SHIFTED:
    case farmdec::A64_CMN_SHIFTED:
        LiftBinOp(a64, w32, llvm::Instruction::Add, BinOpKind::SHIFT, set_flags);
        break;
    case farmdec::A64_SUB_SHIFTED:
    case farmdec::A64_NEG:
    case farmdec::A64_CMP_SHIFTED:
        LiftBinOp(a64, w32, llvm::Instruction::Sub, BinOpKind::SHIFT, set_flags);
        break;
    case farmdec::A64_ADD_EXT:
    case farmdec::A64_CMN_EXT:
        LiftBinOp(a64, w32, llvm::Instruction::Add, BinOpKind::EXT, set_flags);
        break;
    case farmdec::A64_SUB_EXT:
    case farmdec::A64_CMP_EXT:
        LiftBinOp(a64, w32, llvm::Instruction::Sub, BinOpKind::EXT, set_flags);
        break;
    case farmdec::A64_ADC:
    case farmdec::A64_SBC:
    case farmdec::A64_NGC: {
        // ADC: Rd := Rn + Rm + carry.
        // SBC: Rd := Rn - (Rm + NOT(carry)).
        bool add = (a64.op == farmdec::A64_ADC);
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = GetGp(a64.rm, w32);
        auto cf = GetFlag(Facet::CF);
        cf = (add) ? cf : irb.CreateNot(cf);
        rhs = irb.CreateAdd(rhs, irb.CreateZExt(cf, irb.getIntNTy(bits)));
        auto val = (add) ? irb.CreateAdd(lhs, rhs) : irb.CreateSub(lhs, rhs);
        SetGp(a64.rd, w32, val);
        if (set_flags && add) {
            FlagCalcAdd(val, lhs, rhs);
        } else if (set_flags && !add) {
            FlagCalcSub(val, lhs, rhs);
        }
        break;
    }
    case farmdec::A64_CCMN_REG:
    case farmdec::A64_CCMP_REG: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = GetGp(a64.rm, w32);
        LiftCCmp(lhs, rhs, fad_get_cond(a64.flags), a64.ccmp.nzcv, (a64.op == farmdec::A64_CCMN_REG));
        break;
    }
    case farmdec::A64_CCMN_IMM:
    case farmdec::A64_CCMP_IMM: {
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = irb.getIntN(bits, a64.ccmp.imm5);
        LiftCCmp(lhs, rhs, fad_get_cond(a64.flags), a64.ccmp.nzcv, (a64.op == farmdec::A64_CCMN_IMM));
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
    case farmdec::A64_MUL:
        // LLVM and AArch64 both treat overflow by simply rolling over. No need
        // for extending to i(N*2) like in the other cases.
        SetGp(a64.rd, w32, irb.CreateMul(GetGp(a64.rn, w32), GetGp(a64.rm, w32)));
        break;
    case farmdec::A64_MADD:
    case farmdec::A64_MSUB:
    case farmdec::A64_MNEG: {
        bool sub = (a64.op == farmdec::A64_MSUB || a64.op == farmdec::A64_MNEG);
        auto lhs = GetGp(a64.rn, w32);
        auto rhs = GetGp(a64.rm, w32);
        auto base = GetGp(a64.ra, w32);

        // Yes, lhs*rhs might overflow before the addition, but this is irrelevant
        // to the final value.
        auto prod = irb.CreateMul(lhs, rhs);
        auto val = (sub) ? irb.CreateSub(base, prod) : irb.CreateAdd(base, prod);
        SetGp(a64.rd, w32, val);
        break;
    }
    case farmdec::A64_SMADDL:
    case farmdec::A64_SMULL:
        SetGp(a64.rd, /*w32=*/false, MulAddSub(GetGp(a64.ra, /*w32=*/false), llvm::Instruction::Add,
                             GetGp(a64.rn, /*w32=*/true), GetGp(a64.rm, /*w32=*/true), llvm::Instruction::SExt));
        break;
    case farmdec::A64_SMSUBL:
    case farmdec::A64_SMNEGL:
        SetGp(a64.rd, /*w32=*/false, MulAddSub(GetGp(a64.ra, /*w32=*/false), llvm::Instruction::Sub,
                             GetGp(a64.rn, /*w32=*/true), GetGp(a64.rm, /*w32=*/true), llvm::Instruction::SExt));
        break;
    case farmdec::A64_SMULH: {
        auto lhs = GetGp(a64.rn, /*w32=*/false);
        auto rhs = GetGp(a64.rm, /*w32=*/false);
        auto long_val = MulAddSub(irb.getIntN(64, 0), llvm::Instruction::Add, lhs, rhs, llvm::Instruction::SExt);
        auto high_half = irb.CreateLShr(long_val, irb.getIntN(128, 64));
        SetGp(a64.rd, /*w32=*/false, irb.CreateTrunc(high_half, irb.getInt64Ty()));
        break;
    }
    case farmdec::A64_UMADDL:
    case farmdec::A64_UMULL:
        SetGp(a64.rd, /*w32=*/false, MulAddSub(GetGp(a64.ra, /*w32=*/false), llvm::Instruction::Add,
                             GetGp(a64.rn, true), GetGp(a64.rm, /*w32=*/true), llvm::Instruction::ZExt));
        break;
    case farmdec::A64_UMSUBL:
    case farmdec::A64_UMNEGL:
        SetGp(a64.rd, /*w32=*/false, MulAddSub(GetGp(a64.ra, /*w32=*/false), llvm::Instruction::Sub,
                             GetGp(a64.rn, true), GetGp(a64.rm, /*w32=*/true), llvm::Instruction::ZExt));
        break;
    case farmdec::A64_UMULH: {
        auto lhs = GetGp(a64.rn, /*w32=*/false);
        auto rhs = GetGp(a64.rm, /*w32=*/false);
        auto long_val = MulAddSub(irb.getIntN(64, 0), llvm::Instruction::Add, lhs, rhs, llvm::Instruction::ZExt);
        auto high_half = irb.CreateLShr(long_val, irb.getIntN(128, 64));
        SetGp(a64.rd, /*w32=*/false, irb.CreateTrunc(high_half, irb.getInt64Ty()));
        break;
    }
    // Exclusive Loads and Stores always come in pairs and appear in a loop: first the
    // value is loaded and the address thus locked (LDXR), then the value is compared
    // and if necessary, a new value is stored to the previously locked address (STXR).
    // If the store fails (Rs == 1), the loop continues.
    //
    // Because we do not support true parallelism and because they always appear in
    // matching pairs, we can model LDXR/STXR as normal loads and stores, with STXR
    // always succeeding (Rs == 0).
    case farmdec::A64_LDXR:
    case farmdec::A64_LDXP:
        LiftLoadStore(a64, w32);
        break;
    case farmdec::A64_STXR:
    case farmdec::A64_STXP:
        LiftLoadStore(a64, w32);
        SetGp(a64.ldst_order.rs, w32, irb.getIntN(bits, 0));
        break;
    case farmdec::A64_LDP:
    case farmdec::A64_STP:
    case farmdec::A64_LDR:
    case farmdec::A64_STR:
        LiftLoadStore(a64, w32);
        break;
    case farmdec::A64_PRFM: {
        auto addr = Addr(irb.getInt8Ty(), a64); // i8*

        // prfop = type(2):target(2);policy(1)
        // type: 0=PLD (load), 1=PLI (instructions), 2=PST (store)
        // target: 0=L1, 1=L2, 2=L3
        // policy: 0=KEEP, 1=STRM
        int prfop = a64.rt;

        bool is_write, is_data;
        switch ((prfop >> 3) & 3) { // type(2)
        case 0: is_write = false; is_data = true; break;
        case 1: is_write = false; is_data = false; break;
        case 2: is_write = true;  is_data = true; break;
        default:
            assert(false && "bad PRFM prfop");
        }

        int locality = 3 - ((prfop>>1) & 3); // 3 - target

        llvm::SmallVector<llvm::Type*, 1> tys;
        tys.push_back(irb.getInt8PtrTy());
        auto mod = irb.GetInsertBlock()->getModule();
        auto fn = llvm::Intrinsic::getDeclaration(mod, llvm::Intrinsic::prefetch, tys);
        irb.CreateCall(fn, {addr, irb.getInt32(is_write), irb.getInt32(locality), irb.getInt32(is_data)});
        break;
    }
    case farmdec::A64_LDP_FP:
    case farmdec::A64_STP_FP:
    case farmdec::A64_LDR_FP:
    case farmdec::A64_STR_FP:
        LiftLoadStore(a64, w32, /*fp=*/true);
        break;
    case farmdec::A64_FCVT_GPR: {
        assert(a64.fcvt.fbits == 0); // XXX fixed-point currently not supported

        auto fp = GetScalar(a64.rn, fad_get_prec(a64.flags));
        auto rounded = Round(fp, static_cast<farmdec::FPRounding>(a64.fcvt.mode));
        auto ity = irb.getIntNTy((w32) ? 32 : 64);
        auto ival = (a64.fcvt.sgn) ? irb.CreateFPToSI(rounded, ity) : irb.CreateFPToUI(rounded, ity);
        SetGp(a64.rd, w32, ival);
        break;
    }
    case farmdec::A64_CVTF: {     // GPR(int|fixed) → Sca(fp)
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        assert(a64.fcvt.fbits == 0); // XXX fixed-point currently not supported

        auto ival = GetGp(a64.rn, w32);
        auto fp = (a64.fcvt.sgn) ? irb.CreateSIToFP(ival, TypeOf(prec)) : irb.CreateUIToFP(ival, TypeOf(prec));
        SetScalar(a64.rd, fp);
        break;
    }
    // case farmdec::A64_FJCVTZS: never/seldom seen in the wild
    case farmdec::A64_FRINT:
    case farmdec::A64_FRINTX: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        assert(a64.frint.bits == 0); // XXX frint32*, frint64* currently not supported

        bool exact = (a64.op == farmdec::A64_FRINTX);
        SetScalar(a64.rd, Round(GetScalar(a64.rn, prec), static_cast<farmdec::FPRounding>(a64.frint.mode), exact));
        break;
    }
    case farmdec::A64_FCVT_H:
        assert(false && "FP half precision not supported");
        break;
    case farmdec::A64_FCVT_S: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto val = GetScalar(a64.rn, prec);
        SetScalar(a64.rd, irb.CreateFPCast(val, irb.getFloatTy()));
        break;
    }
    case farmdec::A64_FCVT_D: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto val = GetScalar(a64.rn, prec);
        SetScalar(a64.rd, irb.CreateFPCast(val, irb.getDoubleTy()));
        break;
    }
    case farmdec::A64_FABS:
        LiftIntrinsicFP(llvm::Intrinsic::fabs, fad_get_prec(a64.flags), a64.rd, a64.rn);
        break;
    case farmdec::A64_FNEG: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto val = GetScalar(a64.rn, prec);
        SetScalar(a64.rd, irb.CreateFNeg(val));
        break;
    }
    case farmdec::A64_FSQRT:
        LiftIntrinsicFP(llvm::Intrinsic::sqrt, fad_get_prec(a64.flags), a64.rd, a64.rn);
        break;
    case farmdec::A64_FMUL:
        LiftBinOpFP(llvm::Instruction::FMul, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FDIV:
        LiftBinOpFP(llvm::Instruction::FDiv, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FADD:
        LiftBinOpFP(llvm::Instruction::FAdd, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FSUB:
        LiftBinOpFP(llvm::Instruction::FSub, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FMAX:
        LiftIntrinsicFP(llvm::Intrinsic::maximum, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FMAXNM:
        LiftIntrinsicFP(llvm::Intrinsic::maxnum, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FMIN:
        LiftIntrinsicFP(llvm::Intrinsic::minimum, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FMINNM:
        LiftIntrinsicFP(llvm::Intrinsic::minnum, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm);
        break;
    case farmdec::A64_FNMUL: { // - (a*b) (should be fused, but is not)
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = GetScalar(a64.rm, prec);
        SetScalar(a64.rd, irb.CreateFNeg(irb.CreateFMul(lhs, rhs)));
        break;
    }
    case farmdec::A64_FMADD: // Fused (a*b) + c
        LiftIntrinsicFP(llvm::Intrinsic::fma, fad_get_prec(a64.flags), a64.rd, a64.rn, a64.rm, a64.ra);
        break;
    case farmdec::A64_FMSUB: { // (-(a*b)) + c (should be fused, but is not)
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = GetScalar(a64.rm, prec);
        auto add = GetScalar(a64.ra, prec);
        SetScalar(a64.rd, irb.CreateFAdd(irb.CreateFNeg(irb.CreateFMul(lhs, rhs)), add));
        break;
    }
    case farmdec::A64_FNMADD: { // (-(a*b)) - c (should be fused, but is not)
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = GetScalar(a64.rm, prec);
        auto sub = GetScalar(a64.ra, prec);
        SetScalar(a64.rd, irb.CreateFSub(irb.CreateFNeg(irb.CreateFMul(lhs, rhs)), sub));
        break;
    }
    case farmdec::A64_FNMSUB: { // (a*b) - c (should be fused, but is not)
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = GetScalar(a64.rm, prec);
        auto sub = GetScalar(a64.ra, prec);
        SetScalar(a64.rd, irb.CreateFSub(irb.CreateFMul(lhs, rhs), sub));
        break;
    }
    case farmdec::A64_FCMP_REG:
    case farmdec::A64_FCMPE_REG: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = GetScalar(a64.rm, prec);
        FlagCalcFP(lhs, rhs);
        break;
    }
    case farmdec::A64_FCMP_ZERO:
    case farmdec::A64_FCMPE_ZERO: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = llvm::ConstantFP::get(TypeOf(prec), 0.0);
        FlagCalcFP(lhs, rhs);
        break;
    }
    case farmdec::A64_FCCMP:
    case farmdec::A64_FCCMPE: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto lhs = GetScalar(a64.rn, prec);
        auto rhs = GetScalar(a64.rm, prec);
        LiftCCmp(lhs, rhs, fad_get_cond(a64.flags), a64.ccmp.nzcv, /*ccmn=*/false, /*fp=*/true);
        break;
    }
    case farmdec::A64_FCSEL: { // rd := (cond) ? rn : rm;
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        auto on_true = GetScalar(a64.rn, prec);
        auto on_false = GetScalar(a64.rm, prec);
        SetScalar(a64.rd, irb.CreateSelect(IsTrue(fad_get_cond(a64.flags)), on_true, on_false));
        break;
    }
    case farmdec::A64_FMOV_VEC2GPR: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        SetGp(a64.rd, w32, GetScalar(a64.rn, prec, /*fp=*/false));
        break;
    }
    case farmdec::A64_FMOV_GPR2VEC: {
        SetScalar(a64.rd, GetGp(a64.rn, w32));
        break;
    }
    case farmdec::A64_FMOV_TOP2GPR: {
        SetGp(a64.rd, w32, GetElem(a64.rn, farmdec::VA_2D, 1));
        break;
    }
    case farmdec::A64_FMOV_GPR2TOP: {
        InsertElem(a64.rd, 1, GetGp(a64.rn, /*w32=*/false));
        break;
    }
    case farmdec::A64_FMOV_REG:
        SetScalar(a64.rd, GetScalar(a64.rn, fad_get_prec(a64.flags)));
        break;
    case farmdec::A64_FMOV_IMM: {
        farmdec::FPSize prec = fad_get_prec(a64.flags);
        SetScalar(a64.rd, llvm::ConstantFP::get(TypeOf(prec), a64.fimm));
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

// Get a scalar value stored in the A64 register Vr. The sizes FSZ_B (byte, Br)
// and FSZ_Q (quad, Qr) return integers and may only be used with SIMD&FP load/stores.
// FSZ_S, FSZ_D return floating-point values if fp is true (default).
//
// Note: The half-precision FSZ_H returns an integer because there is no
// half-precision support in the rest of Rellume right now.
llvm::Value* Lifter::GetScalar(farmdec::Reg r, farmdec::FPSize fsz, bool fp) {
    Facet fc = (fp) ? Facet::F64 : Facet::I64;

    switch (fsz) {
    case farmdec::FSZ_B: fc = Facet::I8;   break;
    case farmdec::FSZ_H: fc = Facet::I16;  break; // XXX
    case farmdec::FSZ_S: fc = (fp) ? Facet::F32 : Facet::I32; break;
    case farmdec::FSZ_D: fc = (fp) ? Facet::F64 : Facet::I64; break;
    case farmdec::FSZ_Q: fc = Facet::I128; break;
    default:
        assert(false && "invalid FP facet");
    }

    return GetReg(ArchReg::VEC(r), fc);
}

// Set an A64 vector register Vr to a scalar value.
void Lifter::SetScalar(farmdec::Reg r, llvm::Value* val) {
    auto elemty = val->getType();
    Facet fc = Facet::FromType(elemty);
    unsigned bits = elemty->getPrimitiveSizeInBits();

    // Loosely based on the x86 lifter's OpStoreVec: we need to insert
    // (val : elemty) into a vector (nelem x elemty) that spans the
    // entire V register (→ IVEC).
    Facet ivec = Facet::V2I64;
    auto ivecty = ivec.Type(irb.getContext());

    // Does val fill the entire (128-bit) V register?
    if (bits == ivec.Size()) {
        SetReg(ArchReg::VEC(r), ivec, irb.CreateBitCast(val, ivecty));
        SetRegFacet(ArchReg::VEC(r), fc, val);
        return;
    }

    unsigned nelem = ivec.Size() / bits;
    auto vecty = llvm::VectorType::get(elemty, nelem, false);

    llvm::Value* fullvec = llvm::Constant::getNullValue(vecty);
    fullvec = irb.CreateInsertElement(fullvec, val, 0uL);

    SetReg(ArchReg::VEC(r), ivec, irb.CreateBitCast(fullvec, ivecty));
    SetRegFacet(ArchReg::VEC(r), fc, val);
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

// Round the floating-point value v to an itegral floating-point value, according
// to the given rounding mode. If exact is true, FPR_CURRENT may raise an Inexact
// Exception. Whether an Inexact Exception is raised in the other cases is undefined
// behaviour
llvm::Value* Lifter::Round(llvm::Value* v, farmdec::FPRounding mode, bool exact) {
    switch (mode) {
    case farmdec::FPR_CURRENT:
        if (exact) {
            return irb.CreateUnaryIntrinsic(llvm::Intrinsic::rint, v);
        } else {
            return irb.CreateUnaryIntrinsic(llvm::Intrinsic::nearbyint, v);
        }
    case farmdec::FPR_TIE_EVEN: return irb.CreateUnaryIntrinsic(llvm::Intrinsic::roundeven, v);
    case farmdec::FPR_TIE_AWAY: return irb.CreateUnaryIntrinsic(llvm::Intrinsic::round, v);
    case farmdec::FPR_NEG_INF:  return irb.CreateUnaryIntrinsic(llvm::Intrinsic::floor, v);
    case farmdec::FPR_ZERO:     return irb.CreateUnaryIntrinsic(llvm::Intrinsic::trunc, v);
    case farmdec::FPR_POS_INF:  return irb.CreateUnaryIntrinsic(llvm::Intrinsic::ceil, v);
    case farmdec::FPR_ODD:      assert(false && "round to odd not supported");
    }
    assert(false && "invalid rounding mode");
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
    case farmdec::FSZ_H: return irb.getInt16Ty(); // XXX
    case farmdec::FSZ_S: return irb.getFloatTy();
    case farmdec::FSZ_D: return irb.getDoubleTy();
    case farmdec::FSZ_Q: return irb.getIntNTy(128);
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
// The value v must be of type i32 or i64.
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

// Given base, lhs, rhs : iN, calculate (base ± (lhs*rhs)) : i(N*2). This never overflows.
// The inputs are sign- or zero-extended before the operation.
llvm::Value* Lifter::MulAddSub(llvm::Value* base, llvm::Instruction::BinaryOps addsub, llvm::Value* lhs, llvm::Value* rhs, llvm::Instruction::CastOps extend) {
    unsigned n = llvm::cast<llvm::IntegerType>(lhs->getType())->getBitWidth();
    auto long_ty = irb.getIntNTy(n*2);

    auto long_base = irb.CreateCast(extend, base, long_ty);
    auto long_lhs =  irb.CreateCast(extend, lhs, long_ty);
    auto long_rhs =  irb.CreateCast(extend, rhs, long_ty);

    auto long_prod = irb.CreateMul(long_lhs, long_rhs);
    return irb.CreateBinOp(addsub, long_base, long_prod);
}

// Lift an instruction Rd := Rn op X or, if invert_rhs is true, Rd := Rn op X, where X is an optionally
// inverted Shifted(Rm), Extended(Rm) or an Immediate. Most parameters are to avoid needless recalculations.
void Lifter::LiftBinOp(farmdec::Inst a64, bool w32, llvm::Instruction::BinaryOps op, BinOpKind kind, bool set_flags, bool invert_rhs) {
    auto lhs = GetGp(a64.rn, w32);
    llvm::Value* rhs = nullptr;
    switch (kind) {
    case BinOpKind::SHIFT: rhs = Shift(GetGp(a64.rm, w32), static_cast<farmdec::Shift>(a64.shift.type), a64.shift.amount); break;
    case BinOpKind::EXT: rhs = Extend(GetGp(a64.rm, w32), w32, static_cast<farmdec::ExtendType>(a64.extend.type), a64.extend.lsl); break;
    case BinOpKind::IMM: rhs = irb.getIntN((w32) ? 32 : 64, a64.imm); break;
    }
    if (invert_rhs) {
        rhs = irb.CreateNot(rhs);
    }
    auto val = irb.CreateBinOp(op, lhs, rhs);
    SetGp(a64.rd, w32, val);
    if (set_flags) {
        switch (op) {
        case llvm::Instruction::Add: FlagCalcAdd(val, lhs, rhs); break;
        case llvm::Instruction::Sub: FlagCalcSub(val, lhs, rhs); break;
        case llvm::Instruction::And: FlagCalcLogic(val); break;
        default:
            assert(false && "bad instruction for setting flags");
        }
    }
}

// If the condition holds, compare the values; otherwise set the flags according to nzcv.
void Lifter::LiftCCmp(llvm::Value* lhs, llvm::Value* rhs, farmdec::Cond cond, uint8_t nzcv, bool ccmn, bool fp) {
    auto cond_holds = IsTrue(cond);
    if (fp) {
        FlagCalcFP(lhs, rhs);
    } else if (ccmn) {
        FlagCalcAdd(irb.CreateAdd(lhs, rhs), lhs, rhs);
    } else {
        FlagCalcSub(irb.CreateSub(lhs, rhs), lhs, rhs);
    }
    SetFlag(Facet::SF, irb.CreateSelect(cond_holds, GetFlag(Facet::SF), irb.getInt1((nzcv & 8) != 0))); // N
    SetFlag(Facet::ZF, irb.CreateSelect(cond_holds, GetFlag(Facet::ZF), irb.getInt1((nzcv & 4) != 0))); // Z
    SetFlag(Facet::CF, irb.CreateSelect(cond_holds, GetFlag(Facet::CF), irb.getInt1((nzcv & 2) != 0))); // C
    SetFlag(Facet::OF, irb.CreateSelect(cond_holds, GetFlag(Facet::OF), irb.getInt1((nzcv & 1) != 0))); // V
}

// Given a pointer ptr = *T, load a T value, extend it according to ext and put it in rt.
void Lifter::Load(farmdec::Reg rt, bool w32, llvm::Type* srcty,
                  llvm::Value* ptr, farmdec::ExtendType ext,
                  farmdec::MemOrdering mo) {
    llvm::LoadInst* load = irb.CreateLoad(srcty, ptr);
    if (mo != farmdec::MO_NONE) {
        load->setOrdering(Ordering(mo));
        load->setAlignment(llvm::Align(srcty->getPrimitiveSizeInBits() / 8));
    }

    SetGp(rt, w32, Extend(load, w32, ext, 0));
}

// Loads into the SIMD&FP register Vt.
void Lifter::Load(farmdec::Reg rt, llvm::Type* srcty, llvm::Value* ptr, farmdec::MemOrdering mo) {
    llvm::LoadInst* load = irb.CreateLoad(srcty, ptr);
    if (mo != farmdec::MO_NONE) {
        load->setOrdering(Ordering(mo));
        load->setAlignment(llvm::Align(srcty->getPrimitiveSizeInBits() / 8));
    }

    SetScalar(rt, load);
}

// Given a pointer ptr = *T, store the value val, which is truncated appropriately.
void Lifter::Store(llvm::Value* ptr, llvm::Value* val, farmdec::MemOrdering mo) {
    llvm::StoreInst* store = irb.CreateStore(val, ptr);
    if (mo != farmdec::MO_NONE) {
        store->setOrdering(Ordering(mo));
        store->setAlignment(llvm::Align(val->getType()->getPrimitiveSizeInBits() / 8));
    }
}

// (The w32 flag is passed for convenience and is ignored if fp=true.)
void Lifter::LiftLoadStore(farmdec::Inst a64, bool w32, bool fp) {
    farmdec::AddrMode mode = fad_get_addrmode(a64.flags);
    farmdec::ExtendType ext = fad_get_mem_extend(a64.flags); // General load/stores
    farmdec::FPSize fsz = fad_get_prec(a64.flags);           // FP load/stores
    farmdec::MemOrdering mo = farmdec::MO_NONE;
    if (mode == farmdec::AM_SIMPLE) { // AM_SIMPLE → LDAR, LDLAR, STLR, STLLR, ...
        mo = static_cast<farmdec::MemOrdering>(a64.ldst_order.load);

        // If AM_SIMPLE, some memory ordering _must_ be set. Use this fact to
        // avoid checking whether the instruction loads or stores.
        if (mo == farmdec::MO_NONE) {
            mo = static_cast<farmdec::MemOrdering>(a64.ldst_order.store);
        }
    }

    // General: The type of the value to load/store depends on the in-memory
    // size encoded in the lower two bits of ext.
    auto memty = (fp) ? TypeOf(fsz) : TypeOf(static_cast<farmdec::Size>(ext&3));
    auto ptr = Addr(memty, a64);

    switch (a64.op) {
    default:
        assert(false && "not a load/store");
        break;

    case farmdec::A64_LDP:
    case farmdec::A64_LDXP: // TODO: exclusive access
        Load(a64.rt2, w32, memty, irb.CreateConstGEP1_64(memty, ptr, 1), ext, mo);
        /* fallthrough */
    case farmdec::A64_LDR:
    case farmdec::A64_LDXR: // TODO: exclusive access
        Load(a64.rt, w32, memty, ptr, ext, mo);
        break;

    case farmdec::A64_STP:
    case farmdec::A64_STXP: // TODO: exclusive access
        Store(irb.CreateConstGEP1_64(memty, ptr, 1), irb.CreateTruncOrBitCast(GetGp(a64.rt2, w32), memty), mo);
        /* fallthrough */
    case farmdec::A64_STR:
    case farmdec::A64_STXR: // TODO: exclusive access
        Store(ptr, irb.CreateTruncOrBitCast(GetGp(a64.rt, w32), memty), mo);
        break;

    case farmdec::A64_LDP_FP:
        Load(a64.rt2, memty, irb.CreateConstGEP1_64(memty, ptr, 1), mo);
        /* fallthrough */
    case farmdec::A64_LDR_FP:
        Load(a64.rt, memty, ptr, mo);
        break;

    case farmdec::A64_STP_FP:
        Store(irb.CreateConstGEP1_64(memty, ptr, 1), GetScalar(a64.rt2, fsz), mo);
        /* fallthrough */
    case farmdec::A64_STR_FP:
        Store(ptr, GetScalar(a64.rt, fsz), mo);
        break;
    }
}

// Essentially, FCMP as defined by the ARM manual's pseudocode function FPCompare.
void Lifter::FlagCalcFP(llvm::Value* lhs, llvm::Value* rhs) {
    auto is_unordered = irb.CreateFCmpUNO(lhs, rhs);
    auto is_equal = irb.CreateFCmpOEQ(lhs, rhs);
    auto is_less = irb.CreateFCmpOLT(lhs, rhs);
    SetFlag(Facet::SF, is_less);
    SetFlag(Facet::ZF, is_equal);
    SetFlag(Facet::CF, irb.CreateNot(is_less));
    SetFlag(Facet::OF, is_unordered);
}

void Lifter::LiftBinOpFP(llvm::Instruction::BinaryOps op, farmdec::FPSize prec, farmdec::Reg rd, farmdec::Reg rn, farmdec::Reg rm) {
    auto lhs = GetScalar(rn, prec);
    auto rhs = GetScalar(rm, prec);
    SetScalar(rd, irb.CreateBinOp(op, lhs, rhs));
}

void Lifter::LiftIntrinsicFP(llvm::Intrinsic::ID op, farmdec::FPSize prec, farmdec::Reg rd, farmdec::Reg rn) {
    auto val = GetScalar(rn, prec);
    SetScalar(rd, irb.CreateUnaryIntrinsic(op, val));
}

void Lifter::LiftIntrinsicFP(llvm::Intrinsic::ID op, farmdec::FPSize prec, farmdec::Reg rd, farmdec::Reg rn, farmdec::Reg rm) {
    auto lhs = GetScalar(rn, prec);
    auto rhs = GetScalar(rm, prec);
    SetScalar(rd, irb.CreateBinaryIntrinsic(op, lhs, rhs));
}

void Lifter::LiftIntrinsicFP(llvm::Intrinsic::ID op, farmdec::FPSize prec, farmdec::Reg rd, farmdec::Reg rn, farmdec::Reg rm, farmdec::Reg ra) {
    auto lhs = GetScalar(rn, prec);
    auto rhs = GetScalar(rm, prec);
    auto add = GetScalar(ra, prec);
    auto mod = irb.GetInsertBlock()->getModule();
    auto fn = llvm::Intrinsic::getDeclaration(mod, op, {lhs->getType()});
    SetScalar(rd, irb.CreateCall(fn, {lhs, rhs, add}));
}
} // namespace rellume::aarch64

/**
 * @}
 **/
