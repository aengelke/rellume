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

#include "x86-64/lifter.h"
#include "x86-64/lifter-private.h"

#include "arch.h"
#include "facet.h"
#include "instr.h"
#include "regfile.h"

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>

/**
 * @{
 **/

namespace rellume::x86_64 {

bool LiftInstruction(const Instr& inst, FunctionInfo& fi, const LLConfig& cfg,
                     ArchBasicBlock& ab) noexcept {
    return Lifter(fi, cfg, ab).Lift(inst);
}

bool Lifter::Lift(const Instr& inst) {
    // Set new instruction pointer register
    SetIP(inst.end());

    // Add instruction marker
    if (cfg.instr_marker) {
        llvm::StringRef str_ref{reinterpret_cast<const char*>(&inst),
                                sizeof(FdInstr)};
        llvm::MDString* md = llvm::MDString::get(irb.getContext(), str_ref);
        llvm::Value* md_val = llvm::MetadataAsValue::get(irb.getContext(), md);
        irb.CreateCall(cfg.instr_marker, {AddrIPRel(), md_val});
    }

    // Check overridden implementations first.
    const auto& override = cfg.instr_overrides.find(inst.type());
    if (override != cfg.instr_overrides.end()) {
        CallExternalFunction(override->second);
        return true;
    }

    switch (inst.type()) {
    default:
        SetIP(inst.start(), /*nofold=*/true);
        return false;

    case FDI_NOP: /* do nothing */ break;
    case FDI_RDSSP: /* do nothing */ break;
    case FDI_ENDBR64: /* do nothing */ break;
#if 0
    // Fadec doesn't support MPX.
    // Intel MPX, behave as NOP on processors without support (SDM Vol 1, 17.4)
    case FDI_BNDLDX: /* do nothing */ break;
    case FDI_BNDMOV: /* do nothing */ break;
    case FDI_BNDCU: /* do nothing */ break;
    case FDI_BNDCL: /* do nothing */ break;
    case FDI_BNDSTX: /* do nothing */ break;
    case FDI_BNDCN: /* do nothing */ break;
    case FDI_BNDMK: /* do nothing */ break;
#endif

    case FDI_PUSH: LiftPush(inst); break;
    case FDI_PUSHF: LiftPushf(inst); break;
    case FDI_POPF: LiftPopf(inst); break;
    case FDI_POP: LiftPop(inst); break;
    case FDI_LEAVE: LiftLeave(inst); break;
    case FDI_CALL: LiftCall(inst); break;
    case FDI_RET: LiftRet(inst); break;
    case FDI_SYSCALL: LiftSyscall(inst); break;
    case FDI_CPUID: LiftCpuid(inst); break;
    case FDI_RDTSC: LiftRdtsc(inst); break;
    // case FDI_CRC32: NOT IMPLEMENTED
    // case FDI_UD2: Intentionally not implemented.

    case FDI_LAHF: StoreGpFacet(ArchReg::RAX, Facet::I8H, FlagAsReg(8)); break;
    case FDI_SAHF: FlagFromReg(GetReg(ArchReg::RAX, Facet::I8H)); break;

    case FDI_MOV: LiftMovgp(inst); break;
    case FDI_MOVABS: LiftMovgp(inst); break;
    case FDI_MOVZX: LiftMovzx(inst); break;
    case FDI_MOVSX: LiftMovgp(inst); break;
    // TODO: set non-temporal hint
    case FDI_MOVNTI: LiftMovgp(inst); break;
    case FDI_MOVBE: LiftMovbe(inst); break;
    case FDI_ADD: LiftArith(inst, /*sub=*/false); break;
    case FDI_ADC: LiftArith(inst, /*sub=*/false); break;
    case FDI_SUB: LiftArith(inst, /*sub=*/true); break;
    case FDI_SBB: LiftArith(inst, /*sub=*/true); break;
    case FDI_CMP: LiftArith(inst, /*sub=*/true); break;
    case FDI_XADD: LiftArith(inst, /*sub=*/false); break;
    case FDI_CMPXCHG: LiftCmpxchg(inst); break;
    case FDI_XCHG: LiftXchg(inst); break;
    case FDI_LEA: LiftLea(inst); break;
    case FDI_XLATB: LiftXlat(inst); break;
    case FDI_NOT: LiftNot(inst); break;
    case FDI_NEG: LiftNeg(inst); break;
    case FDI_INC: LiftIncDec(inst); break;
    case FDI_DEC: LiftIncDec(inst); break;
    case FDI_AND: LiftAndOrXor(inst, llvm::Instruction::And, llvm::AtomicRMWInst::And); break;
    case FDI_OR: LiftAndOrXor(inst, llvm::Instruction::Or, llvm::AtomicRMWInst::Or); break;
    case FDI_XOR: LiftAndOrXor(inst, llvm::Instruction::Xor, llvm::AtomicRMWInst::Xor); break;
    case FDI_TEST: LiftAndOrXor(inst, llvm::Instruction::And, llvm::AtomicRMWInst::And, /*wb=*/false); break;
    case FDI_IMUL: LiftMul(inst); break;
    case FDI_MUL: LiftMul(inst); break;
    case FDI_IDIV: LiftDiv(inst); break;
    case FDI_DIV: LiftDiv(inst); break;
    case FDI_SHL: LiftShift(inst, llvm::Instruction::Shl); break;
    case FDI_SHR: LiftShift(inst, llvm::Instruction::LShr); break;
    case FDI_SAR: LiftShift(inst, llvm::Instruction::AShr); break;
    case FDI_ROL: LiftRotate(inst); break;
    case FDI_ROR: LiftRotate(inst); break;
    case FDI_SHLD: LiftShiftdouble(inst); break;
    case FDI_SHRD: LiftShiftdouble(inst); break;
    case FDI_BSF: LiftBitscan(inst, /*trailing=*/true); break;
    case FDI_TZCNT: LiftBitscan(inst, /*trailing=*/true); break; // TODO: support TZCNT
    case FDI_BSR: LiftBitscan(inst, /*trailing=*/false); break;
    case FDI_LZCNT: LiftBitscan(inst, /*trailing=*/false); break; // TODO: support LZCNT
    case FDI_POPCNT: LiftPopcnt(inst); break;
    case FDI_BT: LiftBittest(inst, llvm::Instruction::Or, llvm::AtomicRMWInst::Or); break;
    case FDI_BTC: LiftBittest(inst, llvm::Instruction::Xor, llvm::AtomicRMWInst::Xor); break;
    case FDI_BTR: LiftBittest(inst, llvm::Instruction::And, llvm::AtomicRMWInst::And); break;
    case FDI_BTS: LiftBittest(inst, llvm::Instruction::Or, llvm::AtomicRMWInst::Or); break;
    case FDI_BSWAP: LiftBswap(inst); break;
    case FDI_C_EX: LiftCext(inst); break;
    case FDI_C_SEP: LiftCsep(inst); break;

    case FDI_CLC: SetReg(ArchReg::CF, irb.getFalse()); break;
    case FDI_STC: SetReg(ArchReg::CF, irb.getTrue()); break;
    case FDI_CMC: SetReg(ArchReg::CF, irb.CreateNot(GetFlag(ArchReg::CF))); break;

    case FDI_CLD: SetReg(ArchReg::DF, irb.getFalse()); break;
    case FDI_STD: SetReg(ArchReg::DF, irb.getTrue()); break;
    case FDI_LODS: LiftLods(inst); break;
    case FDI_STOS: LiftStos(inst); break;
    case FDI_MOVS: LiftMovs(inst); break;
    case FDI_SCAS: LiftScas(inst); break;
    case FDI_CMPS: LiftCmps(inst); break;

    case FDI_CMOVO: LiftCmovcc(inst, Condition::O); break;
    case FDI_CMOVNO: LiftCmovcc(inst, Condition::NO); break;
    case FDI_CMOVC: LiftCmovcc(inst, Condition::C); break;
    case FDI_CMOVNC: LiftCmovcc(inst, Condition::NC); break;
    case FDI_CMOVZ: LiftCmovcc(inst, Condition::Z); break;
    case FDI_CMOVNZ: LiftCmovcc(inst, Condition::NZ); break;
    case FDI_CMOVBE: LiftCmovcc(inst, Condition::BE); break;
    case FDI_CMOVA: LiftCmovcc(inst, Condition::A); break;
    case FDI_CMOVS: LiftCmovcc(inst, Condition::S); break;
    case FDI_CMOVNS: LiftCmovcc(inst, Condition::NS); break;
    case FDI_CMOVP: LiftCmovcc(inst, Condition::P); break;
    case FDI_CMOVNP: LiftCmovcc(inst, Condition::NP); break;
    case FDI_CMOVL: LiftCmovcc(inst, Condition::L); break;
    case FDI_CMOVGE: LiftCmovcc(inst, Condition::GE); break;
    case FDI_CMOVLE: LiftCmovcc(inst, Condition::LE); break;
    case FDI_CMOVG: LiftCmovcc(inst, Condition::G); break;

    case FDI_SETO: LiftSetcc(inst, Condition::O); break;
    case FDI_SETNO: LiftSetcc(inst, Condition::NO); break;
    case FDI_SETC: LiftSetcc(inst, Condition::C); break;
    case FDI_SETNC: LiftSetcc(inst, Condition::NC); break;
    case FDI_SETZ: LiftSetcc(inst, Condition::Z); break;
    case FDI_SETNZ: LiftSetcc(inst, Condition::NZ); break;
    case FDI_SETBE: LiftSetcc(inst, Condition::BE); break;
    case FDI_SETA: LiftSetcc(inst, Condition::A); break;
    case FDI_SETS: LiftSetcc(inst, Condition::S); break;
    case FDI_SETNS: LiftSetcc(inst, Condition::NS); break;
    case FDI_SETP: LiftSetcc(inst, Condition::P); break;
    case FDI_SETNP: LiftSetcc(inst, Condition::NP); break;
    case FDI_SETL: LiftSetcc(inst, Condition::L); break;
    case FDI_SETGE: LiftSetcc(inst, Condition::GE); break;
    case FDI_SETLE: LiftSetcc(inst, Condition::LE); break;
    case FDI_SETG: LiftSetcc(inst, Condition::G); break;

    // Defined in llinstruction-sse.c
    case FDI_LFENCE: LiftFence(inst); break;
    case FDI_SFENCE: LiftFence(inst); break;
    case FDI_MFENCE: LiftFence(inst); break;
    case FDI_PREFETCH: LiftPrefetch(inst, 0, 3); break;
    case FDI_RESERVED_PREFETCH: LiftPrefetch(inst, 0, 3); break;
    case FDI_PREFETCHT0: LiftPrefetch(inst, 0, 3); break;
    case FDI_PREFETCHT1: LiftPrefetch(inst, 0, 2); break;
    case FDI_PREFETCHT2: LiftPrefetch(inst, 0, 1); break;
    case FDI_PREFETCHNTA: LiftPrefetch(inst, 0, 0); break;
    case FDI_PREFETCHW: LiftPrefetch(inst, 1, 1); break;
    case FDI_PREFETCHWT1: LiftPrefetch(inst, 1, 2); break;
    case FDI_FXSAVE: LiftFxsave(inst); break;
    case FDI_FXRSTOR: LiftFxrstor(inst); break;
    case FDI_FSTCW: LiftFstcw(inst); break;
    case FDI_FLDCW: break; // intentional NOPs. TODO: implement semantics
    case FDI_FSTSW: LiftFstsw(inst); break;
    case FDI_STMXCSR: LiftStmxcsr(inst); break;
    case FDI_LDMXCSR: break; // intentional NOPs. TODO: implement semantics
    case FDI_SSE_MOVD: LiftSseMovq(inst, Facet::I32); break;
    case FDI_SSE_MOVQ: LiftSseMovq(inst, Facet::I64); break;
    // Note: We load I32/I64 instead of F32/F64. Bit-casting a double to a
    // pointer triggers some bug in CodeGen, leading to incorrect compiled code.
    case FDI_SSE_MOVSS: LiftSseMovScalar(inst, Facet::I32); break;
    case FDI_SSE_MOVSD: LiftSseMovScalar(inst, Facet::I64); break;
    case FDI_SSE_MOVUPS: LiftSseMovdq(inst, Facet::V4F32, ALIGN_NONE); break;
    case FDI_SSE_MOVUPD: LiftSseMovdq(inst, Facet::V2F64, ALIGN_NONE); break;
    case FDI_SSE_MOVAPS: LiftSseMovdq(inst, Facet::V4F32, ALIGN_MAX); break;
    case FDI_SSE_MOVAPD: LiftSseMovdq(inst, Facet::V2F64, ALIGN_MAX); break;
    case FDI_SSE_MOVDQU: LiftSseMovdq(inst, Facet::V2I64, ALIGN_NONE); break;
    case FDI_SSE_MOVDQA: LiftSseMovdq(inst, Facet::V2I64, ALIGN_MAX); break;
    case FDI_SSE_LDDQU: LiftSseMovdq(inst, Facet::V2I64, ALIGN_NONE); break;
    case FDI_SSE_MOVNTPS: LiftSseMovntStore(inst, Facet::VF32); break;
    case FDI_SSE_MOVNTPD: LiftSseMovntStore(inst, Facet::VF64); break;
    case FDI_SSE_MOVNTDQ: LiftSseMovntStore(inst, Facet::VI64); break;
    // TODO: set non-temporal hint
    case FDI_SSE_MOVNTDQA: LiftSseMovdq(inst, Facet::V2I64, ALIGN_MAX); break;
    case FDI_SSE_MOVLPS: LiftSseMovlp(inst); break;
    case FDI_SSE_MOVHLPS: LiftSseMovlp(inst); break;
    case FDI_SSE_MOVLPD: LiftSseMovlp(inst); break;
    case FDI_SSE_MOVHPS: LiftSseMovhps(inst); break;
    case FDI_SSE_MOVLHPS: LiftSseMovhps(inst); break;
    case FDI_SSE_MOVHPD: LiftSseMovhpd(inst); break;
    case FDI_SSE_PUNPCKLBW: LiftSseUnpck(inst, Facet::V16I8); break;
    case FDI_SSE_PUNPCKLWD: LiftSseUnpck(inst, Facet::V8I16); break;
    case FDI_SSE_PUNPCKLDQ: LiftSseUnpck(inst, Facet::V4I32); break;
    case FDI_SSE_PUNPCKLQDQ: LiftSseUnpck(inst, Facet::V2I64); break;
    case FDI_SSE_PUNPCKHBW: LiftSseUnpck(inst, Facet::V16I8); break;
    case FDI_SSE_PUNPCKHWD: LiftSseUnpck(inst, Facet::V8I16); break;
    case FDI_SSE_PUNPCKHDQ: LiftSseUnpck(inst, Facet::V4I32); break;
    case FDI_SSE_PUNPCKHQDQ: LiftSseUnpck(inst, Facet::V2I64); break;
    case FDI_SSE_UNPCKLPS: LiftSseUnpck(inst, Facet::V4F32); break;
    case FDI_SSE_UNPCKLPD: LiftSseUnpck(inst, Facet::V2F64); break;
    case FDI_SSE_UNPCKHPS: LiftSseUnpck(inst, Facet::V4F32); break;
    case FDI_SSE_UNPCKHPD: LiftSseUnpck(inst, Facet::V2F64); break;
    case FDI_SSE_SHUFPD: LiftSseShufpd(inst); break;
    case FDI_SSE_SHUFPS: LiftSseShufps(inst); break;
    case FDI_SSE_PSHUFD: LiftSsePshufd(inst); break;
    case FDI_SSE_PSHUFLW: LiftSsePshufw(inst, 0); break;
    case FDI_SSE_PSHUFHW: LiftSsePshufw(inst, 4); break;
    case FDI_SSE_INSERTPS: LiftSseInsertps(inst); break;
    case FDI_SSE_EXTRACTPS: LiftSsePextr(inst, Facet::VF32, 0x03); break;
    case FDI_SSE_MOVSLDUP: LiftSseMovdup(inst, Facet::VF32, 0); break;
    case FDI_SSE_MOVSHDUP: LiftSseMovdup(inst, Facet::VF32, 1); break;
    case FDI_SSE_MOVDDUP: LiftSseMovdup(inst, Facet::VF64, 0); break;
    case FDI_SSE_ADDSS: LiftSseBinOp(inst, llvm::Instruction::FAdd, /*horz=*/false, Facet::F32); break;
    case FDI_SSE_ADDSD: LiftSseBinOp(inst, llvm::Instruction::FAdd, /*horz=*/false, Facet::F64); break;
    case FDI_SSE_ADDPS: LiftSseBinOp(inst, llvm::Instruction::FAdd, /*horz=*/false, Facet::VF32); break;
    case FDI_SSE_ADDPD: LiftSseBinOp(inst, llvm::Instruction::FAdd, /*horz=*/false, Facet::VF64); break;
    case FDI_SSE_ADDSUBPS: LiftSseAddSub(inst, Facet::VF32); break;
    case FDI_SSE_ADDSUBPD: LiftSseAddSub(inst, Facet::VF64); break;
    case FDI_SSE_SUBSS: LiftSseBinOp(inst, llvm::Instruction::FSub, /*horz=*/false, Facet::F32); break;
    case FDI_SSE_SUBSD: LiftSseBinOp(inst, llvm::Instruction::FSub, /*horz=*/false, Facet::F64); break;
    case FDI_SSE_SUBPS: LiftSseBinOp(inst, llvm::Instruction::FSub, /*horz=*/false, Facet::VF32); break;
    case FDI_SSE_SUBPD: LiftSseBinOp(inst, llvm::Instruction::FSub, /*horz=*/false, Facet::VF64); break;
    case FDI_SSE_MULSS: LiftSseBinOp(inst, llvm::Instruction::FMul, /*horz=*/false, Facet::F32); break;
    case FDI_SSE_MULSD: LiftSseBinOp(inst, llvm::Instruction::FMul, /*horz=*/false, Facet::F64); break;
    case FDI_SSE_MULPS: LiftSseBinOp(inst, llvm::Instruction::FMul, /*horz=*/false, Facet::VF32); break;
    case FDI_SSE_MULPD: LiftSseBinOp(inst, llvm::Instruction::FMul, /*horz=*/false, Facet::VF64); break;
    case FDI_SSE_DIVSS: LiftSseBinOp(inst, llvm::Instruction::FDiv, /*horz=*/false, Facet::F32); break;
    case FDI_SSE_DIVSD: LiftSseBinOp(inst, llvm::Instruction::FDiv, /*horz=*/false, Facet::F64); break;
    case FDI_SSE_DIVPS: LiftSseBinOp(inst, llvm::Instruction::FDiv, /*horz=*/false, Facet::VF32); break;
    case FDI_SSE_DIVPD: LiftSseBinOp(inst, llvm::Instruction::FDiv, /*horz=*/false, Facet::VF64); break;
    case FDI_SSE_HADDPS: LiftSseBinOp(inst, llvm::Instruction::FAdd, /*horz=*/true, Facet::VF32); break;
    case FDI_SSE_HADDPD: LiftSseBinOp(inst, llvm::Instruction::FAdd, /*horz=*/true, Facet::VF64); break;
    case FDI_SSE_HSUBPS: LiftSseBinOp(inst, llvm::Instruction::FSub, /*horz=*/true, Facet::VF32); break;
    case FDI_SSE_HSUBPD: LiftSseBinOp(inst, llvm::Instruction::FSub, /*horz=*/true, Facet::VF64); break;
    case FDI_SSE_MINSS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::F32); break;
    case FDI_SSE_MINSD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::F64); break;
    case FDI_SSE_MINPS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::VF32); break;
    case FDI_SSE_MINPD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::VF64); break;
    case FDI_SSE_MAXSS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::F32); break;
    case FDI_SSE_MAXSD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::F64); break;
    case FDI_SSE_MAXPS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::VF32); break;
    case FDI_SSE_MAXPD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::VF64); break;
    case FDI_SSE_ORPS: LiftSseBinOp(inst, llvm::Instruction::Or, /*horz=*/false, Facet::VI32); break;
    case FDI_SSE_ORPD: LiftSseBinOp(inst, llvm::Instruction::Or, /*horz=*/false, Facet::VI64); break;
    case FDI_SSE_ANDPS: LiftSseBinOp(inst, llvm::Instruction::And, /*horz=*/false, Facet::VI32); break;
    case FDI_SSE_ANDPD: LiftSseBinOp(inst, llvm::Instruction::And, /*horz=*/false, Facet::VI64); break;
    case FDI_SSE_XORPS: LiftSseBinOp(inst, llvm::Instruction::Xor, /*horz=*/false, Facet::VI32); break;
    case FDI_SSE_XORPD: LiftSseBinOp(inst, llvm::Instruction::Xor, /*horz=*/false, Facet::VI64); break;
    case FDI_SSE_ANDNPS: LiftSseAndn(inst, Facet::VI32); break;
    case FDI_SSE_ANDNPD: LiftSseAndn(inst, Facet::VI64); break;
    case FDI_SSE_COMISS: LiftSseComis(inst, Facet::F32); break;
    case FDI_SSE_COMISD: LiftSseComis(inst, Facet::F64); break;
    case FDI_SSE_UCOMISS: LiftSseComis(inst, Facet::F32); break;
    case FDI_SSE_UCOMISD: LiftSseComis(inst, Facet::F64); break;
    case FDI_SSE_CMPSS: LiftSseCmp(inst, Facet::F32); break;
    case FDI_SSE_CMPSD: LiftSseCmp(inst, Facet::F64); break;
    case FDI_SSE_CMPPS: LiftSseCmp(inst, Facet::VF32); break;
    case FDI_SSE_CMPPD: LiftSseCmp(inst, Facet::VF64); break;
    case FDI_SSE_SQRTSS: LiftSseSqrt(inst, Facet::F32); break;
    case FDI_SSE_SQRTSD: LiftSseSqrt(inst, Facet::F64); break;
    case FDI_SSE_SQRTPS: LiftSseSqrt(inst, Facet::VF32); break;
    case FDI_SSE_SQRTPD: LiftSseSqrt(inst, Facet::VF64); break;
    case FDI_SSE_CVTDQ2PD: LiftSseCvt(inst, Facet::V2I32, Facet::V2F64); break;
    case FDI_SSE_CVTDQ2PS: LiftSseCvt(inst, Facet::V4I32, Facet::V4F32); break;
    // case FDI_SSE_CVTPD2DQ: TODO // non-truncating, same types as below
    case FDI_SSE_CVTTPD2DQ: LiftSseCvt(inst, Facet::V2F64, Facet::V2I32); break;
    // case FDI_SSE_CVTPS2DQ: TODO // non-truncating, same types as below
    case FDI_SSE_CVTTPS2DQ: LiftSseCvt(inst, Facet::V4F32, Facet::V4I32); break;
    case FDI_SSE_CVTPD2PS: LiftSseCvt(inst, Facet::V2F64, Facet::V2F32); break;
    case FDI_SSE_CVTPS2PD: LiftSseCvt(inst, Facet::V2F32, Facet::V2F64); break;
    case FDI_SSE_CVTSD2SS: LiftSseCvt(inst, Facet::F64, Facet::F32); break;
    case FDI_SSE_CVTSS2SD: LiftSseCvt(inst, Facet::F32, Facet::F64); break;
    // case FDI_SSE_CVTSD2SI: TODO // non-truncating, same types as below
    case FDI_SSE_CVTTSD2SI: LiftSseCvt(inst, Facet::F64, Facet::I); break;
    // case FDI_SSE_CVTSS2SI: TODO // non-truncating, same types as below
    case FDI_SSE_CVTTSS2SI: LiftSseCvt(inst, Facet::F32, Facet::I); break;
    case FDI_SSE_CVTSI2SD: LiftSseCvt(inst, Facet::I, Facet::F64); break;
    case FDI_SSE_CVTSI2SS: LiftSseCvt(inst, Facet::I, Facet::F32); break;

    case FDI_SSE_PXOR: LiftSseBinOp(inst, llvm::Instruction::Xor, /*horz=*/false, Facet::VI64); break;
    case FDI_SSE_POR: LiftSseBinOp(inst, llvm::Instruction::Or, /*horz=*/false, Facet::VI64); break;
    case FDI_SSE_PAND: LiftSseBinOp(inst, llvm::Instruction::And, /*horz=*/false, Facet::VI64); break;
    case FDI_SSE_PANDN: LiftSseAndn(inst, Facet::VI64); break;
    case FDI_SSE_PADDB: LiftSseBinOp(inst, llvm::Instruction::Add, /*horz=*/false, Facet::V16I8); break;
    case FDI_SSE_PADDW: LiftSseBinOp(inst, llvm::Instruction::Add, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PADDD: LiftSseBinOp(inst, llvm::Instruction::Add, /*horz=*/false, Facet::V4I32); break;
    case FDI_SSE_PADDQ: LiftSseBinOp(inst, llvm::Instruction::Add, /*horz=*/false, Facet::V2I64); break;
    case FDI_SSE_PSUBB: LiftSseBinOp(inst, llvm::Instruction::Sub, /*horz=*/false, Facet::V16I8); break;
    case FDI_SSE_PSUBW: LiftSseBinOp(inst, llvm::Instruction::Sub, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PSUBD: LiftSseBinOp(inst, llvm::Instruction::Sub, /*horz=*/false, Facet::V4I32); break;
    case FDI_SSE_PSUBQ: LiftSseBinOp(inst, llvm::Instruction::Sub, /*horz=*/false, Facet::V2I64); break;
    case FDI_SSE_PHADDW: LiftSseBinOp(inst, llvm::Instruction::Add, /*horz=*/true, Facet::VI16); break;
    case FDI_SSE_PHADDD: LiftSseBinOp(inst, llvm::Instruction::Add, /*horz=*/true, Facet::VI32); break;
    case FDI_SSE_PHSUBW: LiftSseBinOp(inst, llvm::Instruction::Sub, /*horz=*/true, Facet::VI16); break;
    case FDI_SSE_PHSUBD: LiftSseBinOp(inst, llvm::Instruction::Sub, /*horz=*/true, Facet::VI32); break;
    case FDI_SSE_PADDSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/true, /*horz=*/false, Facet::V16I8); break;
    case FDI_SSE_PADDSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/true, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PADDUSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/false, /*horz=*/false, Facet::V16I8); break;
    case FDI_SSE_PADDUSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/false, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PSUBSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/true, /*horz=*/false, Facet::V16I8); break;
    case FDI_SSE_PSUBSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/true, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PSUBUSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/false, /*horz=*/false, Facet::V16I8); break;
    case FDI_SSE_PSUBUSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/false, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PHADDSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/true, /*horz=*/true, Facet::V8I16); break;
    case FDI_SSE_PHSUBSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/true, /*horz=*/true, Facet::V8I16); break;
    case FDI_SSE_PSADBW: LiftSsePsadbw(inst); break;
    case FDI_SSE_PMADDUBSW: LiftSsePmaddubsw(inst); break;
    case FDI_SSE_PMULLW: LiftSseBinOp(inst, llvm::Instruction::Mul, /*horz=*/false, Facet::V8I16); break;
    case FDI_SSE_PMULLD: LiftSseBinOp(inst, llvm::Instruction::Mul, /*horz=*/false, Facet::V4I32); break;
    case FDI_SSE_PSLLW: LiftSsePshiftElement(inst, llvm::Instruction::Shl, Facet::VI16); break;
    case FDI_SSE_PSLLD: LiftSsePshiftElement(inst, llvm::Instruction::Shl, Facet::VI32); break;
    case FDI_SSE_PSLLQ: LiftSsePshiftElement(inst, llvm::Instruction::Shl, Facet::VI64); break;
    case FDI_SSE_PSRLW: LiftSsePshiftElement(inst, llvm::Instruction::LShr, Facet::VI16); break;
    case FDI_SSE_PSRLD: LiftSsePshiftElement(inst, llvm::Instruction::LShr, Facet::VI32); break;
    case FDI_SSE_PSRLQ: LiftSsePshiftElement(inst, llvm::Instruction::LShr, Facet::VI64); break;
    case FDI_SSE_PSRAW: LiftSsePshiftElement(inst, llvm::Instruction::AShr, Facet::VI16); break;
    case FDI_SSE_PSRAD: LiftSsePshiftElement(inst, llvm::Instruction::AShr, Facet::VI32); break;
    case FDI_SSE_PSLLDQ: LiftSsePshiftBytes(inst); break;
    case FDI_SSE_PSRLDQ: LiftSsePshiftBytes(inst); break;
    case FDI_SSE_PALIGNR: LiftSsePalignr(inst); break;
    case FDI_SSE_PACKSSWB: LiftSsePack(inst, Facet::VI16, /*sign=*/true); break;
    case FDI_SSE_PACKSSDW: LiftSsePack(inst, Facet::VI32, /*sign=*/true); break;
    case FDI_SSE_PACKUSWB: LiftSsePack(inst, Facet::VI16, /*sign=*/false); break;
    case FDI_SSE_PACKUSDW: LiftSsePack(inst, Facet::VI32, /*sign=*/false); break;
    case FDI_SSE_PINSRB: LiftSsePinsr(inst, Facet::VI8, Facet::I8, 0x0f); break;
    case FDI_SSE_PINSRW: LiftSsePinsr(inst, Facet::VI16, Facet::I16, 0x07); break;
    case FDI_SSE_PINSRD: LiftSsePinsr(inst, Facet::VI32, Facet::I32, 0x03); break;
    case FDI_SSE_PINSRQ: LiftSsePinsr(inst, Facet::VI64, Facet::I64, 0x01); break;
    case FDI_SSE_PEXTRB: LiftSsePextr(inst, Facet::VI8, 0x0f); break;
    case FDI_SSE_PEXTRW: LiftSsePextr(inst, Facet::VI16, 0x07); break;
    case FDI_SSE_PEXTRD: LiftSsePextr(inst, Facet::VI32, 0x03); break;
    case FDI_SSE_PEXTRQ: LiftSsePextr(inst, Facet::VI64, 0x01); break;
    case FDI_SSE_PAVGB: LiftSsePavg(inst, Facet::VI8); break;
    case FDI_SSE_PAVGW: LiftSsePavg(inst, Facet::VI16); break;
    case FDI_SSE_PMULHW: LiftSsePmulhw(inst, llvm::Instruction::SExt); break;
    case FDI_SSE_PMULHUW: LiftSsePmulhw(inst, llvm::Instruction::ZExt); break;
    case FDI_SSE_PMULDQ: LiftSsePmuldq(inst, llvm::Instruction::SExt); break;
    case FDI_SSE_PMULUDQ: LiftSsePmuldq(inst, llvm::Instruction::ZExt); break;
    case FDI_SSE_PMADDWD: LiftSsePmaddwd(inst); break;
    case FDI_SSE_PCMPEQB: LiftSsePcmp(inst, llvm::CmpInst::ICMP_EQ, Facet::VI8); break;
    case FDI_SSE_PCMPEQW: LiftSsePcmp(inst, llvm::CmpInst::ICMP_EQ, Facet::VI16); break;
    case FDI_SSE_PCMPEQD: LiftSsePcmp(inst, llvm::CmpInst::ICMP_EQ, Facet::VI32); break;
    case FDI_SSE_PCMPEQQ: LiftSsePcmp(inst, llvm::CmpInst::ICMP_EQ, Facet::VI64); break;
    case FDI_SSE_PCMPGTB: LiftSsePcmp(inst, llvm::CmpInst::ICMP_SGT, Facet::VI8); break;
    case FDI_SSE_PCMPGTW: LiftSsePcmp(inst, llvm::CmpInst::ICMP_SGT, Facet::VI16); break;
    case FDI_SSE_PCMPGTD: LiftSsePcmp(inst, llvm::CmpInst::ICMP_SGT, Facet::VI32); break;
    case FDI_SSE_PCMPGTQ: LiftSsePcmp(inst, llvm::CmpInst::ICMP_SGT, Facet::VI64); break;
    case FDI_SSE_PMINUB: LiftSsePminmax(inst, llvm::CmpInst::ICMP_ULT, Facet::VI8); break;
    case FDI_SSE_PMINUW: LiftSsePminmax(inst, llvm::CmpInst::ICMP_ULT, Facet::VI16); break;
    case FDI_SSE_PMINUD: LiftSsePminmax(inst, llvm::CmpInst::ICMP_ULT, Facet::VI32); break;
    case FDI_SSE_PMINSB: LiftSsePminmax(inst, llvm::CmpInst::ICMP_SLT, Facet::VI8); break;
    case FDI_SSE_PMINSW: LiftSsePminmax(inst, llvm::CmpInst::ICMP_SLT, Facet::VI16); break;
    case FDI_SSE_PMINSD: LiftSsePminmax(inst, llvm::CmpInst::ICMP_SLT, Facet::VI32); break;
    case FDI_SSE_PMAXUB: LiftSsePminmax(inst, llvm::CmpInst::ICMP_UGT, Facet::VI8); break;
    case FDI_SSE_PMAXUW: LiftSsePminmax(inst, llvm::CmpInst::ICMP_UGT, Facet::VI16); break;
    case FDI_SSE_PMAXUD: LiftSsePminmax(inst, llvm::CmpInst::ICMP_UGT, Facet::VI32); break;
    case FDI_SSE_PMAXSB: LiftSsePminmax(inst, llvm::CmpInst::ICMP_SGT, Facet::VI8); break;
    case FDI_SSE_PMAXSW: LiftSsePminmax(inst, llvm::CmpInst::ICMP_SGT, Facet::VI16); break;
    case FDI_SSE_PMAXSD: LiftSsePminmax(inst, llvm::CmpInst::ICMP_SGT, Facet::VI32); break;
    case FDI_SSE_PABSB: LiftSsePabs(inst, Facet::VI8); break;
    case FDI_SSE_PABSW: LiftSsePabs(inst, Facet::VI16); break;
    case FDI_SSE_PABSD: LiftSsePabs(inst, Facet::VI32); break;
    case FDI_SSE_PSIGNB: LiftSsePsign(inst, Facet::VI8); break;
    case FDI_SSE_PSIGNW: LiftSsePsign(inst, Facet::VI16); break;
    case FDI_SSE_PSIGND: LiftSsePsign(inst, Facet::VI32); break;
    case FDI_SSE_PMOVMSKB: LiftSseMovmsk(inst, Facet::VI8); break;
    case FDI_SSE_MOVMSKPS: LiftSseMovmsk(inst, Facet::VI32); break;
    case FDI_SSE_MOVMSKPD: LiftSseMovmsk(inst, Facet::VI64); break;
    case FDI_SSE_PMOVSXBW: LiftSsePmovx(inst, llvm::Instruction::SExt, Facet::V8I8, Facet::V8I16); break;
    case FDI_SSE_PMOVSXBD: LiftSsePmovx(inst, llvm::Instruction::SExt, Facet::V4I8, Facet::V4I32); break;
    case FDI_SSE_PMOVSXBQ: LiftSsePmovx(inst, llvm::Instruction::SExt, Facet::V2I8, Facet::V2I64); break;
    case FDI_SSE_PMOVSXWD: LiftSsePmovx(inst, llvm::Instruction::SExt, Facet::V4I16, Facet::V4I32); break;
    case FDI_SSE_PMOVSXWQ: LiftSsePmovx(inst, llvm::Instruction::SExt, Facet::V2I16, Facet::V2I64); break;
    case FDI_SSE_PMOVSXDQ: LiftSsePmovx(inst, llvm::Instruction::SExt, Facet::V2I32, Facet::V2I64); break;
    case FDI_SSE_PMOVZXBW: LiftSsePmovx(inst, llvm::Instruction::ZExt, Facet::V8I8, Facet::V8I16); break;
    case FDI_SSE_PMOVZXBD: LiftSsePmovx(inst, llvm::Instruction::ZExt, Facet::V4I8, Facet::V4I32); break;
    case FDI_SSE_PMOVZXBQ: LiftSsePmovx(inst, llvm::Instruction::ZExt, Facet::V2I8, Facet::V2I64); break;
    case FDI_SSE_PMOVZXWD: LiftSsePmovx(inst, llvm::Instruction::ZExt, Facet::V4I16, Facet::V4I32); break;
    case FDI_SSE_PMOVZXWQ: LiftSsePmovx(inst, llvm::Instruction::ZExt, Facet::V2I16, Facet::V2I64); break;
    case FDI_SSE_PMOVZXDQ: LiftSsePmovx(inst, llvm::Instruction::ZExt, Facet::V2I32, Facet::V2I64); break;

    // Jumps are handled in the basic block generation code.
    case FDI_JMP: LiftJmp(inst); break;
    case FDI_JO: LiftJcc(inst, Condition::O); break;
    case FDI_JNO: LiftJcc(inst, Condition::NO); break;
    case FDI_JC: LiftJcc(inst, Condition::C); break;
    case FDI_JNC: LiftJcc(inst, Condition::NC); break;
    case FDI_JZ: LiftJcc(inst, Condition::Z); break;
    case FDI_JNZ: LiftJcc(inst, Condition::NZ); break;
    case FDI_JBE: LiftJcc(inst, Condition::BE); break;
    case FDI_JA: LiftJcc(inst, Condition::A); break;
    case FDI_JS: LiftJcc(inst, Condition::S); break;
    case FDI_JNS: LiftJcc(inst, Condition::NS); break;
    case FDI_JP: LiftJcc(inst, Condition::P); break;
    case FDI_JNP: LiftJcc(inst, Condition::NP); break;
    case FDI_JL: LiftJcc(inst, Condition::L); break;
    case FDI_JGE: LiftJcc(inst, Condition::GE); break;
    case FDI_JLE: LiftJcc(inst, Condition::LE); break;
    case FDI_JG: LiftJcc(inst, Condition::G); break;
    case FDI_JCXZ: LiftJcxz(inst); break;
    case FDI_LOOP: LiftLoop(inst); break;
    case FDI_LOOPZ: LiftLoop(inst); break;
    case FDI_LOOPNZ: LiftLoop(inst); break;
    }

    return true;
}

} // namespace rellume::x86_64

/**
 * @}
 **/
