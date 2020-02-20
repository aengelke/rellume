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

#include "lifter-private.h"

#include "facet.h"
#include "instr.h"
#include "regfile.h"
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>


/**
 * \defgroup InstructionGP General Purpose Instructions
 * \ingroup Instruction
 *
 * @{
 **/

namespace rellume {

static llvm::Instruction* WrapNoFold(llvm::Value* v) {
    return llvm::CastInst::Create(llvm::Instruction::BitCast, v, v->getType());
}

bool Lifter::Lift(const Instr& inst) {
    // Set new instruction pointer register
    SetReg(X86Reg::IP, Facet::I64, irb.getInt64(inst.end()));

    // Add separator for debugging.
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    irb.CreateCall(llvm::Intrinsic::getDeclaration(module,
                                                   llvm::Intrinsic::donothing,
                                                   {}));

    // Check overridden implementations first.
    const auto& override = cfg.instr_overrides.find(inst.type());
    if (override != cfg.instr_overrides.end()) {
        if (inst.type() == FDI_SYSCALL) {
            SetReg(X86Reg::RCX, Facet::I64, GetReg(X86Reg::IP, Facet::I64));
            SetReg(X86Reg::GP(11), Facet::I64, FlagAsReg(64));
        }

        auto fn_type = llvm::FunctionType::get(irb.getVoidTy(), {fi.sptr_raw->getType()}, false);

        // Pack all state into the CPU struct.
        CallConv sptr_conv = CallConv::SPTR;
        sptr_conv.Pack(*regfile, fi);
        llvm::CallInst* call = irb.CreateCall(fn_type, override->second, {fi.sptr_raw});
        regfile->Clear(); // Clear all facets before importing register state
        sptr_conv.Unpack(*regfile, fi);

        // Directly inline alwaysinline functions
        if (override->second->hasFnAttribute(llvm::Attribute::AlwaysInline)) {
            llvm::InlineFunctionInfo ifi;
            llvm::InlineFunction(llvm::CallSite(call), ifi);
        }

        return true;
    }

    switch (inst.type()) {
    default:
        SetReg(X86Reg::IP, Facet::I64, irb.Insert(WrapNoFold(irb.getInt64(inst.start()))));
        return false;

    case FDI_NOP: /* do nothing */ break;
    case FDI_RDSSP: /* do nothing */ break;
    // Intel MPX, behave as NOP on processors without support (SDM Vol 1, 17.4)
    case FDI_BNDLDX: /* do nothing */ break;
    case FDI_BNDMOV: /* do nothing */ break;
    case FDI_BNDCU: /* do nothing */ break;
    case FDI_BNDCL: /* do nothing */ break;
    case FDI_BNDSTX: /* do nothing */ break;
    case FDI_BNDCN: /* do nothing */ break;
    case FDI_BNDMK: /* do nothing */ break;

    case FDI_PUSH: LiftPush(inst); break;
    case FDI_PUSHF: LiftPushf(inst); break;
    case FDI_POPF: LiftPopf(inst); break;
    case FDI_POP: LiftPop(inst); break;
    case FDI_LEAVE: LiftLeave(inst); break;
    case FDI_CALL: LiftCall(inst); break;
    case FDI_RET: LiftRet(inst); break;
    // case FDI_SYSCALL: NOT IMPLEMENTED
    // case FDI_CPUID: NOT IMPLEMENTED
    // case FDI_RDTSC: NOT IMPLEMENTED
    // case FDI_CRC32: NOT IMPLEMENTED
    case FDI_UD2: LiftUnreachable(inst); break;

    case FDI_LAHF: OpStoreGp(X86Reg::RAX, Facet::I8H, FlagAsReg(8)); break;
    case FDI_SAHF: FlagFromReg(GetReg(X86Reg::RAX, Facet::I8H)); break;

    case FDI_MOV: LiftMovgp(inst, llvm::Instruction::SExt); break;
    case FDI_MOVABS: LiftMovgp(inst, llvm::Instruction::SExt); break;
    case FDI_MOVZX: LiftMovgp(inst, llvm::Instruction::ZExt); break;
    case FDI_MOVSX: LiftMovgp(inst, llvm::Instruction::SExt); break;
    // TODO: set non-temporal hint
    case FDI_MOVNTI: LiftMovgp(inst, llvm::Instruction::SExt); break;
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
    case FDI_AND: LiftAndOrXor(inst, llvm::Instruction::And); break;
    case FDI_OR: LiftAndOrXor(inst, llvm::Instruction::Or); break;
    case FDI_XOR: LiftAndOrXor(inst, llvm::Instruction::Xor); break;
    case FDI_TEST: LiftAndOrXor(inst, llvm::Instruction::And, /*wb=*/false); break;
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
    case FDI_BT: LiftBittest(inst); break;
    case FDI_BTC: LiftBittest(inst); break;
    case FDI_BTR: LiftBittest(inst); break;
    case FDI_BTS: LiftBittest(inst); break;
    case FDI_BSWAP: LiftBswap(inst); break;
    case FDI_C_EX: LiftCext(inst); break;
    case FDI_C_SEP: LiftCsep(inst); break;

    case FDI_CLC: SetFlag(Facet::CF, irb.getFalse()); break;
    case FDI_STC: SetFlag(Facet::CF, irb.getTrue()); break;
    case FDI_CMC: SetFlag(Facet::CF, irb.CreateNot(GetFlag(Facet::CF))); break;

    case FDI_CLD: SetFlag(Facet::DF, irb.getFalse()); break;
    case FDI_STD: SetFlag(Facet::DF, irb.getTrue()); break;
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
    case FDI_PREFETCHT0: LiftPrefetch(inst, 0, 3); break;
    case FDI_PREFETCHT1: LiftPrefetch(inst, 0, 2); break;
    case FDI_PREFETCHT2: LiftPrefetch(inst, 0, 1); break;
    case FDI_PREFETCHNTA: LiftPrefetch(inst, 0, 0); break;
    case FDI_PREFETCHW: LiftPrefetch(inst, 1, 1); break;
    case FDI_FXSAVE: LiftFxsave(inst); break;
    case FDI_FXRSTOR: LiftFxrstor(inst); break;
    case FDI_FSTCW: LiftFstcw(inst); break;
    // case FDI_FLDCW: TODO break;
    case FDI_FSTSW: LiftFstsw(inst); break;
    case FDI_STMXCSR: LiftStmxcsr(inst); break;
    // case FDI_LDMXCSR: TODO break;
    case FDI_SSE_MOVD: LiftSseMovq(inst, Facet::I32); break;
    case FDI_SSE_MOVQ: LiftSseMovq(inst, Facet::I64); break;
    case FDI_SSE_MOVSS: LiftSseMovScalar(inst, Facet::F32); break;
    case FDI_SSE_MOVSD: LiftSseMovScalar(inst, Facet::F64); break;
    case FDI_SSE_MOVUPS: LiftSseMovdq(inst, Facet::V4F32, ALIGN_NONE); break;
    case FDI_SSE_MOVUPD: LiftSseMovdq(inst, Facet::V2F64, ALIGN_NONE); break;
    case FDI_SSE_MOVAPS: LiftSseMovdq(inst, Facet::V4F32, ALIGN_MAX); break;
    case FDI_SSE_MOVAPD: LiftSseMovdq(inst, Facet::V2F64, ALIGN_MAX); break;
    case FDI_SSE_MOVDQU: LiftSseMovdq(inst, Facet::I128, ALIGN_NONE); break;
    case FDI_SSE_MOVDQA: LiftSseMovdq(inst, Facet::I128, ALIGN_MAX); break;
    case FDI_SSE_MOVNTPS: LiftSseMovntStore(inst, Facet::VF32); break;
    case FDI_SSE_MOVNTPD: LiftSseMovntStore(inst, Facet::VF64); break;
    case FDI_SSE_MOVNTDQ: LiftSseMovntStore(inst, Facet::VI64); break;
    // TODO: set non-temporal hint
    case FDI_SSE_MOVNTDQA: LiftSseMovdq(inst, Facet::I128, ALIGN_MAX); break;
    case FDI_SSE_MOVLPS: LiftSseMovlp(inst); break;
    case FDI_SSE_MOVLPD: LiftSseMovlp(inst); break;
    case FDI_SSE_MOVHPS: LiftSseMovhps(inst); break;
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
    case FDI_SSE_ADDSS: LiftSseBinOp(inst, llvm::Instruction::FAdd, Facet::F32); break;
    case FDI_SSE_ADDSD: LiftSseBinOp(inst, llvm::Instruction::FAdd, Facet::F64); break;
    case FDI_SSE_ADDPS: LiftSseBinOp(inst, llvm::Instruction::FAdd, Facet::VF32); break;
    case FDI_SSE_ADDPD: LiftSseBinOp(inst, llvm::Instruction::FAdd, Facet::VF64); break;
    case FDI_SSE_SUBSS: LiftSseBinOp(inst, llvm::Instruction::FSub, Facet::F32); break;
    case FDI_SSE_SUBSD: LiftSseBinOp(inst, llvm::Instruction::FSub, Facet::F64); break;
    case FDI_SSE_SUBPS: LiftSseBinOp(inst, llvm::Instruction::FSub, Facet::VF32); break;
    case FDI_SSE_SUBPD: LiftSseBinOp(inst, llvm::Instruction::FSub, Facet::VF64); break;
    case FDI_SSE_MULSS: LiftSseBinOp(inst, llvm::Instruction::FMul, Facet::F32); break;
    case FDI_SSE_MULSD: LiftSseBinOp(inst, llvm::Instruction::FMul, Facet::F64); break;
    case FDI_SSE_MULPS: LiftSseBinOp(inst, llvm::Instruction::FMul, Facet::VF32); break;
    case FDI_SSE_MULPD: LiftSseBinOp(inst, llvm::Instruction::FMul, Facet::VF64); break;
    case FDI_SSE_DIVSS: LiftSseBinOp(inst, llvm::Instruction::FDiv, Facet::F32); break;
    case FDI_SSE_DIVSD: LiftSseBinOp(inst, llvm::Instruction::FDiv, Facet::F64); break;
    case FDI_SSE_DIVPS: LiftSseBinOp(inst, llvm::Instruction::FDiv, Facet::VF32); break;
    case FDI_SSE_DIVPD: LiftSseBinOp(inst, llvm::Instruction::FDiv, Facet::VF64); break;
    case FDI_SSE_MINSS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::F32); break;
    case FDI_SSE_MINSD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::F64); break;
    case FDI_SSE_MINPS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::VF32); break;
    case FDI_SSE_MINPD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OLT, Facet::VF64); break;
    case FDI_SSE_MAXSS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::F32); break;
    case FDI_SSE_MAXSD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::F64); break;
    case FDI_SSE_MAXPS: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::VF32); break;
    case FDI_SSE_MAXPD: LiftSseMinmax(inst, llvm::CmpInst::FCMP_OGT, Facet::VF64); break;
    case FDI_SSE_ORPS: LiftSseBinOp(inst, llvm::Instruction::Or, Facet::VI32); break;
    case FDI_SSE_ORPD: LiftSseBinOp(inst, llvm::Instruction::Or, Facet::VI64); break;
    case FDI_SSE_ANDPS: LiftSseBinOp(inst, llvm::Instruction::And, Facet::VI32); break;
    case FDI_SSE_ANDPD: LiftSseBinOp(inst, llvm::Instruction::And, Facet::VI64); break;
    case FDI_SSE_XORPS: LiftSseBinOp(inst, llvm::Instruction::Xor, Facet::VI32); break;
    case FDI_SSE_XORPD: LiftSseBinOp(inst, llvm::Instruction::Xor, Facet::VI64); break;
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

    case FDI_SSE_PXOR: LiftSseBinOp(inst, llvm::Instruction::Xor, Facet::VI64); break;
    case FDI_SSE_POR: LiftSseBinOp(inst, llvm::Instruction::Or, Facet::VI64); break;
    case FDI_SSE_PAND: LiftSseBinOp(inst, llvm::Instruction::And, Facet::VI64); break;
    case FDI_SSE_PANDN: LiftSseAndn(inst, Facet::VI64); break;
    case FDI_SSE_PADDB: LiftSseBinOp(inst, llvm::Instruction::Add, Facet::V16I8); break;
    case FDI_SSE_PADDW: LiftSseBinOp(inst, llvm::Instruction::Add, Facet::V8I16); break;
    case FDI_SSE_PADDD: LiftSseBinOp(inst, llvm::Instruction::Add, Facet::V4I32); break;
    case FDI_SSE_PADDQ: LiftSseBinOp(inst, llvm::Instruction::Add, Facet::V2I64); break;
    case FDI_SSE_PSUBB: LiftSseBinOp(inst, llvm::Instruction::Sub, Facet::V16I8); break;
    case FDI_SSE_PSUBW: LiftSseBinOp(inst, llvm::Instruction::Sub, Facet::V8I16); break;
    case FDI_SSE_PSUBD: LiftSseBinOp(inst, llvm::Instruction::Sub, Facet::V4I32); break;
    case FDI_SSE_PSUBQ: LiftSseBinOp(inst, llvm::Instruction::Sub, Facet::V2I64); break;
    case FDI_SSE_PADDSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/true, Facet::V16I8); break;
    case FDI_SSE_PADDSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/true, Facet::V8I16); break;
    case FDI_SSE_PADDUSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/false, Facet::V16I8); break;
    case FDI_SSE_PADDUSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Add, /*sign=*/false, Facet::V8I16); break;
    case FDI_SSE_PSUBSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/true, Facet::V16I8); break;
    case FDI_SSE_PSUBSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/true, Facet::V8I16); break;
    case FDI_SSE_PSUBUSB: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/false, Facet::V16I8); break;
    case FDI_SSE_PSUBUSW: LiftSsePaddsubSaturate(inst, llvm::Instruction::Sub, /*sign=*/false, Facet::V8I16); break;
    case FDI_SSE_PMULLW: LiftSseBinOp(inst, llvm::Instruction::Mul, Facet::V8I16); break;
    case FDI_SSE_PMULLD: LiftSseBinOp(inst, llvm::Instruction::Mul, Facet::V4I32); break;
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
    case FDI_SSE_PMOVMSKB: LiftSseMovmsk(inst, Facet::VI8); break;
    case FDI_SSE_MOVMSKPS: LiftSseMovmsk(inst, Facet::VI32); break;
    case FDI_SSE_MOVMSKPD: LiftSseMovmsk(inst, Facet::VI64); break;

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

void Lifter::LiftMovgp(const Instr& inst, llvm::Instruction::CastOps cast) {
    // TODO: if the instruction moves the whole register, keep all facets.
    // TODO: implement this for all register-register moves.

    llvm::Value* val = OpLoad(inst.op(1), Facet::I);
    llvm::Type* tgt_ty = irb.getIntNTy(inst.op(1).bits());
    OpStoreGp(inst.op(0), irb.CreateCast(cast, val, tgt_ty));
}

// Implementation of ADD, ADC, SUB, SBB, CMP, and XADD
void Lifter::LiftArith(const Instr& inst, bool sub) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    if (inst.type() == FDI_ADC || inst.type() == FDI_SBB)
        op2 = irb.CreateAdd(op2, irb.CreateZExt(GetFlag(Facet::CF), op2->getType()));

    auto arith_op = sub ? llvm::Instruction::Sub : llvm::Instruction::Add;
    llvm::Value* res = irb.CreateBinOp(arith_op, op1, op2);

    if (inst.type() != FDI_CMP)
        OpStoreGp(inst.op(0), res);
    if (inst.type() == FDI_XADD)
        OpStoreGp(inst.op(1), op1);

    if (sub)
        FlagCalcSub(res, op1, op2);
    else
        FlagCalcAdd(res, op1, op2);
}

void Lifter::LiftCmpxchg(const Instr& inst) {
    auto acc = GetReg(X86Reg::RAX, Facet::In(inst.op(0).bits()));
    auto dst = OpLoad(inst.op(0), Facet::I);
    auto src = OpLoad(inst.op(1), Facet::I);

    // Full compare with acc and dst
    llvm::Value* cmp_res = irb.CreateSub(acc, dst);
    FlagCalcSub(cmp_res, acc, dst);

    // Store SRC if DST=ACC, else store DST again (i.e. don't change memory).
    OpStoreGp(inst.op(0), irb.CreateSelect(GetFlag(Facet::ZF), src, dst));
    // ACC gets the value from memory.
    OpStoreGp(X86Reg::RAX, dst);
}

void Lifter::LiftXchg(const Instr& inst) {
    // TODO: atomic memory operation
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    OpStoreGp(inst.op(0), op2);
    OpStoreGp(inst.op(1), op1);
}

void Lifter::LiftAndOrXor(const Instr& inst, llvm::Instruction::BinaryOps op,
                           bool writeback) {
    llvm::Value* res = irb.CreateBinOp(op, OpLoad(inst.op(0), Facet::I),
                                       OpLoad(inst.op(1), Facet::I));
    if (writeback)
        OpStoreGp(inst.op(0), res);

    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlagUndef({Facet::AF});
    SetFlag(Facet::CF, irb.getFalse());
    SetFlag(Facet::OF, irb.getFalse());
}

void Lifter::LiftNot(const Instr& inst) {
    OpStoreGp(inst.op(0), irb.CreateNot(OpLoad(inst.op(0), Facet::I)));
}

void Lifter::LiftNeg(const Instr& inst) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* res = irb.CreateNeg(op1);
    llvm::Value* zero = llvm::Constant::getNullValue(res->getType());
    FlagCalcSub(res, zero, op1);
    OpStoreGp(inst.op(0), res);
}

void Lifter::LiftIncDec(const Instr& inst) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = irb.getIntN(inst.op(0).bits(), 1);
    llvm::Value* res = nullptr;
    if (inst.type() == FDI_INC) {
        res = irb.CreateAdd(op1, op2);
        FlagCalcOAdd(res, op1, op2);
    } else if (inst.type() == FDI_DEC) {
        res = irb.CreateSub(op1, op2);
        FlagCalcOSub(res, op1, op2);
    }

    // Carry flag is _not_ updated.
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    FlagCalcA(res, op1, op2);
    OpStoreGp(inst.op(0), res);
}

void Lifter::LiftShift(const Instr& inst, llvm::Instruction::BinaryOps op) {
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    llvm::Value* src_ex;
    if (inst.op(0).size() >= 4)
        src_ex = src;
    else if (inst.type() == FDI_SAR)
        src_ex = irb.CreateSExt(src, irb.getInt32Ty());
    else // inst.op(0).size() < 4 && (SHR || SHL)
        src_ex = irb.CreateZExt(src, irb.getInt32Ty());

    assert(inst.op(1) && "shift without second operand");
    llvm::Value* shift = OpLoad(inst.op(1), Facet::I);

    unsigned mask = inst.op(0).size() == 8 ? 0x3f : 0x1f;
    shift = irb.CreateAnd(irb.CreateZExt(shift, src_ex->getType()), mask);

    llvm::Value* res_ex = irb.CreateBinOp(op, src_ex, shift);
    llvm::Value* res = irb.CreateTrunc(res_ex, src->getType());
    OpStoreGp(inst.op(0), res);

    // CF is the last bit shifted out
    llvm::Value* cf_big;
    if (inst.type() == FDI_SHL) {
        unsigned sz = inst.op(0).bits();
        llvm::Value* max = llvm::ConstantInt::get(src_ex->getType(), sz);
        cf_big = irb.CreateLShr(src_ex, irb.CreateSub(max, shift));
    } else { // SHR/SAR
        llvm::Value* one = llvm::ConstantInt::get(src_ex->getType(), 1);
        cf_big = irb.CreateLShr(src_ex, irb.CreateSub(shift, one));
    }

    // TODO: flags are only affected if shift != 0
    FlagCalcS(res);
    FlagCalcZ(res);
    FlagCalcP(res);
    SetFlag(Facet::CF, irb.CreateTrunc(cf_big, irb.getInt1Ty()));
    llvm::Value* zero = llvm::ConstantInt::get(src->getType(), 0);
    SetFlag(Facet::OF, irb.CreateICmpSLT(irb.CreateXor(src, res), zero));
    SetFlagUndef({Facet::AF});
}

void Lifter::LiftShiftdouble(const Instr& inst) {
    llvm::Value* src1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* src2 = OpLoad(inst.op(1), Facet::I);
    llvm::Type* ty = src1->getType();
    llvm::Value* res;

    assert(inst.op(2) && "shld/shrd without third operand");
    unsigned mask = inst.op(0).size() == 8 ? 0x3f : 0x1f;
    llvm::Value* shift = irb.CreateZExt(OpLoad(inst.op(2), Facet::I), ty);
    // TODO: support small shifts with amount > len
    // LLVM sets the result to poison if this occurs.
    shift = irb.CreateAnd(shift, mask);

    auto id = inst.type() == FDI_SHLD ? llvm::Intrinsic::fshl
                                       : llvm::Intrinsic::fshr;
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    if (inst.type() == FDI_SHLD)
        res = irb.CreateCall(intrinsic, {src1, src2, shift});
    else if (inst.type() == FDI_SHRD)
        res = irb.CreateCall(intrinsic, {src2, src1, shift});
    else
        assert(false && "invalid double-shift operation");
    OpStoreGp(inst.op(0), res);

    // TODO: calculate flags correctly
    FlagCalcZ(res);
    FlagCalcS(res);
    FlagCalcP(res);
    SetFlagUndef({Facet::OF, Facet::AF, Facet::CF});
}

void Lifter::LiftRotate(const Instr& inst) {
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    llvm::Type* ty = src->getType();

    assert(inst.op(1) && "rotate without second operand");

    unsigned mask = inst.op(0).size() == 8 ? 0x3f : 0x1f;
    llvm::Value* shift = irb.CreateZExt(OpLoad(inst.op(1), Facet::I), ty);
    // TODO: support small shifts with amount > len
    // LLVM sets the result to poison if this occurs.
    shift = irb.CreateAnd(shift, mask);

    auto id = inst.type() == FDI_ROL ? llvm::Intrinsic::fshl
                                      : llvm::Intrinsic::fshr;
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    llvm::Value* res = irb.CreateCall(intrinsic, {src, src, shift});
    OpStoreGp(inst.op(0), res);

    // SF, ZF, AF, and PF are unaffected.
    // TODO: calculate flags correctly
    // CF is affected only if count > 0
    // OF is affected only if count > 0, but undefined if count > 1
    SetFlagUndef({Facet::OF, Facet::CF});
}

void Lifter::LiftMul(const Instr& inst) {
    unsigned sz = inst.op(0).bits();

    unsigned op_cnt = inst.op(2) ? 3 : inst.op(1) ? 2 : 1;

    llvm::Value* op1;
    llvm::Value* op2;
    if (op_cnt == 1) {
        op1 = OpLoad(inst.op(0), Facet::I);
        op2 = GetReg(X86Reg::RAX, Facet::In(sz));
    } else {
        op1 = OpLoad(inst.op(op_cnt - 2), Facet::I);
        op2 = OpLoad(inst.op(op_cnt - 1), Facet::I);
    }

    // Perform "normal" multiplication
    llvm::Value* short_res = irb.CreateMul(op1, op2);

    // Extend operand values and perform extended multiplication
    auto cast_op = inst.type() == FDI_IMUL ? llvm::Instruction::SExt
                                            : llvm::Instruction::ZExt;
    llvm::Type* double_ty = irb.getIntNTy(sz * 2);
    llvm::Value* ext_op1 = irb.CreateCast(cast_op, op1, double_ty);
    llvm::Value* ext_op2 = irb.CreateCast(cast_op, op2, double_ty);
    llvm::Value* ext_res = irb.CreateMul(ext_op1, ext_op2);

    if (op_cnt == 1) {
        if (sz == 8) {
            OpStoreGp(X86Reg::RAX, ext_res);
        } else {
            // Don't use short_res to avoid having two multiplications.
            // TODO: is this concern still valid?
            llvm::Type* value_ty = irb.getIntNTy(sz);
            llvm::Value* res_a = irb.CreateTrunc(ext_res, value_ty);
            llvm::Value* high = irb.CreateLShr(ext_res, sz);
            llvm::Value* res_d = irb.CreateTrunc(high, value_ty);
            OpStoreGp(X86Reg::RAX, res_a);
            OpStoreGp(X86Reg::RDX, res_d);
        }
    } else {
        OpStoreGp(inst.op(0), short_res);
    }

    llvm::Value* overflow;
    if (cfg.enableOverflowIntrinsics) {
        llvm::Intrinsic::ID id = llvm::Intrinsic::smul_with_overflow;
        llvm::Value* packed = irb.CreateBinaryIntrinsic(id, op1, op2);
        overflow = irb.CreateExtractValue(packed, 1);
    } else {
        llvm::Value* ext_short_res = irb.CreateCast(cast_op, short_res, double_ty);
        overflow = irb.CreateICmpNE(ext_res, ext_short_res);
    }

    SetFlag(Facet::OF, overflow);
    SetFlag(Facet::CF, overflow);
    SetFlagUndef({Facet::SF, Facet::ZF, Facet::AF, Facet::PF});
}

void Lifter::LiftDiv(const Instr& inst) {
    unsigned sz = inst.op(0).bits();
    bool sign = inst.type() == FDI_IDIV;
    auto ext_op = sign ? llvm::Instruction::SExt : llvm::Instruction::ZExt;
    auto div_op = sign ? llvm::Instruction::SDiv : llvm::Instruction::UDiv;
    auto rem_op = sign ? llvm::Instruction::SRem : llvm::Instruction::URem;

    // TODO: raise #DE on division by zero or overflow.

    auto ex_ty = irb.getIntNTy(sz * 2);

    llvm::Value* dividend;
    if (sz == 8) {
        // Dividend is AX
        dividend = GetReg(X86Reg::RAX, Facet::I16);
    } else {
        // Dividend is DX:AX/EDX:EAX/RDX:RAX
        auto low = GetReg(X86Reg::RAX, Facet::In(sz));
        auto high = GetReg(X86Reg::RDX, Facet::In(sz));
        high = irb.CreateShl(irb.CreateZExt(high, ex_ty), sz);
        dividend = irb.CreateOr(irb.CreateZExt(low, ex_ty), high);
    }

    // Divisor is the operand
    auto divisor = irb.CreateCast(ext_op, OpLoad(inst.op(0), Facet::I), ex_ty);

    auto quot = irb.CreateBinOp(div_op, dividend, divisor);
    auto rem = irb.CreateBinOp(rem_op, dividend, divisor);

    auto val_ty = irb.getIntNTy(sz);
    quot = irb.CreateTrunc(quot, val_ty);
    rem = irb.CreateTrunc(rem, val_ty);

    if (sz == 8) {
        // Quotient is AL, remainder is AH
        OpStoreGp(X86Reg::RAX, Facet::I8, quot);
        OpStoreGp(X86Reg::RAX, Facet::I8H, rem);
    } else {
        // Quotient is AX/EAX/RAX, remainer is DX/EDX/RDX
        OpStoreGp(X86Reg::RAX, quot);
        OpStoreGp(X86Reg::RDX, rem);
    }

    SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                  Facet::CF});
}

void
Lifter::LiftLea(const Instr& inst)
{
    assert(inst.op(0).is_reg());
    assert(inst.op(1).is_mem());

    // Compute pointer before we overwrite any registers, but ignore segment.
    llvm::Value* res_ptr = OpAddr(inst.op(1), irb.getInt8Ty(), FD_REG_DS);

    // Compute as integer
    unsigned addrsz = inst.op(1).addrsz() * 8;
    Facet facet = Facet{Facet::I}.Resolve(addrsz);
    llvm::Value* res = irb.getIntN(addrsz, inst.op(1).off());
    if (inst.op(1).base())
        res = irb.CreateAdd(res, GetReg(MapReg(inst.op(1).base()), facet));
    if (inst.op(1).scale() != 0) {
        llvm::Value* offset = GetReg(MapReg(inst.op(1).index()), facet);
        offset = irb.CreateMul(offset, irb.getIntN(addrsz, inst.op(1).scale()));
        res = irb.CreateAdd(res, offset);
    }

    llvm::Type* op_type = irb.getIntNTy(inst.op(0).bits());
    OpStoreGp(inst.op(0), irb.CreateZExtOrTrunc(res, op_type));

    if (cfg.use_gep_ptr_arithmetic && inst.op(0).size() == 8)
        SetRegFacet(MapReg(inst.op(0).reg()), Facet::PTR, res_ptr);
}

void Lifter::LiftXlat(const Instr& inst) {
    llvm::Value* al = GetReg(X86Reg::RAX, Facet::I8);
    llvm::Value* bx;
    if (inst.addrsz() == 8) {
        bx = GetReg(X86Reg::RBX, Facet::PTR);
        bx = irb.CreatePointerCast(bx, irb.getInt8PtrTy());
    } else {
        bx = GetReg(X86Reg::RBX, Facet::I32);
        bx = irb.CreateZExt(bx, irb.getInt64Ty());
        bx = irb.CreateIntToPtr(bx, irb.getInt8PtrTy());
    }

    llvm::Value* ptr = irb.CreateGEP(bx, irb.CreateZExt(al, irb.getInt32Ty()));
    OpStoreGp(X86Reg::RAX, irb.CreateLoad(irb.getInt8Ty(), ptr));
}

void Lifter::LiftCmovcc(const Instr& inst, Condition cond) {
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::I);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::I);
    // Note that 32-bit registers are still zero-extended when cond is false.
    OpStoreGp(inst.op(0), irb.CreateSelect(FlagCond(cond), op2, op1));
}

void Lifter::LiftSetcc(const Instr& inst, Condition cond) {
    OpStoreGp(inst.op(0), irb.CreateZExt(FlagCond(cond), irb.getInt8Ty()));
}

void Lifter::LiftCext(const Instr& inst) {
    unsigned sz = inst.opsz() * 8;
    llvm::Value* ax = GetReg(X86Reg::RAX, Facet::In(sz / 2));
    OpStoreGp(X86Reg::RAX, irb.CreateSExt(ax, irb.getIntNTy(sz)));
}

void Lifter::LiftCsep(const Instr& inst) {
    unsigned sz = inst.opsz() * 8;
    llvm::Value* ax = GetReg(X86Reg::RAX, Facet::In(sz));
    OpStoreGp(X86Reg::RDX, irb.CreateAShr(ax, sz - 1));
}

void Lifter::LiftBitscan(const Instr& inst, bool trailing) {
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    auto id = trailing ? llvm::Intrinsic::cttz : llvm::Intrinsic::ctlz;
    llvm::Value* res = irb.CreateBinaryIntrinsic(id, src,
                                                 /*zero_undef=*/irb.getTrue());
    if (!trailing) {
        unsigned sz = inst.op(1).bits();
        res = irb.CreateSub(irb.getIntN(sz, sz - 1), res);
    }
    OpStoreGp(inst.op(0), res);

    FlagCalcZ(src);
    SetFlagUndef({Facet::OF, Facet::SF, Facet::AF, Facet::PF, Facet::CF});
}

void Lifter::LiftBittest(const Instr& inst) {
    llvm::Value* index = OpLoad(inst.op(1), Facet::I);
    unsigned op_size = inst.op(0).bits();
    assert((op_size == 16 || op_size == 32 || op_size == 64) &&
            "invalid bittest operation size");

    llvm::Value* val;
    llvm::Value* addr = nullptr;
    if (inst.op(0).is_reg()) {
        val = OpLoad(inst.op(0), Facet::I);
    } else { // LL_OP_MEM
        addr = OpAddr(inst.op(0), irb.getIntNTy(op_size), inst.op(0).seg());
        // Immediate operands are truncated, register operands are sign-extended
        if (inst.op(1).is_reg()) {
            llvm::Value* off = irb.CreateAShr(index, __builtin_ctz(op_size));
            addr = irb.CreateGEP(addr, irb.CreateSExt(off, irb.getInt64Ty()));
        }
        val = irb.CreateLoad(addr);
    }

    // Truncated here because memory operand may need full value.
    index = irb.CreateAnd(index, irb.getIntN(op_size, op_size-1));
    llvm::Value* mask = irb.CreateShl(irb.getIntN(op_size, 1), index);

    llvm::Value* bit = irb.CreateAnd(val, mask);

    if (inst.type() == FDI_BT) {
        goto skip_writeback;
    } else if (inst.type() == FDI_BTC) {
        val = irb.CreateXor(val, mask);
    } else if (inst.type() == FDI_BTR) {
        val = irb.CreateAnd(val, irb.CreateNot(mask));
    } else if (inst.type() == FDI_BTS) {
        val = irb.CreateOr(val, mask);
    }

    if (inst.op(0).is_reg())
        OpStoreGp(inst.op(0), val);
    else // LL_OP_MEM
        irb.CreateStore(val, addr);

skip_writeback:
    // Zero flag is not modified
    SetFlag(Facet::CF, irb.CreateICmpNE(bit, irb.getIntN(op_size, 0)));
    SetFlagUndef({Facet::OF, Facet::SF, Facet::AF, Facet::PF});
}

void Lifter::LiftMovbe(const Instr& inst) {
    llvm::Value* src = OpLoad(inst.op(1), Facet::I);
    OpStoreGp(inst.op(0), CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftBswap(const Instr& inst) {
    assert(inst.op(0).is_reg() && "bswap with non-reg operand");
    llvm::Value* src = OpLoad(inst.op(0), Facet::I);
    OpStoreGp(inst.op(0), CreateUnaryIntrinsic(llvm::Intrinsic::bswap, src));
}

void Lifter::LiftJmp(const Instr& inst) {
    // Force default data segment, 3e is notrack.
    SetReg(X86Reg::IP, Facet::I64, OpLoad(inst.op(0), Facet::I64, ALIGN_NONE,
                                          FD_REG_DS));
}

void Lifter::LiftJcc(const Instr& inst, Condition cond) {
    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(FlagCond(cond),
        OpLoad(inst.op(0), Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftJcxz(const Instr& inst) {
    unsigned sz = inst.addrsz();
    llvm::Value* cx = GetReg(X86Reg::RCX, Facet::In(sz * 8));
    llvm::Value* cond = irb.CreateICmpEQ(cx, irb.getIntN(sz*8, 0));
    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(cond,
        OpLoad(inst.op(0), Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftLoop(const Instr& inst) {
    unsigned sz = inst.addrsz();

    // Decrement RCX/ECX
    llvm::Value* cx = GetReg(X86Reg::RCX, Facet::In(sz * 8));
    cx = irb.CreateSub(cx, irb.getIntN(sz * 8, 1));
    OpStoreGp(X86Reg::RCX, cx);

    // Construct condition
    llvm::Value* cond = irb.CreateICmpNE(cx, irb.getIntN(sz*8, 0));
    if (inst.type() == FDI_LOOPZ)
        cond = irb.CreateAnd(cond, GetFlag(Facet::ZF));
    else if (inst.type() == FDI_LOOPNZ)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(Facet::ZF)));

    SetReg(X86Reg::IP, Facet::I64, irb.CreateSelect(cond,
        OpLoad(inst.op(0), Facet::I64),
        GetReg(X86Reg::IP, Facet::I64)
    ));
}

void Lifter::LiftCall(const Instr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                      Facet::CF});

    // Force default data segment, 3e is notrack.
    llvm::Value* new_rip = OpLoad(inst.op(0), Facet::I, ALIGN_NONE, FD_REG_DS);
    StackPush(GetReg(X86Reg::IP, Facet::I64));
    SetReg(X86Reg::IP, Facet::I64, new_rip);
}

void Lifter::LiftRet(const Instr& inst) {
    if (cfg.call_ret_clobber_flags)
        SetFlagUndef({Facet::OF, Facet::SF, Facet::ZF, Facet::AF, Facet::PF,
                      Facet::CF});

    SetReg(X86Reg::IP, Facet::I64, StackPop());
}

void Lifter::LiftUnreachable(const Instr& inst) {
    irb.CreateUnreachable();
    SetReg(X86Reg::IP, Facet::I64, llvm::UndefValue::get(irb.getInt64Ty()));
}

LifterBase::RepInfo LifterBase::RepBegin(const Instr& inst) {
    RepInfo info = {};
    bool di = inst.type() != FDI_LODS;
    bool si = inst.type() != FDI_STOS && inst.type() != FDI_SCAS;

    bool condrep = inst.type() == FDI_SCAS || inst.type() == FDI_CMPS;
    if (inst.has_rep())
        info.mode = condrep ? RepInfo::REPZ : RepInfo::REP;
    else if (inst.has_repnz() && condrep)
        info.mode = RepInfo::REPNZ;
    else
        info.mode = RepInfo::NO_REP;

    // Iff instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode != RepInfo::NO_REP) {
        info.loop_block = ablock.AddBlock();
        info.cont_block = ablock.AddBlock();

        llvm::Value* count = GetReg(X86Reg::RCX, Facet::I64);
        llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
        llvm::Value* enter_loop = irb.CreateICmpNE(count, zero);
        ablock.GetInsertBlock()->BranchTo(enter_loop, *info.loop_block,
                                          *info.cont_block);

        SetInsertBlock(info.loop_block);
    }

    llvm::Type* op_ty = irb.getIntNTy(inst.opsz() * 8);
    if (di) {
        llvm::Value* ptr = GetReg(X86Reg::RDI, Facet::PTR);
        info.di = irb.CreatePointerCast(ptr, op_ty->getPointerTo());
    }
    if (si) {
        llvm::Value* ptr = GetReg(X86Reg::RSI, Facet::PTR);
        info.si = irb.CreatePointerCast(ptr, op_ty->getPointerTo());
    }

    return info;
}

void LifterBase::RepEnd(RepInfo info) {
    // First update pointer registers with direction flag
    llvm::Value* df = GetFlag(Facet::DF);
    llvm::Value* adj = irb.CreateSelect(df, irb.getInt64(-1), irb.getInt64(1));

    std::pair<int, llvm::Value*> ptr_regs[] = {
        {FD_REG_DI, info.di}, {FD_REG_SI, info.si}
    };

    for (auto reg : ptr_regs) {
        if (reg.second == nullptr)
            continue;

        llvm::Value* ptr = irb.CreateGEP(reg.second, adj);
        llvm::Value* int_val = irb.CreatePtrToInt(ptr, irb.getInt64Ty());
        SetReg(X86Reg::GP(reg.first), Facet::I64, int_val);
        SetRegFacet(X86Reg::GP(reg.first), Facet::PTR, ptr);
    }

    // If instruction has REP/REPZ/REPNZ, add branching logic
    if (info.mode == RepInfo::NO_REP)
        return;

    // Decrement count and check.
    llvm::Value* count = GetReg(X86Reg::GP(FD_REG_CX), Facet::I64);
    count = irb.CreateSub(count, irb.getInt64(1));
    SetReg(X86Reg::GP(FD_REG_CX), Facet::I64, count);

    llvm::Value* zero = llvm::Constant::getNullValue(count->getType());
    llvm::Value* cond = irb.CreateICmpNE(count, zero);
    if (info.mode == RepInfo::REPZ)
        cond = irb.CreateAnd(cond, GetFlag(Facet::ZF));
    else if (info.mode == RepInfo::REPNZ)
        cond = irb.CreateAnd(cond, irb.CreateNot(GetFlag(Facet::ZF)));

    ablock.GetInsertBlock()->BranchTo(cond, *info.loop_block, *info.cont_block);
    SetInsertBlock(info.cont_block);
}

void Lifter::LiftLods(const Instr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    unsigned size = inst.opsz();
    OpStoreGp(X86Reg::RAX, irb.CreateLoad(irb.getIntNTy(size), rep_info.di));

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftStos(const Instr& inst) {
    // TODO: optimize REP STOSB and other sizes with constant zero to llvm
    // memset intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    auto ax = GetReg(X86Reg::RAX, Facet::In(inst.opsz() * 8));
    irb.CreateStore(ax, rep_info.di);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftMovs(const Instr& inst) {
    // TODO: optimize REP MOVSB to use llvm memcpy intrinsic.
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    irb.CreateStore(irb.CreateLoad(rep_info.si), rep_info.di);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftScas(const Instr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    auto src = GetReg(X86Reg::RAX, Facet::In(inst.opsz() * 8));
    llvm::Value* dst = irb.CreateLoad(rep_info.di);
    // Perform a normal CMP operation.
    FlagCalcSub(irb.CreateSub(src, dst), src, dst);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

void Lifter::LiftCmps(const Instr& inst) {
    RepInfo rep_info = RepBegin(inst); // NOTE: this modifies control flow!

    llvm::Value* src = irb.CreateLoad(rep_info.si);
    llvm::Value* dst = irb.CreateLoad(rep_info.di);
    // Perform a normal CMP operation.
    FlagCalcSub(irb.CreateSub(src, dst), src, dst);

    RepEnd(rep_info); // NOTE: this modifies control flow!
}

} // namespace

/**
 * @}
 **/
