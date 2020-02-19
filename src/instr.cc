/**
 * This file is part of Rellume.
 *
 * (c) 2019, Alexis Engelke <alexis.engelke@googlemail.com>
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

#include "rellume/instr.h"
#include "instr.h"

#include <fadec.h>
#include <cstdint>
#include <cstdio>
#include <cstring>


/**
 * \defgroup LLInstr Instruction
 * \brief Representation of an instruction
 *
 * @{
 **/

namespace rellume {

Instr::Type Instr::type() const {
    switch (FD_TYPE(fdi())) {
    case FDI_INT3: return LL_INS_Invalid;
    case FDI_NOP: return LL_INS_NOP;
    case FDI_RDSSP: return LL_INS_RDSSP;
    case FDI_BNDLDX: return LL_INS_BNDLDX;
    case FDI_BNDMOV: return LL_INS_BNDMOV;
    case FDI_BNDCU: return LL_INS_BNDCU;
    case FDI_BNDCL: return LL_INS_BNDCL;
    case FDI_BNDSTX: return LL_INS_BNDSTX;
    case FDI_BNDCN: return LL_INS_BNDCN;
    case FDI_BNDMK: return LL_INS_BNDMK;
    case FDI_CALL: return LL_INS_CALL;
    case FDI_CALL_IND: return LL_INS_CALL;
    case FDI_RET: return LL_INS_RET;
    case FDI_SYSCALL: return LL_INS_SYSCALL;
    case FDI_CPUID: return LL_INS_CPUID;
    case FDI_RDTSC: return LL_INS_RDTSC;
    case FDI_UD2: return LL_INS_UD2;
    case FDI_PUSH: return LL_INS_PUSH;
    case FDI_PUSHF: return LL_INS_PUSHFQ;
    case FDI_POP: return LL_INS_POP;
    case FDI_POPF: return LL_INS_POPFQ;
    case FDI_LEAVE: return LL_INS_LEAVE;
    case FDI_LAHF: return LL_INS_LAHF;
    case FDI_SAHF: return LL_INS_SAHF;
    case FDI_MOV: return LL_INS_MOV;
    case FDI_MOV_IMM: return LL_INS_MOV;
    case FDI_MOVABS_IMM: return LL_INS_MOV;
    case FDI_MOVZX: return LL_INS_MOVZX;
    case FDI_MOVSX: return LL_INS_MOVSX;
    case FDI_MOVNTI: return LL_INS_MOVNTI;
    case FDI_ADD: return LL_INS_ADD;
    case FDI_ADD_IMM: return LL_INS_ADD;
    case FDI_ADC: return LL_INS_ADC;
    case FDI_ADC_IMM: return LL_INS_ADC;
    case FDI_XADD: return LL_INS_XADD;
    case FDI_SUB: return LL_INS_SUB;
    case FDI_SUB_IMM: return LL_INS_SUB;
    case FDI_SBB: return LL_INS_SBB;
    case FDI_SBB_IMM: return LL_INS_SBB;
    case FDI_CMP: return LL_INS_CMP;
    case FDI_CMP_IMM: return LL_INS_CMP;
    case FDI_CMPXCHG: return LL_INS_CMPXCHG;
    case FDI_XCHG: return LL_INS_XCHG;
    case FDI_LEA: return LL_INS_LEA;
    case FDI_XLATB: return LL_INS_XLAT;
    case FDI_NOT: return LL_INS_NOT;
    case FDI_NEG: return LL_INS_NEG;
    case FDI_INC: return LL_INS_INC;
    case FDI_DEC: return LL_INS_DEC;
    case FDI_AND: return LL_INS_AND;
    case FDI_AND_IMM: return LL_INS_AND;
    case FDI_OR: return LL_INS_OR;
    case FDI_OR_IMM: return LL_INS_OR;
    case FDI_XOR: return LL_INS_XOR;
    case FDI_XOR_IMM: return LL_INS_XOR;
    case FDI_TEST: return LL_INS_TEST;
    case FDI_TEST_IMM: return LL_INS_TEST;
    case FDI_IMUL: return LL_INS_IMUL;
    case FDI_IMUL2: return LL_INS_IMUL;
    case FDI_IMUL3: return LL_INS_IMUL;
    case FDI_MUL: return LL_INS_MUL;
    case FDI_IDIV: return LL_INS_IDIV;
    case FDI_DIV: return LL_INS_DIV;
    case FDI_SHL_IMM: return LL_INS_SHL;
    case FDI_SHL_CL: return LL_INS_SHL;
    case FDI_SHR_IMM: return LL_INS_SHR;
    case FDI_SHR_CL: return LL_INS_SHR;
    case FDI_SAR_IMM: return LL_INS_SAR;
    case FDI_SAR_CL: return LL_INS_SAR;
    case FDI_ROL_IMM: return LL_INS_ROL;
    case FDI_ROL_CL: return LL_INS_ROL;
    case FDI_ROR_IMM: return LL_INS_ROR;
    case FDI_ROR_CL: return LL_INS_ROR;
    case FDI_SHLD_IMM: return LL_INS_SHLD;
    case FDI_SHLD_CL: return LL_INS_SHLD;
    case FDI_SHRD_IMM: return LL_INS_SHRD;
    case FDI_SHRD_CL: return LL_INS_SHRD;
    case FDI_BSF: return LL_INS_BSF;
    case FDI_TZCNT: return LL_INS_TZCNT;
    case FDI_BSR: return LL_INS_BSR;
    case FDI_LZCNT: return LL_INS_LZCNT;
    case FDI_BT: return LL_INS_BT;
    case FDI_BT_IMM: return LL_INS_BT;
    case FDI_BTC: return LL_INS_BTC;
    case FDI_BTC_IMM: return LL_INS_BTC;
    case FDI_BTR: return LL_INS_BTR;
    case FDI_BTR_IMM: return LL_INS_BTR;
    case FDI_BTS: return LL_INS_BTS;
    case FDI_BTS_IMM: return LL_INS_BTS;
    case FDI_CRC32: return LL_INS_CRC32;
    case FDI_MOVBE: return LL_INS_MOVBE;
    case FDI_BSWAP: return LL_INS_BSWAP;
    case FDI_CLC: return LL_INS_CLC;
    case FDI_STC: return LL_INS_STC;
    case FDI_CMC: return LL_INS_CMC;
    case FDI_CLD: return LL_INS_CLD;
    case FDI_STD: return LL_INS_STD;
    case FDI_LODS: return !FD_HAS_REP(fdi()) ? LL_INS_LODS : LL_INS_REP_LODS;
    case FDI_STOS: return !FD_HAS_REP(fdi()) ? LL_INS_STOS : LL_INS_REP_STOS;
    case FDI_MOVS: return !FD_HAS_REP(fdi()) ? LL_INS_MOVS : LL_INS_REP_MOVS;
    case FDI_SCAS: return !FD_HAS_REP(fdi()) ? (!FD_HAS_REPNZ(fdi()) ? LL_INS_SCAS : LL_INS_REPNZ_SCAS) : LL_INS_REPZ_SCAS;
    case FDI_CMPS: return !FD_HAS_REP(fdi()) ? (!FD_HAS_REPNZ(fdi()) ? LL_INS_CMPS : LL_INS_REPNZ_CMPS) : LL_INS_REPZ_CMPS;
    case FDI_CMOVO: return LL_INS_CMOVO;
    case FDI_CMOVNO: return LL_INS_CMOVNO;
    case FDI_CMOVC: return LL_INS_CMOVC;
    case FDI_CMOVNC: return LL_INS_CMOVNC;
    case FDI_CMOVZ: return LL_INS_CMOVZ;
    case FDI_CMOVNZ: return LL_INS_CMOVNZ;
    case FDI_CMOVBE: return LL_INS_CMOVBE;
    case FDI_CMOVA: return LL_INS_CMOVA;
    case FDI_CMOVS: return LL_INS_CMOVS;
    case FDI_CMOVNS: return LL_INS_CMOVNS;
    case FDI_CMOVP: return LL_INS_CMOVP;
    case FDI_CMOVNP: return LL_INS_CMOVNP;
    case FDI_CMOVL: return LL_INS_CMOVL;
    case FDI_CMOVGE: return LL_INS_CMOVGE;
    case FDI_CMOVLE: return LL_INS_CMOVLE;
    case FDI_CMOVG: return LL_INS_CMOVG;
    case FDI_SETO: return LL_INS_SETO;
    case FDI_SETNO: return LL_INS_SETNO;
    case FDI_SETC: return LL_INS_SETC;
    case FDI_SETNC: return LL_INS_SETNC;
    case FDI_SETZ: return LL_INS_SETZ;
    case FDI_SETNZ: return LL_INS_SETNZ;
    case FDI_SETBE: return LL_INS_SETBE;
    case FDI_SETA: return LL_INS_SETA;
    case FDI_SETS: return LL_INS_SETS;
    case FDI_SETNS: return LL_INS_SETNS;
    case FDI_SETP: return LL_INS_SETP;
    case FDI_SETNP: return LL_INS_SETNP;
    case FDI_SETL: return LL_INS_SETL;
    case FDI_SETGE: return LL_INS_SETGE;
    case FDI_SETLE: return LL_INS_SETLE;
    case FDI_SETG: return LL_INS_SETG;
    case FDI_LFENCE: return LL_INS_LFENCE;
    case FDI_SFENCE: return LL_INS_SFENCE;
    case FDI_MFENCE: return LL_INS_MFENCE;
    case FDI_PREFETCH0: return LL_INS_PREFETCHT0;
    case FDI_PREFETCH1: return LL_INS_PREFETCHT1;
    case FDI_PREFETCH2: return LL_INS_PREFETCHT2;
    case FDI_PREFETCHNTA: return LL_INS_PREFETCHNTA;
    case FDI_PREFETCHW: return LL_INS_PREFETCHW;
    case FDI_FXSAVE: return LL_INS_FXSAVE;
    case FDI_FXRSTOR: return LL_INS_FXRSTOR;
    case FDI_FSTCW: return LL_INS_FSTCW;
    case FDI_FLDCW: return LL_INS_FLDCW;
    case FDI_FSTSW: return LL_INS_FSTSW;
    case FDI_STMXCSR: return LL_INS_STMXCSR;
    case FDI_LDMXCSR: return LL_INS_LDMXCSR;
    case FDI_SSE_MOVD_G2X: return LL_INS_MOVD;
    case FDI_SSE_MOVD_X2G: return LL_INS_MOVD;
    case FDI_SSE_MOVQ_G2X: return LL_INS_MOVQ;
    case FDI_SSE_MOVQ_X2G: return LL_INS_MOVQ;
    case FDI_SSE_MOVQ_X2X: return LL_INS_MOVQ;
    case FDI_SSE_MOVSS: return LL_INS_MOVSS;
    case FDI_SSE_MOVSD: return LL_INS_MOVSD;
    case FDI_SSE_MOVUPS: return LL_INS_MOVUPS;
    case FDI_SSE_MOVUPD: return LL_INS_MOVUPD;
    case FDI_SSE_MOVAPS: return LL_INS_MOVAPS;
    case FDI_SSE_MOVAPD: return LL_INS_MOVAPD;
    case FDI_SSE_MOVDQU: return LL_INS_MOVDQU;
    case FDI_SSE_MOVDQA: return LL_INS_MOVDQA;
    case FDI_SSE_MOVNTPS: return LL_INS_MOVNTPS;
    case FDI_SSE_MOVNTPD: return LL_INS_MOVNTPD;
    case FDI_SSE_MOVNTDQ: return LL_INS_MOVNTDQ;
    case FDI_SSE_MOVNTDQA: return LL_INS_MOVNTDQA;
    case FDI_SSE_MOVLPS: return LL_INS_MOVLPS;
    case FDI_SSE_MOVLPD: return LL_INS_MOVLPD;
    case FDI_SSE_MOVHPS: return LL_INS_MOVHPS;
    case FDI_SSE_MOVHPD: return LL_INS_MOVHPD;
    case FDI_SSE_PUNPCKLBW: return LL_INS_PUNPCKLBW;
    case FDI_SSE_PUNPCKLWD: return LL_INS_PUNPCKLWD;
    case FDI_SSE_PUNPCKLDQ: return LL_INS_PUNPCKLDQ;
    case FDI_SSE_PUNPCKLQDQ: return LL_INS_PUNPCKLQDQ;
    case FDI_SSE_PUNPCKHBW: return LL_INS_PUNPCKHBW;
    case FDI_SSE_PUNPCKHWD: return LL_INS_PUNPCKHWD;
    case FDI_SSE_PUNPCKHDQ: return LL_INS_PUNPCKHDQ;
    case FDI_SSE_PUNPCKHQDQ: return LL_INS_PUNPCKHQDQ;
    case FDI_SSE_UNPACKLPS: return LL_INS_UNPCKLPS;
    case FDI_SSE_UNPACKLPD: return LL_INS_UNPCKLPD;
    case FDI_SSE_UNPACKHPS: return LL_INS_UNPCKHPS;
    case FDI_SSE_UNPACKHPD: return LL_INS_UNPCKHPD;
    case FDI_SSE_SHUFPD: return LL_INS_SHUFPD;
    case FDI_SSE_SHUFPS: return LL_INS_SHUFPS;
    case FDI_SSE_PSHUFD: return LL_INS_PSHUFD;
    case FDI_SSE_PSHUFLW: return LL_INS_PSHUFLW;
    case FDI_SSE_PSHUFHW: return LL_INS_PSHUFHW;
    case FDI_SSE_INSERTPS: return LL_INS_INSERTPS;
    case FDI_SSE_ADDSS: return LL_INS_ADDSS;
    case FDI_SSE_ADDSD: return LL_INS_ADDSD;
    case FDI_SSE_ADDPS: return LL_INS_ADDPS;
    case FDI_SSE_ADDPD: return LL_INS_ADDPD;
    case FDI_SSE_SUBSS: return LL_INS_SUBSS;
    case FDI_SSE_SUBSD: return LL_INS_SUBSD;
    case FDI_SSE_SUBPS: return LL_INS_SUBPS;
    case FDI_SSE_SUBPD: return LL_INS_SUBPD;
    case FDI_SSE_MULSS: return LL_INS_MULSS;
    case FDI_SSE_MULSD: return LL_INS_MULSD;
    case FDI_SSE_MULPS: return LL_INS_MULPS;
    case FDI_SSE_MULPD: return LL_INS_MULPD;
    case FDI_SSE_DIVSS: return LL_INS_DIVSS;
    case FDI_SSE_DIVSD: return LL_INS_DIVSD;
    case FDI_SSE_DIVPS: return LL_INS_DIVPS;
    case FDI_SSE_DIVPD: return LL_INS_DIVPD;
    case FDI_SSE_MINSS: return LL_INS_MINSS;
    case FDI_SSE_MINSD: return LL_INS_MINSD;
    case FDI_SSE_MINPS: return LL_INS_MINPS;
    case FDI_SSE_MINPD: return LL_INS_MINPD;
    case FDI_SSE_MAXSS: return LL_INS_MAXSS;
    case FDI_SSE_MAXSD: return LL_INS_MAXSD;
    case FDI_SSE_MAXPS: return LL_INS_MAXPS;
    case FDI_SSE_MAXPD: return LL_INS_MAXPD;
    case FDI_SSE_ORPS: return LL_INS_ORPS;
    case FDI_SSE_ORPD: return LL_INS_ORPD;
    case FDI_SSE_ANDPS: return LL_INS_ANDPS;
    case FDI_SSE_ANDPD: return LL_INS_ANDPD;
    case FDI_SSE_XORPS: return LL_INS_XORPS;
    case FDI_SSE_XORPD: return LL_INS_XORPD;
    case FDI_SSE_ANDNPS: return LL_INS_ANDNPS;
    case FDI_SSE_ANDNPD: return LL_INS_ANDNPD;
    case FDI_SSE_COMISS: return LL_INS_COMISS;
    case FDI_SSE_COMISD: return LL_INS_COMISD;
    case FDI_SSE_UCOMISS: return LL_INS_UCOMISS;
    case FDI_SSE_UCOMISD: return LL_INS_UCOMISD;
    case FDI_SSE_CMPSS: return LL_INS_CMPSS;
    case FDI_SSE_CMPSD: return LL_INS_CMPSD;
    case FDI_SSE_CMPPS: return LL_INS_CMPPS;
    case FDI_SSE_CMPPD: return LL_INS_CMPPD;
    case FDI_SSE_SQRTSS: return LL_INS_SQRTSS;
    case FDI_SSE_SQRTSD: return LL_INS_SQRTSD;
    case FDI_SSE_SQRTPS: return LL_INS_SQRTPS;
    case FDI_SSE_SQRTPD: return LL_INS_SQRTPD;
    case FDI_SSE_CVTDQ2PD: return LL_INS_CVTDQ2PD;
    case FDI_SSE_CVTDQ2PS: return LL_INS_CVTDQ2PS;
    case FDI_SSE_CVTPD2DQ: return LL_INS_CVTPD2DQ;
    case FDI_SSE_CVTTPD2DQ: return LL_INS_CVTTPD2DQ;
    case FDI_SSE_CVTPS2DQ: return LL_INS_CVTPS2DQ;
    case FDI_SSE_CVTTPS2DQ: return LL_INS_CVTTPS2DQ;
    case FDI_SSE_CVTPD2PS: return LL_INS_CVTPD2PS;
    case FDI_SSE_CVTPS2PD: return LL_INS_CVTPS2PD;
    case FDI_SSE_CVTSD2SS: return LL_INS_CVTSD2SS;
    case FDI_SSE_CVTSS2SD: return LL_INS_CVTSS2SD;
    case FDI_SSE_CVTSD2SI: return LL_INS_CVTSD2SI;
    case FDI_SSE_CVTTSD2SI: return LL_INS_CVTTSD2SI;
    case FDI_SSE_CVTSS2SI: return LL_INS_CVTSS2SI;
    case FDI_SSE_CVTTSS2SI: return LL_INS_CVTTSS2SI;
    case FDI_SSE_CVTSI2SD: return LL_INS_CVTSI2SD;
    case FDI_SSE_CVTSI2SS: return LL_INS_CVTSI2SS;
    case FDI_SSE_PXOR: return LL_INS_PXOR;
    case FDI_SSE_POR: return LL_INS_POR;
    case FDI_SSE_PAND: return LL_INS_PAND;
    case FDI_SSE_PANDN: return LL_INS_PANDN;
    case FDI_SSE_PADDB: return LL_INS_PADDB;
    case FDI_SSE_PADDW: return LL_INS_PADDW;
    case FDI_SSE_PADDD: return LL_INS_PADDD;
    case FDI_SSE_PADDQ: return LL_INS_PADDQ;
    case FDI_SSE_PSUBB: return LL_INS_PSUBB;
    case FDI_SSE_PSUBW: return LL_INS_PSUBW;
    case FDI_SSE_PSUBD: return LL_INS_PSUBD;
    case FDI_SSE_PSUBQ: return LL_INS_PSUBQ;
    case FDI_SSE_PADDSB: return LL_INS_PADDSB;
    case FDI_SSE_PADDSW: return LL_INS_PADDSW;
    case FDI_SSE_PADDUSB: return LL_INS_PADDUSB;
    case FDI_SSE_PADDUSW: return LL_INS_PADDUSW;
    case FDI_SSE_PSUBSB: return LL_INS_PSUBSB;
    case FDI_SSE_PSUBSW: return LL_INS_PSUBSW;
    case FDI_SSE_PSUBUSB: return LL_INS_PSUBUSB;
    case FDI_SSE_PSUBUSW: return LL_INS_PSUBUSW;
    case FDI_SSE_PMULLW: return LL_INS_PMULLW;
    case FDI_SSE_PMULLD: return LL_INS_PMULLD;
    case FDI_SSE_PSLLW: return LL_INS_PSLLW;
    case FDI_SSE_PSLLD: return LL_INS_PSLLD;
    case FDI_SSE_PSLLQ: return LL_INS_PSLLQ;
    case FDI_SSE_PSRLW: return LL_INS_PSRLW;
    case FDI_SSE_PSRLD: return LL_INS_PSRLD;
    case FDI_SSE_PSRLQ: return LL_INS_PSRLQ;
    case FDI_SSE_PSRAW: return LL_INS_PSRAW;
    case FDI_SSE_PSRAD: return LL_INS_PSRAD;
    case FDI_SSE_PSLLDQ: return LL_INS_PSLLDQ;
    case FDI_SSE_PSRLDQ: return LL_INS_PSRLDQ;
    case FDI_SSE_PACKSSWB: return LL_INS_PACKSSWB;
    case FDI_SSE_PACKSSDW: return LL_INS_PACKSSDW;
    case FDI_SSE_PACKUSWB: return LL_INS_PACKUSWB;
    case FDI_SSE_PACKUSDW: return LL_INS_PACKUSDW;
    case FDI_SSE_PINSRB: return LL_INS_PINSRB;
    case FDI_SSE_PINSRW: return LL_INS_PINSRW;
    case FDI_SSE_PINSRD: return LL_INS_PINSRD;
    case FDI_SSE_PINSRQ: return LL_INS_PINSRQ;
    case FDI_SSE_PEXTRB: return LL_INS_PEXTRB;
    case FDI_SSE_PEXTRW: return LL_INS_PEXTRW;
    case FDI_SSE_PEXTRD: return LL_INS_PEXTRD;
    case FDI_SSE_PEXTRQ: return LL_INS_PEXTRQ;
    case FDI_SSE_PAVGB: return LL_INS_PAVGB;
    case FDI_SSE_PAVGW: return LL_INS_PAVGW;
    case FDI_SSE_PMULHW: return LL_INS_PMULHW;
    case FDI_SSE_PMULHUW: return LL_INS_PMULHUW;
    case FDI_SSE_PCMPEQB: return LL_INS_PCMPEQB;
    case FDI_SSE_PCMPEQW: return LL_INS_PCMPEQW;
    case FDI_SSE_PCMPEQD: return LL_INS_PCMPEQD;
    case FDI_SSE_PCMPEQQ: return LL_INS_PCMPEQQ;
    case FDI_SSE_PCMPGTB: return LL_INS_PCMPGTB;
    case FDI_SSE_PCMPGTW: return LL_INS_PCMPGTW;
    case FDI_SSE_PCMPGTD: return LL_INS_PCMPGTD;
    case FDI_SSE_PCMPGTQ: return LL_INS_PCMPGTQ;
    case FDI_SSE_PMINUB: return LL_INS_PMINUB;
    case FDI_SSE_PMINUW: return LL_INS_PMINUW;
    case FDI_SSE_PMINUD: return LL_INS_PMINUD;
    case FDI_SSE_PMINSB: return LL_INS_PMINSB;
    case FDI_SSE_PMINSW: return LL_INS_PMINSW;
    case FDI_SSE_PMINSD: return LL_INS_PMINSD;
    case FDI_SSE_PMAXUB: return LL_INS_PMAXUB;
    case FDI_SSE_PMAXUW: return LL_INS_PMAXUW;
    case FDI_SSE_PMAXUD: return LL_INS_PMAXUD;
    case FDI_SSE_PMAXSB: return LL_INS_PMAXSB;
    case FDI_SSE_PMAXSW: return LL_INS_PMAXSW;
    case FDI_SSE_PMAXSD: return LL_INS_PMAXSD;
    case FDI_SSE_PMOVMSKB: return LL_INS_PMOVMSKB;
    case FDI_SSE_MOVMSKPS: return LL_INS_MOVMSKPS;
    case FDI_SSE_MOVMSKPD: return LL_INS_MOVMSKPD;
    case FDI_JMP: return LL_INS_JMP;
    case FDI_JMP_IND: return LL_INS_JMP;
    case FDI_JO: return LL_INS_JO;
    case FDI_JNO: return LL_INS_JNO;
    case FDI_JC: return LL_INS_JC;
    case FDI_JNC: return LL_INS_JNC;
    case FDI_JZ: return LL_INS_JZ;
    case FDI_JNZ: return LL_INS_JNZ;
    case FDI_JBE: return LL_INS_JBE;
    case FDI_JA: return LL_INS_JA;
    case FDI_JS: return LL_INS_JS;
    case FDI_JNS: return LL_INS_JNS;
    case FDI_JP: return LL_INS_JP;
    case FDI_JNP: return LL_INS_JNP;
    case FDI_JL: return LL_INS_JL;
    case FDI_JGE: return LL_INS_JGE;
    case FDI_JLE: return LL_INS_JLE;
    case FDI_JG: return LL_INS_JG;
    case FDI_JCXZ: return LL_INS_JCXZ;
    case FDI_LOOP: return LL_INS_LOOP;
    case FDI_LOOPZ: return LL_INS_LOOPE;
    case FDI_LOOPNZ: return LL_INS_LOOPNE;
    case FDI_C_EX: return LL_INS_CEXT;
    case FDI_C_SEP: return LL_INS_CSEP;
    case FDI_ENDBR64: return LL_INS_NOP;
    default: return LL_INS_Invalid;
    }

}

bool Instr::BreaksAlways() const {
    switch (type()) {
    case LL_INS_RET:
    case LL_INS_JMP:
    case LL_INS_CALL:
    case LL_INS_SYSCALL:
    case LL_INS_UD2:
        return true;
    default:
        return false;
    }
}

bool Instr::BreaksConditionally() const {
    switch (type()) {
    case LL_INS_JO:
    case LL_INS_JNO:
    case LL_INS_JC:
    case LL_INS_JNC:
    case LL_INS_JZ:
    case LL_INS_JNZ:
    case LL_INS_JBE:
    case LL_INS_JA:
    case LL_INS_JS:
    case LL_INS_JNS:
    case LL_INS_JP:
    case LL_INS_JNP:
    case LL_INS_JL:
    case LL_INS_JGE:
    case LL_INS_JLE:
    case LL_INS_JG:
    case LL_INS_JCXZ:
    case LL_INS_LOOP:
    case LL_INS_LOOPE:
    case LL_INS_LOOPNE:
        return true;
    default:
        return false;
    }
}

bool Instr::HasAbsJumpTarget() const {
    switch (type()) {
    case LL_INS_JO:
    case LL_INS_JNO:
    case LL_INS_JC:
    case LL_INS_JNC:
    case LL_INS_JZ:
    case LL_INS_JNZ:
    case LL_INS_JBE:
    case LL_INS_JA:
    case LL_INS_JS:
    case LL_INS_JNS:
    case LL_INS_JP:
    case LL_INS_JNP:
    case LL_INS_JL:
    case LL_INS_JGE:
    case LL_INS_JLE:
    case LL_INS_JG:
    case LL_INS_JCXZ:
    case LL_INS_LOOP:
    case LL_INS_LOOPE:
    case LL_INS_LOOPNE:
        return true;
    case LL_INS_JMP:
    case LL_INS_CALL:
        return op(0).is_imm();
    default:
        return false;
    }
}

LLReg Instr::MapFdReg(unsigned idx, unsigned type) {
    if (idx == FD_REG_NONE)
        return LLReg{ LL_RT_None, LL_RI_None };
    if (idx == FD_REG_IP && type == FD_RT_GPL)
        return LLReg{ LL_RT_IP, 0 };
    if (type == FD_RT_GPL)
        return LLReg{ LL_RT_GP, (uint16_t) idx };
    if (type == FD_RT_GPH)
        return LLReg{ LL_RT_GP8High, (uint16_t) idx };
    if (type == FD_RT_VEC)
        return LLReg{ LL_RT_XMM, (uint16_t) idx };
    if (type == FD_RT_SEG)
        return LLReg{ LL_RT_SEG, (uint16_t) idx };
    if (type == FD_RT_BND)
        return LLReg{ LL_RT_BND, (uint16_t) idx };

    fprintf(stderr, "Unknown reg convert %d/%d\n", type, idx);
    return LLReg{ LL_RT_None, LL_RI_None };
}

}

/**
 * @}
 **/
