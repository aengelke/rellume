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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

extern "C" {
#include <fadec.h>
}

#include <rellume/instr.h>

#include <llcommon-internal.h>


/**
 * \defgroup LLInstr Instruction
 * \brief Representation of an instruction
 *
 * @{
 **/

const char*
LLReg::Name() const
{
    int max;
    const char (* table)[6];

#define TABLE(name, ...) \
        case name : { \
            static const char tab[][6] = { __VA_ARGS__ }; \
            table = tab; max = sizeof(tab) / sizeof(*tab); break; }

    switch (rt) {
    default: return "(unk)";

    TABLE(LL_RT_GP8,
        "al","cl","dl","bl","spl","bpl","sil","dil",
        "r8b","r9b","r10b","r11b","r12b","r13b","r14b","r15b")
    TABLE(LL_RT_GP8Leg,
        "al","cl","dl","bl","ah","ch","dh","bh",
        "r8b","r9b","r10b","r11b","r12b","r13b","r14b","r15b")
    TABLE(LL_RT_GP16,
        "ax","cx","dx","bx","sp","bp","si","di",
        "r8w","r9w","r10w","r11w","r12w","r13w","r14w","r15w")
    TABLE(LL_RT_GP32,
        "eax","ecx","edx","ebx","esp","ebp","esi","edi",
        "r8d","r9d","r10d","r11d","r12d","r13d","r14d","r15d")
    TABLE(LL_RT_GP64,
        "rax","rcx","rdx","rbx","rsp","rbp","rsi","rdi",
        "r8","r9","r10","r11","r12","r13","r14","r15")
    TABLE(LL_RT_IP, "rip")
    TABLE(LL_RT_XMM,
        "xmm0","xmm1","xmm2","xmm3","xmm4","xmm5","xmm6","xmm7",
        "xmm8","xmm9","xmm10","xmm11","xmm12","xmm13","xmm14","xmm15")
    TABLE(LL_RT_YMM,
        "ymm0","ymm1","ymm2","ymm3","ymm4","ymm5","ymm6","ymm7",
        "ymm8","ymm9","ymm10","ymm11","ymm12","ymm13","ymm14","ymm15")

    }

    return ri < max ? table[ri] : "(inv)";
}

size_t
LLReg::Size() const
{
    switch (rt) {
    case LL_RT_GP8:     return 1;
    case LL_RT_GP8Leg:  return 1;
    case LL_RT_GP16:    return 2;
    case LL_RT_GP32:    return 4;
    case LL_RT_GP64:    return 8;
    case LL_RT_IP:      return 8;
    case LL_RT_XMM:     return 16;
    default:            return 0;
    }
}

static LLReg
convert_reg(int size, int idx, int type)
{
    if (idx == FD_REG_NONE)
        return LLReg{ LL_RT_None, LL_RI_None };
    if (idx == FD_REG_IP && type == FD_RT_GPL)
        return LLReg{ LL_RT_IP, 0 };
    if (type == FD_RT_GPL)
        return LLReg::Gp(size, idx, /*legacy=*/false);
    if (type == FD_RT_GPH)
        return LLReg::Gp(size, idx);
    if (type == FD_RT_VEC && size == 32)
        return LLReg{ LL_RT_YMM, (uint16_t) idx };
    if (type == FD_RT_VEC)
        return LLReg{ LL_RT_XMM, (uint16_t) idx };
    if (type == FD_RT_SEG)
        return LLReg{ LL_RT_SEG, (uint16_t) idx };

    printf("Unknown reg convert %d/%d\n", type, size);
    abort();
}

LLInstr LLInstr::Decode(uint8_t* buf, size_t buf_size, uint64_t addr)
{
    LLInstr llinst;
    FdInstr fdi;
    int ret = fd_decode(buf, buf_size, 64, addr, &fdi);
    if (ret < 0)
        return LLInstr::Invalid(addr);

    llinst.addr = addr;
    llinst.len = FD_SIZE(&fdi);

    llinst.operand_count = 0;
    for (int i = 0; i < 3; i++)
    {
        switch (FD_OP_TYPE(&fdi, i))
        {
        case FD_OT_NONE:
            goto end_ops;
        case FD_OT_IMM:
            llinst.ops[i].type = LL_OP_IMM;
            llinst.ops[i].val = FD_OP_IMM(&fdi, i);
            llinst.ops[i].size = FD_OP_SIZE(&fdi, i);
            break;
        case FD_OT_REG:
            llinst.ops[i].type = LL_OP_REG;
            llinst.ops[i].size = FD_OP_SIZE(&fdi, i);
            llinst.ops[i].scale = 0;
            llinst.ops[i].val = 0;
            llinst.ops[i].reg = convert_reg(FD_OP_SIZE(&fdi, i), FD_OP_REG(&fdi, i),
                                      FD_OP_REG_TYPE(&fdi, i));
            break;
        case FD_OT_MEM:
            llinst.ops[i].type = LL_OP_MEM;
            llinst.ops[i].seg = convert_reg(2, FD_SEGMENT(&fdi), FD_RT_SEG).ri;
            llinst.ops[i].val = FD_OP_DISP(&fdi, i);
            llinst.ops[i].reg = convert_reg(8, FD_OP_BASE(&fdi, i), FD_RT_GPL);
            if (FD_OP_INDEX(&fdi, i) != FD_REG_NONE)
            {
                llinst.ops[i].ireg = convert_reg(8, FD_OP_INDEX(&fdi, i), FD_RT_GPL);
                llinst.ops[i].scale = 1 << FD_OP_SCALE(&fdi, i);
            }
            else
            {
                llinst.ops[i].scale = 0;
            }
            llinst.ops[i].size = FD_OP_SIZE(&fdi, i);
            break;
        }
        llinst.operand_count = i + 1;
    }
end_ops:

    switch (FD_TYPE(&fdi))
    {
    case FDI_NOP: llinst.type = LL_INS_NOP; break;
    case FDI_CALL: llinst.type = LL_INS_CALL; break;
    case FDI_RET: llinst.type = LL_INS_RET; break;
    case FDI_PUSH: llinst.type = LL_INS_PUSH; break;
    case FDI_PUSHF: llinst.type = LL_INS_PUSHFQ; break;
    case FDI_POP: llinst.type = LL_INS_POP; break;
    case FDI_LEAVE: llinst.type = LL_INS_LEAVE; break;
    case FDI_MOV: llinst.type = LL_INS_MOV; break;
    case FDI_MOV_IMM: llinst.type = LL_INS_MOV; break;
    case FDI_MOVABS_IMM: llinst.type = LL_INS_MOV; break;
    case FDI_MOVZX: llinst.type = LL_INS_MOVZX; break;
    case FDI_MOVSX: llinst.type = LL_INS_MOVSX; break;
    case FDI_ADD: llinst.type = LL_INS_ADD; break;
    case FDI_ADD_IMM: llinst.type = LL_INS_ADD; break;
    case FDI_SUB: llinst.type = LL_INS_SUB; break;
    case FDI_SUB_IMM: llinst.type = LL_INS_SUB; break;
    case FDI_CMP: llinst.type = LL_INS_CMP; break;
    case FDI_CMP_IMM: llinst.type = LL_INS_CMP; break;
    case FDI_LEA: llinst.type = LL_INS_LEA; break;
    case FDI_NOT: llinst.type = LL_INS_NOT; break;
    case FDI_NEG: llinst.type = LL_INS_NEG; break;
    case FDI_INC: llinst.type = LL_INS_INC; break;
    case FDI_DEC: llinst.type = LL_INS_DEC; break;
    case FDI_AND: llinst.type = LL_INS_AND; break;
    case FDI_AND_IMM: llinst.type = LL_INS_AND; break;
    case FDI_OR: llinst.type = LL_INS_OR; break;
    case FDI_OR_IMM: llinst.type = LL_INS_OR; break;
    case FDI_XOR: llinst.type = LL_INS_XOR; break;
    case FDI_XOR_IMM: llinst.type = LL_INS_XOR; break;
    case FDI_TEST: llinst.type = LL_INS_TEST; break;
    case FDI_TEST_IMM: llinst.type = LL_INS_TEST; break;
    case FDI_IMUL: llinst.type = LL_INS_IMUL; break;
    case FDI_IMUL2: llinst.type = LL_INS_IMUL; break;
    case FDI_IMUL3: llinst.type = LL_INS_IMUL; break;
    case FDI_MUL: llinst.type = LL_INS_MUL; break;
    case FDI_SHL_IMM: llinst.type = LL_INS_SHL; break;
    case FDI_SHL_CL: llinst.type = LL_INS_SHL; llinst.ops[1].type = LL_OP_REG; llinst.ops[1].size = 1; llinst.ops[1].reg = LLReg::Gp(1, 1); llinst.operand_count = 2; break;
    case FDI_SHR_IMM: llinst.type = LL_INS_SHR; break;
    case FDI_SHR_CL: llinst.type = LL_INS_SHR; llinst.ops[1].type = LL_OP_REG; llinst.ops[1].size = 1; llinst.ops[1].reg = LLReg::Gp(1, 1); llinst.operand_count = 2; break;
    case FDI_SAR_IMM: llinst.type = LL_INS_SAR; break;
    case FDI_SAR_CL: llinst.type = LL_INS_SAR; llinst.ops[1].type = LL_OP_REG; llinst.ops[1].size = 1; llinst.ops[1].reg = LLReg::Gp(1, 1); llinst.operand_count = 2; break;
    case FDI_CMOVO: llinst.type = LL_INS_CMOVO; break;
    case FDI_CMOVNO: llinst.type = LL_INS_CMOVNO; break;
    case FDI_CMOVC: llinst.type = LL_INS_CMOVC; break;
    case FDI_CMOVNC: llinst.type = LL_INS_CMOVNC; break;
    case FDI_CMOVZ: llinst.type = LL_INS_CMOVZ; break;
    case FDI_CMOVNZ: llinst.type = LL_INS_CMOVNZ; break;
    case FDI_CMOVBE: llinst.type = LL_INS_CMOVBE; break;
    case FDI_CMOVA: llinst.type = LL_INS_CMOVA; break;
    case FDI_CMOVS: llinst.type = LL_INS_CMOVS; break;
    case FDI_CMOVNS: llinst.type = LL_INS_CMOVNS; break;
    case FDI_CMOVP: llinst.type = LL_INS_CMOVP; break;
    case FDI_CMOVNP: llinst.type = LL_INS_CMOVNP; break;
    case FDI_CMOVL: llinst.type = LL_INS_CMOVL; break;
    case FDI_CMOVGE: llinst.type = LL_INS_CMOVGE; break;
    case FDI_CMOVLE: llinst.type = LL_INS_CMOVLE; break;
    case FDI_CMOVG: llinst.type = LL_INS_CMOVG; break;
    case FDI_SETO: llinst.type = LL_INS_SETO; break;
    case FDI_SETNO: llinst.type = LL_INS_SETNO; break;
    case FDI_SETC: llinst.type = LL_INS_SETC; break;
    case FDI_SETNC: llinst.type = LL_INS_SETNC; break;
    case FDI_SETZ: llinst.type = LL_INS_SETZ; break;
    case FDI_SETNZ: llinst.type = LL_INS_SETNZ; break;
    case FDI_SETBE: llinst.type = LL_INS_SETBE; break;
    case FDI_SETA: llinst.type = LL_INS_SETA; break;
    case FDI_SETS: llinst.type = LL_INS_SETS; break;
    case FDI_SETNS: llinst.type = LL_INS_SETNS; break;
    case FDI_SETP: llinst.type = LL_INS_SETP; break;
    case FDI_SETNP: llinst.type = LL_INS_SETNP; break;
    case FDI_SETL: llinst.type = LL_INS_SETL; break;
    case FDI_SETGE: llinst.type = LL_INS_SETGE; break;
    case FDI_SETLE: llinst.type = LL_INS_SETLE; break;
    case FDI_SETG: llinst.type = LL_INS_SETG; break;
    case FDI_SSE_MOVD_G2X: llinst.type = LL_INS_MOVD; break;
    case FDI_SSE_MOVD_X2G: llinst.type = LL_INS_MOVD; break;
    case FDI_SSE_MOVQ_G2X: llinst.type = LL_INS_MOVQ; break;
    case FDI_SSE_MOVQ_X2G: llinst.type = LL_INS_MOVQ; break;
    case FDI_SSE_MOVQ_X2X: llinst.type = LL_INS_MOVQ; break;
    case FDI_SSE_MOVSS: llinst.type = LL_INS_MOVSS; break;
    case FDI_SSE_MOVSD: llinst.type = LL_INS_MOVSD; break;
    case FDI_SSE_MOVUPS: llinst.type = LL_INS_MOVUPS; break;
    case FDI_SSE_MOVUPD: llinst.type = LL_INS_MOVUPD; break;
    case FDI_SSE_MOVAPS: llinst.type = LL_INS_MOVAPS; break;
    case FDI_SSE_MOVAPD: llinst.type = LL_INS_MOVAPD; break;
    case FDI_SSE_MOVDQU: llinst.type = LL_INS_MOVDQU; break;
    case FDI_SSE_MOVDQA: llinst.type = LL_INS_MOVDQA; break;
    case FDI_SSE_MOVLPS: llinst.type = LL_INS_MOVLPS; break;
    case FDI_SSE_MOVLPD: llinst.type = LL_INS_MOVLPD; break;
    case FDI_SSE_MOVHPS: llinst.type = LL_INS_MOVHPS; break;
    case FDI_SSE_MOVHPD: llinst.type = LL_INS_MOVHPD; break;
    case FDI_SSE_UNPACKLPS: llinst.type = LL_INS_UNPCKLPS; break;
    case FDI_SSE_UNPACKLPD: llinst.type = LL_INS_UNPCKLPD; break;
    case FDI_SSE_UNPACKHPS: llinst.type = LL_INS_UNPCKHPS; break;
    case FDI_SSE_UNPACKHPD: llinst.type = LL_INS_UNPCKHPD; break;
    case FDI_SSE_SHUFPS: llinst.type = LL_INS_SHUFPS; break;
    case FDI_SSE_INSERTPS: llinst.type = LL_INS_INSERTPS; break;
    case FDI_SSE_ADDSS: llinst.type = LL_INS_ADDSS; break;
    case FDI_SSE_ADDSD: llinst.type = LL_INS_ADDSD; break;
    case FDI_SSE_ADDPS: llinst.type = LL_INS_ADDPS; break;
    case FDI_SSE_ADDPD: llinst.type = LL_INS_ADDPD; break;
    case FDI_SSE_SUBSS: llinst.type = LL_INS_SUBSS; break;
    case FDI_SSE_SUBSD: llinst.type = LL_INS_SUBSD; break;
    case FDI_SSE_SUBPS: llinst.type = LL_INS_SUBPS; break;
    case FDI_SSE_SUBPD: llinst.type = LL_INS_SUBPD; break;
    case FDI_SSE_MULSS: llinst.type = LL_INS_MULSS; break;
    case FDI_SSE_MULSD: llinst.type = LL_INS_MULSD; break;
    case FDI_SSE_MULPS: llinst.type = LL_INS_MULPS; break;
    case FDI_SSE_MULPD: llinst.type = LL_INS_MULPD; break;
    case FDI_SSE_DIVSS: llinst.type = LL_INS_DIVSS; break;
    case FDI_SSE_DIVSD: llinst.type = LL_INS_DIVSD; break;
    case FDI_SSE_DIVPS: llinst.type = LL_INS_DIVPS; break;
    case FDI_SSE_DIVPD: llinst.type = LL_INS_DIVPD; break;
    case FDI_SSE_ORPS: llinst.type = LL_INS_ORPS; break;
    case FDI_SSE_ORPD: llinst.type = LL_INS_ORPD; break;
    case FDI_SSE_ANDPS: llinst.type = LL_INS_ANDPS; break;
    case FDI_SSE_ANDPD: llinst.type = LL_INS_ANDPD; break;
    case FDI_SSE_XORPS: llinst.type = LL_INS_XORPS; break;
    case FDI_SSE_XORPD: llinst.type = LL_INS_XORPD; break;
    case FDI_SSE_PXOR: llinst.type = LL_INS_PXOR; break;
    case FDI_JMP: llinst.type = LL_INS_JMP; break;
    case FDI_JO: llinst.type = LL_INS_JO; break;
    case FDI_JNO: llinst.type = LL_INS_JNO; break;
    case FDI_JC: llinst.type = LL_INS_JC; break;
    case FDI_JNC: llinst.type = LL_INS_JNC; break;
    case FDI_JZ: llinst.type = LL_INS_JZ; break;
    case FDI_JNZ: llinst.type = LL_INS_JNZ; break;
    case FDI_JBE: llinst.type = LL_INS_JBE; break;
    case FDI_JA: llinst.type = LL_INS_JA; break;
    case FDI_JS: llinst.type = LL_INS_JS; break;
    case FDI_JNS: llinst.type = LL_INS_JNS; break;
    case FDI_JP: llinst.type = LL_INS_JP; break;
    case FDI_JNP: llinst.type = LL_INS_JNP; break;
    case FDI_JL: llinst.type = LL_INS_JL; break;
    case FDI_JGE: llinst.type = LL_INS_JGE; break;
    case FDI_JLE: llinst.type = LL_INS_JLE; break;
    case FDI_JG: llinst.type = LL_INS_JG; break;
    case FDI_C_EX: if (FD_OPSIZE(&fdi) == 8) llinst.type = LL_INS_CLTQ; break;
    default: {
        char buf[128];
        fd_format(&fdi, buf, sizeof(buf));
        printf("Cannot convert instruction %s\n", buf);
        return LLInstr::Invalid(addr);
    }
    }

    return llinst;
}

/**
 * @}
 **/
