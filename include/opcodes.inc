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

DEF_IT(NOP, CD_NOP())

// Defined in llinstruction-callret.c
DEF_IT(CALL, CD_FUNCTION(ll_instruction_call))
DEF_IT(RET, CD_FUNCTION(ll_instruction_ret))

// Defined in llinstruction-stack.c
DEF_IT(PUSH, CD_FUNCTION(ll_instruction_stack))
DEF_IT(PUSHFQ, CD_FUNCTION(ll_instruction_stack))
DEF_IT(POP, CD_FUNCTION(ll_instruction_stack))
DEF_IT(LEAVE, CD_FUNCTION(ll_instruction_stack))

// Defined in llinstruction-gp.c
DEF_IT(MOV, CD_FUNCTION(ll_instruction_movgp))
DEF_IT(MOVZX, CD_FUNCTION(ll_instruction_movgp))
DEF_IT(MOVSX, CD_FUNCTION(ll_instruction_movgp))
DEF_IT(ADD, CD_FUNCTION(ll_instruction_add))
DEF_IT(SUB, CD_FUNCTION(ll_instruction_sub))
DEF_IT(CMP, CD_FUNCTION(ll_instruction_cmp))
DEF_IT(LEA, CD_FUNCTION(ll_instruction_lea))
DEF_IT(NOT, CD_FUNCTION(ll_instruction_notneg))
DEF_IT(NEG, CD_FUNCTION(ll_instruction_notneg))
DEF_IT(INC, CD_FUNCTION(ll_instruction_incdec))
DEF_IT(DEC, CD_FUNCTION(ll_instruction_incdec))
DEF_IT(AND, CD_FUNCTION(ll_instruction_logical, LLVMAnd))
DEF_IT(OR, CD_FUNCTION(ll_instruction_logical, LLVMOr))
DEF_IT(XOR, CD_FUNCTION(ll_instruction_logical, LLVMXor))
DEF_IT(TEST, CD_FUNCTION(ll_instruction_test))
DEF_IT(IMUL, CD_FUNCTION(ll_instruction_mul))
DEF_IT(MUL, CD_FUNCTION(ll_instruction_mul))
DEF_IT(SHL, CD_FUNCTION(ll_instruction_shift))
DEF_IT(SHR, CD_FUNCTION(ll_instruction_shift))
DEF_IT(SAR, CD_FUNCTION(ll_instruction_shift))
DEF_IT(CLTQ, CD_FUNCTION(ll_instruction_cdqe))

DEF_IT(CMOVO, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVNO, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVC, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVNC, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVZ, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVNZ, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVBE, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVA, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVS, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVNS, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVP, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVNP, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVL, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVGE, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVLE, CD_FUNCTION(ll_instruction_cmov))
DEF_IT(CMOVG, CD_FUNCTION(ll_instruction_cmov))

DEF_IT(SETO, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETNO, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETC, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETNC, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETZ, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETNZ, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETBE, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETA, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETS, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETNS, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETP, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETNP, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETL, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETGE, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETLE, CD_FUNCTION(ll_instruction_setcc))
DEF_IT(SETG, CD_FUNCTION(ll_instruction_setcc))

// Defined in llinstruction-sse.c
DEF_IT(MOVD, CD_FUNCTION(ll_instruction_movq))
DEF_IT(MOVQ, CD_FUNCTION(ll_instruction_movq))
DEF_IT(MOVSS, CD_FUNCTION(ll_instruction_movs))
DEF_IT(MOVSD, CD_FUNCTION(ll_instruction_movs))
DEF_IT(MOVUPS, CD_FUNCTION(ll_instruction_movp))
DEF_IT(MOVUPD, CD_FUNCTION(ll_instruction_movp))
DEF_IT(MOVAPS, CD_FUNCTION(ll_instruction_movp))
DEF_IT(MOVAPD, CD_FUNCTION(ll_instruction_movp))
DEF_IT(MOVDQU, CD_FUNCTION(ll_instruction_movdq))
DEF_IT(MOVDQA, CD_FUNCTION(ll_instruction_movdq))
DEF_IT(MOVLPS, CD_FUNCTION(ll_instruction_movlp))
DEF_IT(MOVLPD, CD_FUNCTION(ll_instruction_movlp))
DEF_IT(MOVHPS, CD_FUNCTION(ll_instruction_movhps))
DEF_IT(MOVHPD, CD_FUNCTION(ll_instruction_movhpd))
DEF_IT(UNPCKLPS, CD_FUNCTION(ll_instruction_unpckl))
DEF_IT(UNPCKLPD, CD_FUNCTION(ll_instruction_unpckl))
DEF_IT(ADDSS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFAdd, true, OP_SF32))
DEF_IT(ADDSD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFAdd, true, OP_SF64))
DEF_IT(ADDPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFAdd, true, OP_VF32))
DEF_IT(ADDPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFAdd, true, OP_VF64))
DEF_IT(SUBSS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFSub, true, OP_SF32))
DEF_IT(SUBSD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFSub, true, OP_SF64))
DEF_IT(SUBPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFSub, true, OP_VF32))
DEF_IT(SUBPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFSub, true, OP_VF64))
DEF_IT(MULSS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFMul, true, OP_SF32))
DEF_IT(MULSD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFMul, true, OP_SF64))
DEF_IT(MULPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFMul, true, OP_VF32))
DEF_IT(MULPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFMul, true, OP_VF64))
DEF_IT(DIVSS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFDiv, true, OP_SF32))
DEF_IT(DIVSD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFDiv, true, OP_SF64))
DEF_IT(DIVPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMFDiv, true, OP_VF32))
DEF_IT(DIVPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMFDiv, true, OP_VF64))
DEF_IT(ORPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMOr, false, OP_VI32))
DEF_IT(ORPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMOr, false, OP_VI64))
DEF_IT(ANDPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMAnd, false, OP_VI32))
DEF_IT(ANDPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMAnd, false, OP_VI64))
DEF_IT(XORPS, CD_FUNCTION(ll_instruction_sse_binary, LLVMXor, false, OP_VI32))
DEF_IT(XORPD, CD_FUNCTION(ll_instruction_sse_binary, LLVMXor, false, OP_VI64))

DEF_IT(PXOR, CD_FUNCTION(ll_instruction_sse_binary, LLVMXor, false, OP_VI64))

// Jumps are handled in the basic block generation code.
DEF_IT(JMP, CD_NOP())
DEF_IT(JO, CD_NOP())
DEF_IT(JNO, CD_NOP())
DEF_IT(JC, CD_NOP())
DEF_IT(JNC, CD_NOP())
DEF_IT(JZ, CD_NOP())
DEF_IT(JNZ, CD_NOP())
DEF_IT(JBE, CD_NOP())
DEF_IT(JA, CD_NOP())
DEF_IT(JS, CD_NOP())
DEF_IT(JNS, CD_NOP())
DEF_IT(JP, CD_NOP())
DEF_IT(JNP, CD_NOP())
DEF_IT(JL, CD_NOP())
DEF_IT(JGE, CD_NOP())
DEF_IT(JLE, CD_NOP())
DEF_IT(JG, CD_NOP())
