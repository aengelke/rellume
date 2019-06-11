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

DEF_IT(NOP,)

// Defined in llinstruction-callret.c
DEF_IT(CALL, ll_instruction_call(instr, state))
DEF_IT(RET, ll_instruction_ret(instr, state))

// Defined in llinstruction-stack.c
DEF_IT(PUSH, ll_instruction_stack(instr, state))
DEF_IT(PUSHFQ, ll_instruction_stack(instr, state))
DEF_IT(POP, ll_instruction_stack(instr, state))
DEF_IT(LEAVE, ll_instruction_stack(instr, state))

// Defined in llinstruction-gp.c
DEF_IT(MOV, ll_instruction_movgp(instr, state))
DEF_IT(MOVZX, ll_instruction_movgp(instr, state))
DEF_IT(MOVSX, ll_instruction_movgp(instr, state))
DEF_IT(ADD, ll_instruction_add(instr, state))
DEF_IT(SUB, ll_instruction_sub(instr, state))
DEF_IT(CMP, ll_instruction_cmp(instr, state))
DEF_IT(LEA, ll_instruction_lea(instr, state))
DEF_IT(NOT, ll_instruction_notneg(instr, state))
DEF_IT(NEG, ll_instruction_notneg(instr, state))
DEF_IT(INC, ll_instruction_incdec(instr, state))
DEF_IT(DEC, ll_instruction_incdec(instr, state))
DEF_IT(AND, ll_instruction_logical(instr, state, LLVMAnd))
DEF_IT(OR, ll_instruction_logical(instr, state, LLVMOr))
DEF_IT(XOR, ll_instruction_logical(instr, state, LLVMXor))
DEF_IT(TEST, ll_instruction_test(instr, state))
DEF_IT(IMUL, ll_instruction_mul(instr, state))
DEF_IT(MUL, ll_instruction_mul(instr, state))
DEF_IT(SHL, ll_instruction_shift(instr, state))
DEF_IT(SHR, ll_instruction_shift(instr, state))
DEF_IT(SAR, ll_instruction_shift(instr, state))
DEF_IT(CLTQ, ll_instruction_cdqe(instr, state))

DEF_IT(CMOVO, ll_instruction_cmov(instr, state))
DEF_IT(CMOVNO, ll_instruction_cmov(instr, state))
DEF_IT(CMOVC, ll_instruction_cmov(instr, state))
DEF_IT(CMOVNC, ll_instruction_cmov(instr, state))
DEF_IT(CMOVZ, ll_instruction_cmov(instr, state))
DEF_IT(CMOVNZ, ll_instruction_cmov(instr, state))
DEF_IT(CMOVBE, ll_instruction_cmov(instr, state))
DEF_IT(CMOVA, ll_instruction_cmov(instr, state))
DEF_IT(CMOVS, ll_instruction_cmov(instr, state))
DEF_IT(CMOVNS, ll_instruction_cmov(instr, state))
DEF_IT(CMOVP, ll_instruction_cmov(instr, state))
DEF_IT(CMOVNP, ll_instruction_cmov(instr, state))
DEF_IT(CMOVL, ll_instruction_cmov(instr, state))
DEF_IT(CMOVGE, ll_instruction_cmov(instr, state))
DEF_IT(CMOVLE, ll_instruction_cmov(instr, state))
DEF_IT(CMOVG, ll_instruction_cmov(instr, state))

DEF_IT(SETO, ll_instruction_setcc(instr, state))
DEF_IT(SETNO, ll_instruction_setcc(instr, state))
DEF_IT(SETC, ll_instruction_setcc(instr, state))
DEF_IT(SETNC, ll_instruction_setcc(instr, state))
DEF_IT(SETZ, ll_instruction_setcc(instr, state))
DEF_IT(SETNZ, ll_instruction_setcc(instr, state))
DEF_IT(SETBE, ll_instruction_setcc(instr, state))
DEF_IT(SETA, ll_instruction_setcc(instr, state))
DEF_IT(SETS, ll_instruction_setcc(instr, state))
DEF_IT(SETNS, ll_instruction_setcc(instr, state))
DEF_IT(SETP, ll_instruction_setcc(instr, state))
DEF_IT(SETNP, ll_instruction_setcc(instr, state))
DEF_IT(SETL, ll_instruction_setcc(instr, state))
DEF_IT(SETGE, ll_instruction_setcc(instr, state))
DEF_IT(SETLE, ll_instruction_setcc(instr, state))
DEF_IT(SETG, ll_instruction_setcc(instr, state))

// Defined in llinstruction-sse.c
DEF_IT(MOVD, ll_instruction_movq(instr, state))
DEF_IT(MOVQ, ll_instruction_movq(instr, state))
DEF_IT(MOVSS, ll_instruction_movs(instr, state))
DEF_IT(MOVSD, ll_instruction_movs(instr, state))
DEF_IT(MOVUPS, ll_instruction_movp(instr, state))
DEF_IT(MOVUPD, ll_instruction_movp(instr, state))
DEF_IT(MOVAPS, ll_instruction_movp(instr, state))
DEF_IT(MOVAPD, ll_instruction_movp(instr, state))
DEF_IT(MOVDQU, ll_instruction_movdq(instr, state))
DEF_IT(MOVDQA, ll_instruction_movdq(instr, state))
DEF_IT(MOVLPS, ll_instruction_movlp(instr, state))
DEF_IT(MOVLPD, ll_instruction_movlp(instr, state))
DEF_IT(MOVHPS, ll_instruction_movhps(instr, state))
DEF_IT(MOVHPD, ll_instruction_movhpd(instr, state))
DEF_IT(UNPCKLPS, ll_instruction_unpck(instr, state, OP_VF32))
DEF_IT(UNPCKLPD, ll_instruction_unpck(instr, state, OP_VF64))
DEF_IT(UNPCKHPS, ll_instruction_unpck(instr, state, OP_VF32))
DEF_IT(UNPCKHPD, ll_instruction_unpck(instr, state, OP_VF64))
DEF_IT(SHUFPS, ll_instruction_shufps(instr, state))
DEF_IT(ADDSS, ll_instruction_sse_binary(instr, state, LLVMFAdd, true, OP_SF32))
DEF_IT(ADDSD, ll_instruction_sse_binary(instr, state, LLVMFAdd, true, OP_SF64))
DEF_IT(ADDPS, ll_instruction_sse_binary(instr, state, LLVMFAdd, true, OP_VF32))
DEF_IT(ADDPD, ll_instruction_sse_binary(instr, state, LLVMFAdd, true, OP_VF64))
DEF_IT(SUBSS, ll_instruction_sse_binary(instr, state, LLVMFSub, true, OP_SF32))
DEF_IT(SUBSD, ll_instruction_sse_binary(instr, state, LLVMFSub, true, OP_SF64))
DEF_IT(SUBPS, ll_instruction_sse_binary(instr, state, LLVMFSub, true, OP_VF32))
DEF_IT(SUBPD, ll_instruction_sse_binary(instr, state, LLVMFSub, true, OP_VF64))
DEF_IT(MULSS, ll_instruction_sse_binary(instr, state, LLVMFMul, true, OP_SF32))
DEF_IT(MULSD, ll_instruction_sse_binary(instr, state, LLVMFMul, true, OP_SF64))
DEF_IT(MULPS, ll_instruction_sse_binary(instr, state, LLVMFMul, true, OP_VF32))
DEF_IT(MULPD, ll_instruction_sse_binary(instr, state, LLVMFMul, true, OP_VF64))
DEF_IT(DIVSS, ll_instruction_sse_binary(instr, state, LLVMFDiv, true, OP_SF32))
DEF_IT(DIVSD, ll_instruction_sse_binary(instr, state, LLVMFDiv, true, OP_SF64))
DEF_IT(DIVPS, ll_instruction_sse_binary(instr, state, LLVMFDiv, true, OP_VF32))
DEF_IT(DIVPD, ll_instruction_sse_binary(instr, state, LLVMFDiv, true, OP_VF64))
DEF_IT(ORPS, ll_instruction_sse_binary(instr, state, LLVMOr, false, OP_VI32))
DEF_IT(ORPD, ll_instruction_sse_binary(instr, state, LLVMOr, false, OP_VI64))
DEF_IT(ANDPS, ll_instruction_sse_binary(instr, state, LLVMAnd, false, OP_VI32))
DEF_IT(ANDPD, ll_instruction_sse_binary(instr, state, LLVMAnd, false, OP_VI64))
DEF_IT(XORPS, ll_instruction_sse_binary(instr, state, LLVMXor, false, OP_VI32))
DEF_IT(XORPD, ll_instruction_sse_binary(instr, state, LLVMXor, false, OP_VI64))

DEF_IT(PXOR, ll_instruction_sse_binary(instr, state, LLVMXor, false, OP_VI64))

// Jumps are handled in the basic block generation code.
DEF_IT(JMP,)
DEF_IT(JO,)
DEF_IT(JNO,)
DEF_IT(JC,)
DEF_IT(JNC,)
DEF_IT(JZ,)
DEF_IT(JNZ,)
DEF_IT(JBE,)
DEF_IT(JA,)
DEF_IT(JS,)
DEF_IT(JNS,)
DEF_IT(JP,)
DEF_IT(JNP,)
DEF_IT(JL,)
DEF_IT(JGE,)
DEF_IT(JLE,)
DEF_IT(JG,)
