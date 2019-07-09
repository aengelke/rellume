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
DEF_IT(CALL, ll_instruction_call(instr, &state))
DEF_IT(RET, new_rip = state.LiftRet(*instr))

// Defined in llinstruction-stack.c
DEF_IT(PUSH, ll_instruction_stack(instr, &state))
DEF_IT(PUSHFQ, ll_instruction_stack(instr, &state))
DEF_IT(POP, ll_instruction_stack(instr, &state))
DEF_IT(LEAVE, ll_instruction_stack(instr, &state))

// Defined in llinstruction-gp.c
DEF_IT(MOV, ll_instruction_movgp(instr, &state))
DEF_IT(MOVZX, ll_instruction_movgp(instr, &state))
DEF_IT(MOVSX, ll_instruction_movgp(instr, &state))
DEF_IT(ADD, ll_instruction_add(instr, &state))
DEF_IT(SUB, ll_instruction_sub(instr, &state))
DEF_IT(CMP, state.LiftCmp(*instr))
DEF_IT(LEA, state.LiftLea(*instr))
DEF_IT(NOT, state.LiftNot(*instr))
DEF_IT(NEG, state.LiftNeg(*instr))
DEF_IT(INC, ll_instruction_incdec(instr, &state))
DEF_IT(DEC, ll_instruction_incdec(instr, &state))
DEF_IT(AND, state.LiftAndOrXor(*instr, llvm::Instruction::And))
DEF_IT(OR, state.LiftAndOrXor(*instr, llvm::Instruction::Or))
DEF_IT(XOR, state.LiftAndOrXor(*instr, llvm::Instruction::Xor))
DEF_IT(TEST, state.LiftAndOrXor(*instr, llvm::Instruction::And, /*wb=*/false))
DEF_IT(IMUL, state.LiftMul(*instr))
DEF_IT(MUL, state.LiftMul(*instr))
DEF_IT(SHL, ll_instruction_shift(instr, &state))
DEF_IT(SHR, ll_instruction_shift(instr, &state))
DEF_IT(SAR, ll_instruction_shift(instr, &state))
DEF_IT(CLTQ, ll_instruction_cdqe(instr, &state))

DEF_IT(CMOVO, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVNO, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVC, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVNC, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVZ, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVNZ, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVBE, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVA, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVS, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVNS, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVP, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVNP, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVL, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVGE, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVLE, ll_instruction_cmov(instr, &state))
DEF_IT(CMOVG, ll_instruction_cmov(instr, &state))

DEF_IT(SETO, ll_instruction_setcc(instr, &state))
DEF_IT(SETNO, ll_instruction_setcc(instr, &state))
DEF_IT(SETC, ll_instruction_setcc(instr, &state))
DEF_IT(SETNC, ll_instruction_setcc(instr, &state))
DEF_IT(SETZ, ll_instruction_setcc(instr, &state))
DEF_IT(SETNZ, ll_instruction_setcc(instr, &state))
DEF_IT(SETBE, ll_instruction_setcc(instr, &state))
DEF_IT(SETA, ll_instruction_setcc(instr, &state))
DEF_IT(SETS, ll_instruction_setcc(instr, &state))
DEF_IT(SETNS, ll_instruction_setcc(instr, &state))
DEF_IT(SETP, ll_instruction_setcc(instr, &state))
DEF_IT(SETNP, ll_instruction_setcc(instr, &state))
DEF_IT(SETL, ll_instruction_setcc(instr, &state))
DEF_IT(SETGE, ll_instruction_setcc(instr, &state))
DEF_IT(SETLE, ll_instruction_setcc(instr, &state))
DEF_IT(SETG, ll_instruction_setcc(instr, &state))

// Defined in llinstruction-sse.c
DEF_IT(MOVD, state.LiftSseMovq(*instr, Facet::I32))
DEF_IT(MOVQ, state.LiftSseMovq(*instr, Facet::I64))
DEF_IT(MOVSS, ll_instruction_movs(instr, &state))
DEF_IT(MOVSD, ll_instruction_movs(instr, &state))
DEF_IT(MOVUPS, ll_instruction_movp(instr, &state))
DEF_IT(MOVUPD, ll_instruction_movp(instr, &state))
DEF_IT(MOVAPS, ll_instruction_movp(instr, &state))
DEF_IT(MOVAPD, ll_instruction_movp(instr, &state))
DEF_IT(MOVDQU, ll_instruction_movdq(instr, &state))
DEF_IT(MOVDQA, ll_instruction_movdq(instr, &state))
DEF_IT(MOVLPS, ll_instruction_movlp(instr, &state))
DEF_IT(MOVLPD, ll_instruction_movlp(instr, &state))
DEF_IT(MOVHPS, ll_instruction_movhps(instr, &state))
DEF_IT(MOVHPD, ll_instruction_movhpd(instr, &state))
DEF_IT(UNPCKLPS, ll_instruction_unpck(instr, &state, OP_VF32))
DEF_IT(UNPCKLPD, ll_instruction_unpck(instr, &state, OP_VF64))
DEF_IT(UNPCKHPS, ll_instruction_unpck(instr, &state, OP_VF32))
DEF_IT(UNPCKHPD, ll_instruction_unpck(instr, &state, OP_VF64))
DEF_IT(SHUFPS, ll_instruction_shufps(instr, &state))
DEF_IT(INSERTPS, ll_instruction_insertps(instr, &state))
DEF_IT(ADDSS, ll_instruction_sse_binary(instr, &state, LLVMFAdd, true, OP_SF32))
DEF_IT(ADDSD, ll_instruction_sse_binary(instr, &state, LLVMFAdd, true, OP_SF64))
DEF_IT(ADDPS, ll_instruction_sse_binary(instr, &state, LLVMFAdd, true, OP_VF32))
DEF_IT(ADDPD, ll_instruction_sse_binary(instr, &state, LLVMFAdd, true, OP_VF64))
DEF_IT(SUBSS, ll_instruction_sse_binary(instr, &state, LLVMFSub, true, OP_SF32))
DEF_IT(SUBSD, ll_instruction_sse_binary(instr, &state, LLVMFSub, true, OP_SF64))
DEF_IT(SUBPS, ll_instruction_sse_binary(instr, &state, LLVMFSub, true, OP_VF32))
DEF_IT(SUBPD, ll_instruction_sse_binary(instr, &state, LLVMFSub, true, OP_VF64))
DEF_IT(MULSS, ll_instruction_sse_binary(instr, &state, LLVMFMul, true, OP_SF32))
DEF_IT(MULSD, ll_instruction_sse_binary(instr, &state, LLVMFMul, true, OP_SF64))
DEF_IT(MULPS, ll_instruction_sse_binary(instr, &state, LLVMFMul, true, OP_VF32))
DEF_IT(MULPD, ll_instruction_sse_binary(instr, &state, LLVMFMul, true, OP_VF64))
DEF_IT(DIVSS, ll_instruction_sse_binary(instr, &state, LLVMFDiv, true, OP_SF32))
DEF_IT(DIVSD, ll_instruction_sse_binary(instr, &state, LLVMFDiv, true, OP_SF64))
DEF_IT(DIVPS, ll_instruction_sse_binary(instr, &state, LLVMFDiv, true, OP_VF32))
DEF_IT(DIVPD, ll_instruction_sse_binary(instr, &state, LLVMFDiv, true, OP_VF64))
DEF_IT(ORPS, ll_instruction_sse_binary(instr, &state, LLVMOr, false, OP_VI32))
DEF_IT(ORPD, ll_instruction_sse_binary(instr, &state, LLVMOr, false, OP_VI64))
DEF_IT(ANDPS, ll_instruction_sse_binary(instr, &state, LLVMAnd, false, OP_VI32))
DEF_IT(ANDPD, ll_instruction_sse_binary(instr, &state, LLVMAnd, false, OP_VI64))
DEF_IT(XORPS, ll_instruction_sse_binary(instr, &state, LLVMXor, false, OP_VI32))
DEF_IT(XORPD, ll_instruction_sse_binary(instr, &state, LLVMXor, false, OP_VI64))

DEF_IT(PXOR, ll_instruction_sse_binary(instr, &state, LLVMXor, false, OP_VI64))

// Jumps are handled in the basic block generation code.
DEF_IT(JMP, new_rip = state.LiftJmp(*instr))
DEF_IT(JO, new_rip = state.LiftJcc(*instr))
DEF_IT(JNO, new_rip = state.LiftJcc(*instr))
DEF_IT(JC, new_rip = state.LiftJcc(*instr))
DEF_IT(JNC, new_rip = state.LiftJcc(*instr))
DEF_IT(JZ, new_rip = state.LiftJcc(*instr))
DEF_IT(JNZ, new_rip = state.LiftJcc(*instr))
DEF_IT(JBE, new_rip = state.LiftJcc(*instr))
DEF_IT(JA, new_rip = state.LiftJcc(*instr))
DEF_IT(JS, new_rip = state.LiftJcc(*instr))
DEF_IT(JNS, new_rip = state.LiftJcc(*instr))
DEF_IT(JP, new_rip = state.LiftJcc(*instr))
DEF_IT(JNP, new_rip = state.LiftJcc(*instr))
DEF_IT(JL, new_rip = state.LiftJcc(*instr))
DEF_IT(JGE, new_rip = state.LiftJcc(*instr))
DEF_IT(JLE, new_rip = state.LiftJcc(*instr))
DEF_IT(JG, new_rip = state.LiftJcc(*instr))
