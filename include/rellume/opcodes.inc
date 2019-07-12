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

// Defined in llinstruction-stack.c
DEF_IT(PUSH, state.LiftPushPushf(*instr))
DEF_IT(PUSHFQ, state.LiftPushPushf(*instr))
DEF_IT(POP, state.LiftPopLeaveRet(*instr))
DEF_IT(LEAVE, state.LiftPopLeaveRet(*instr))
DEF_IT(RET, state.LiftPopLeaveRet(*instr))

// Defined in llinstruction-gp.c
DEF_IT(MOV, state.LiftMovgp(*instr, llvm::Instruction::SExt))
DEF_IT(MOVZX, state.LiftMovgp(*instr, llvm::Instruction::ZExt))
DEF_IT(MOVSX, state.LiftMovgp(*instr, llvm::Instruction::SExt))
DEF_IT(ADD, state.LiftAdd(*instr))
DEF_IT(SUB, state.LiftSub(*instr))
DEF_IT(CMP, state.LiftCmp(*instr))
DEF_IT(LEA, state.LiftLea(*instr))
DEF_IT(NOT, state.LiftNot(*instr))
DEF_IT(NEG, state.LiftNeg(*instr))
DEF_IT(INC, state.LiftIncDec(*instr))
DEF_IT(DEC, state.LiftIncDec(*instr))
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

DEF_IT(CMOVO, state.LiftCmovcc(*instr))
DEF_IT(CMOVNO, state.LiftCmovcc(*instr))
DEF_IT(CMOVC, state.LiftCmovcc(*instr))
DEF_IT(CMOVNC, state.LiftCmovcc(*instr))
DEF_IT(CMOVZ, state.LiftCmovcc(*instr))
DEF_IT(CMOVNZ, state.LiftCmovcc(*instr))
DEF_IT(CMOVBE, state.LiftCmovcc(*instr))
DEF_IT(CMOVA, state.LiftCmovcc(*instr))
DEF_IT(CMOVS, state.LiftCmovcc(*instr))
DEF_IT(CMOVNS, state.LiftCmovcc(*instr))
DEF_IT(CMOVP, state.LiftCmovcc(*instr))
DEF_IT(CMOVNP, state.LiftCmovcc(*instr))
DEF_IT(CMOVL, state.LiftCmovcc(*instr))
DEF_IT(CMOVGE, state.LiftCmovcc(*instr))
DEF_IT(CMOVLE, state.LiftCmovcc(*instr))
DEF_IT(CMOVG, state.LiftCmovcc(*instr))

DEF_IT(SETO, state.LiftSetcc(*instr))
DEF_IT(SETNO, state.LiftSetcc(*instr))
DEF_IT(SETC, state.LiftSetcc(*instr))
DEF_IT(SETNC, state.LiftSetcc(*instr))
DEF_IT(SETZ, state.LiftSetcc(*instr))
DEF_IT(SETNZ, state.LiftSetcc(*instr))
DEF_IT(SETBE, state.LiftSetcc(*instr))
DEF_IT(SETA, state.LiftSetcc(*instr))
DEF_IT(SETS, state.LiftSetcc(*instr))
DEF_IT(SETNS, state.LiftSetcc(*instr))
DEF_IT(SETP, state.LiftSetcc(*instr))
DEF_IT(SETNP, state.LiftSetcc(*instr))
DEF_IT(SETL, state.LiftSetcc(*instr))
DEF_IT(SETGE, state.LiftSetcc(*instr))
DEF_IT(SETLE, state.LiftSetcc(*instr))
DEF_IT(SETG, state.LiftSetcc(*instr))

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
DEF_IT(JMP, state.LiftJmp(*instr))
DEF_IT(JO, state.LiftJcc(*instr))
DEF_IT(JNO, state.LiftJcc(*instr))
DEF_IT(JC, state.LiftJcc(*instr))
DEF_IT(JNC, state.LiftJcc(*instr))
DEF_IT(JZ, state.LiftJcc(*instr))
DEF_IT(JNZ, state.LiftJcc(*instr))
DEF_IT(JBE, state.LiftJcc(*instr))
DEF_IT(JA, state.LiftJcc(*instr))
DEF_IT(JS, state.LiftJcc(*instr))
DEF_IT(JNS, state.LiftJcc(*instr))
DEF_IT(JP, state.LiftJcc(*instr))
DEF_IT(JNP, state.LiftJcc(*instr))
DEF_IT(JL, state.LiftJcc(*instr))
DEF_IT(JGE, state.LiftJcc(*instr))
DEF_IT(JLE, state.LiftJcc(*instr))
DEF_IT(JG, state.LiftJcc(*instr))
