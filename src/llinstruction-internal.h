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

#ifndef LL_INSTRUCTION_H
#define LL_INSTRUCTION_H

#include <cstdbool>
#include <cstdint>

#include <llvm-c/Core.h>

#include <llcommon-internal.h>
#include <llstate-internal.h>
#include <rellume/instr.h>

#ifdef __cplusplus
extern "C" {
#endif

void ll_instruction_movgp(LLInstr*, LLState*);
void ll_instruction_add(LLInstr*, LLState*);
void ll_instruction_sub(LLInstr*, LLState*);
void ll_instruction_incdec(LLInstr*, LLState*);
void ll_instruction_shift(LLInstr*, LLState*);
void ll_instruction_rotate(LLInstr*, LLState*);
void ll_instruction_cmov(LLInstr*, LLState*);
void ll_instruction_setcc(LLInstr*, LLState*);
void ll_instruction_cdqe(LLInstr*, LLState*);

void ll_instruction_call(LLInstr*, LLState*);

void ll_instruction_stack(LLInstr*, LLState*);

void ll_instruction_movq(LLInstr* instr, LLState* state);
void ll_instruction_movs(LLInstr* instr, LLState* state);
void ll_instruction_movp(LLInstr* instr, LLState* state);
void ll_instruction_movdq(LLInstr* instr, LLState* state);
void ll_instruction_movlp(LLInstr* instr, LLState* state);
void ll_instruction_movhps(LLInstr* instr, LLState* state);
void ll_instruction_movhpd(LLInstr* instr, LLState* state);
void ll_instruction_shufps(LLInstr* instr, LLState* state);
void ll_instruction_sse_binary(LLInstr*, LLState*, LLVMOpcode, bool,
                               OperandDataType);
void ll_instruction_unpck(LLInstr* instr, LLState* state, OperandDataType);
void ll_instruction_insertps(LLInstr* instr, LLState* state);

void ll_generate_instruction(LLInstr*, LLState*);

#ifdef __cplusplus
}
#endif

#endif
