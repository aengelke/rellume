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

#ifndef LL_FLAGS_INTERNAL_H
#define LL_FLAGS_INTERNAL_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <llvm-c/Core.h>

#include <llcommon-internal.h>
#include <llinstr-internal.h>


LLVMValueRef ll_flags_condition(LLInstrType, LLInstrType, LLState*);

void ll_flags_set_af(LLVMValueRef, LLVMValueRef, LLVMValueRef, LLState*);
void ll_flags_set_zf(LLVMValueRef, LLState*);
void ll_flags_set_sf(LLVMValueRef, LLState*);
void ll_flags_set_pf(LLVMValueRef, LLState*);
void ll_flags_set_of_sub(LLVMValueRef, LLVMValueRef, LLVMValueRef, LLState*);
void ll_flags_set_of_imul(LLVMValueRef, LLVMValueRef, LLVMValueRef, LLState*);

void ll_flags_set_sub(LLVMValueRef, LLVMValueRef, LLVMValueRef, LLState*);
void ll_flags_set_add(LLVMValueRef, LLVMValueRef, LLVMValueRef, LLState*);
void ll_flags_set_bit(LLState*, LLVMValueRef, LLVMValueRef, LLVMValueRef);
void ll_flags_set_inc(LLVMValueRef, LLVMValueRef, LLState*);
void ll_flags_set_dec(LLVMValueRef, LLVMValueRef, LLState*);
void ll_flags_set_shl(LLState*, LLVMValueRef, LLVMValueRef, LLVMValueRef);
void ll_flags_set_shr(LLState*, LLVMValueRef, LLVMValueRef, LLVMValueRef);
void ll_flags_set_sar(LLState*, LLVMValueRef, LLVMValueRef, LLVMValueRef);
void ll_flags_invalidate(LLState*);

#endif
