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

#ifndef LL_BASICBLOCK_H
#define LL_BASICBLOCK_H

#include <rellume/instr.h>

#ifdef __cplusplus
extern "C" {
#endif

struct LLBasicBlock;

typedef struct LLBasicBlock LLBasicBlock;

void ll_basic_block_add_inst(LLBasicBlock*, LLInstr*);
void ll_basic_block_add_branches(LLBasicBlock*, LLBasicBlock*, LLBasicBlock*);

#ifdef __cplusplus
}
#endif

#endif
