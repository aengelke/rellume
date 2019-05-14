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

#ifndef LL_INSTR_INTERNAL_H
#define LL_INSTR_INTERNAL_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <llinstr.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
#define ll_reg(t,i) LLReg{ (uint16_t) t, (uint16_t) i }
#else
#define ll_reg(t,i) ((LLReg) { .rt = t, .ri = i })
#endif
LLReg ll_reg_gp(size_t size, bool legacy, int index);

const char* ll_reg_name(LLReg reg);

size_t ll_reg_size(LLReg reg);
#define ll_reg_high(reg) ((reg).rt == LL_RT_GP8Leg && (reg).ri >= 4 && (reg).ri < 8)

#define regIsGP(r) ((r).rt == LL_RT_GP8Leg || (r).rt == LL_RT_GP8 || (r).rt == LL_RT_GP16 || (r).rt == LL_RT_GP32 || (r).rt == LL_RT_GP64)
#define regIsV(r) ((r).rt == LL_RT_XMM || (r).rt == LL_RT_YMM)

#define getRegOp(r) &((LLInstrOp) { .type = 1, .reg = (r), .val = 0, .ireg = ll_reg(LL_RT_None, LL_RI_None), .scale = 0, .seg = LL_RI_None, .size = ll_reg_size(r) })

#define instr2string(instr, ...) "<llinstr>"

#ifdef __cplusplus
}
#endif

#endif

