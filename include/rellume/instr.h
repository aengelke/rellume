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

#ifndef LL_INSTR_H
#define LL_INSTR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <fadec.h>

#ifdef __cplusplus
extern "C" {
#endif


struct LLReg {
    uint16_t rt;
    uint16_t ri;
#if defined(__cplusplus) && defined(RELLUME_ENABLE_CPP_HEADER)
    explicit operator bool() const { return ri != FD_REG_NONE; }
#endif
};

typedef struct LLReg LLReg;

#ifdef __cplusplus
}
#endif

#endif
