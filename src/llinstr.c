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
#include <llvm-c/Core.h>

#include <llinstr.h>
#include <llinstr-internal.h>

#include <llcommon-internal.h>


/**
 * \defgroup LLInstr Instruction
 * \brief Representation of an instruction
 *
 * @{
 **/

LLReg
ll_reg_gp(size_t size, bool legacy, int index) {
    switch (size) {
    case 1: return ll_reg(legacy ? LL_RT_GP8Leg : LL_RT_GP8, index);
    case 2: return ll_reg(LL_RT_GP16, index);
    case 4: return ll_reg(LL_RT_GP32, index);
    case 8: return ll_reg(LL_RT_GP64, index);
    default: warn_if_reached(); return ll_reg(LL_RT_None, LL_RI_None);
    }
}

static const char* names_reg_gp8 =
    "al\0   cl\0   dl\0   bl\0   spl\0  bpl\0  sil\0  dil\0  "
    "r8b\0  r9b\0  r10b\0 r11b\0 r12b\0 r13b\0 r14b\0 r15b\0 ";
static const char* names_reg_gp8leg =
    "al\0   cl\0   dl\0   bl\0   ah\0   ch\0   dh\0   bh\0   "
    "r8b\0  r9b\0  r10b\0 r11b\0 r12b\0 r13b\0 r14b\0 r15b\0 ";
static const char* names_reg_gp16 =
    "ax\0   cx\0   dx\0   bx\0   sp\0   bp\0   si\0   di\0   "
    "r8w\0  r9w\0  r10w\0 r11w\0 r12w\0 r13w\0 r14w\0 r15w\0 ";
static const char* names_reg_gp32 =
    "eax\0  ecx\0  edx\0  ebx\0  esp\0  ebp\0  esi\0  edi\0  "
    "r8d\0  r9d\0  r10d\0 r11d\0 r12d\0 r13d\0 r14d\0 r15d\0 ";
static const char* names_reg_gp64 =
    "rax\0  rcx\0  rdx\0  rbx\0  rsp\0  rbp\0  rsi\0  rdi\0  "
    "r8\0   r9\0   r10\0  r11\0  r12\0  r13\0  r14\0  r15\0  ";
static const char* names_reg_vec128 =
    "xmm0\0 xmm1\0 xmm2\0 xmm3\0 xmm4\0 xmm5\0 xmm6\0 xmm7\0 "
    "xmm8\0 xmm9\0 xmm10\0xmm11\0xmm12\0xmm13\0xmm14\0xmm15\0";
static const char* names_reg_vec256 =
    "ymm0\0 ymm1\0 ymm2\0 ymm3\0 ymm4\0 ymm5\0 ymm6\0 ymm7\0 "
    "ymm8\0 ymm9\0 ymm10\0ymm11\0ymm12\0ymm13\0ymm14\0ymm15\0";

const char*
ll_reg_name(LLReg reg)
{
    int max;
    const char* table;

    switch (reg.rt) {
    case LL_RT_GP8:     max = 16; table = names_reg_gp8; break;
    case LL_RT_GP8Leg:  max = 16; table = names_reg_gp8leg; break;
    case LL_RT_GP16:    max = 16; table = names_reg_gp16; break;
    case LL_RT_GP32:    max = 16; table = names_reg_gp32; break;
    case LL_RT_GP64:    max = 16; table = names_reg_gp64; break;
    case LL_RT_IP:      max = 1;  table = "rip"; break;
    case LL_RT_XMM:     max = 16; table = names_reg_vec128; break;
    case LL_RT_YMM:     max = 16; table = names_reg_vec256; break;
    default:            max = 1;  table = "(unk)"; break;
    }

    return reg.ri < max ? table + (reg.ri * 6) : "(inv)";
}

size_t
ll_reg_size(LLReg reg)
{
    switch (reg.rt) {
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


/**
 * @}
 **/
