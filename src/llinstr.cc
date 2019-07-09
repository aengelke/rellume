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

#include <rellume/instr.h>

#include <llcommon-internal.h>


/**
 * \defgroup LLInstr Instruction
 * \brief Representation of an instruction
 *
 * @{
 **/

const char*
LLReg::Name() const
{
    int max;
    const char (* table)[6];

#define TABLE(name, ...) \
        case name : { \
            static const char tab[][6] = { __VA_ARGS__ }; \
            table = tab; max = sizeof(tab) / sizeof(*tab); break; }

    switch (rt) {
    default: return "(unk)";

    TABLE(LL_RT_GP8,
        "al","cl","dl","bl","spl","bpl","sil","dil",
        "r8b","r9b","r10b","r11b","r12b","r13b","r14b","r15b")
    TABLE(LL_RT_GP8Leg,
        "al","cl","dl","bl","ah","ch","dh","bh",
        "r8b","r9b","r10b","r11b","r12b","r13b","r14b","r15b")
    TABLE(LL_RT_GP16,
        "ax","cx","dx","bx","sp","bp","si","di",
        "r8w","r9w","r10w","r11w","r12w","r13w","r14w","r15w")
    TABLE(LL_RT_GP32,
        "eax","ecx","edx","ebx","esp","ebp","esi","edi",
        "r8d","r9d","r10d","r11d","r12d","r13d","r14d","r15d")
    TABLE(LL_RT_GP64,
        "rax","rcx","rdx","rbx","rsp","rbp","rsi","rdi",
        "r8","r9","r10","r11","r12","r13","r14","r15")
    TABLE(LL_RT_IP, "rip")
    TABLE(LL_RT_XMM,
        "xmm0","xmm1","xmm2","xmm3","xmm4","xmm5","xmm6","xmm7",
        "xmm8","xmm9","xmm10","xmm11","xmm12","xmm13","xmm14","xmm15")
    TABLE(LL_RT_YMM,
        "ymm0","ymm1","ymm2","ymm3","ymm4","ymm5","ymm6","ymm7",
        "ymm8","ymm9","ymm10","ymm11","ymm12","ymm13","ymm14","ymm15")

    }

    return ri < max ? table[ri] : "(inv)";
}

size_t
LLReg::Size() const
{
    switch (rt) {
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
