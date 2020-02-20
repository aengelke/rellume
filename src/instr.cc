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

#include "rellume/instr.h"
#include "instr.h"

#include <fadec.h>
#include <cstdint>
#include <cstdio>
#include <cstring>


/**
 * \defgroup LLInstr Instruction
 * \brief Representation of an instruction
 *
 * @{
 **/

namespace rellume {

bool Instr::BreaksAlways() const {
    switch (type()) {
    case FDI_RET:
    case FDI_JMP:
    case FDI_CALL:
    case FDI_SYSCALL:
    case FDI_INT:
    case FDI_INT3:
    case FDI_INTO:
    case FDI_UD0:
    case FDI_UD1:
    case FDI_UD2:
    case FDI_HLT:
        return true;
    default:
        return false;
    }
}

bool Instr::BreaksConditionally() const {
    switch (type()) {
    case FDI_JO:
    case FDI_JNO:
    case FDI_JC:
    case FDI_JNC:
    case FDI_JZ:
    case FDI_JNZ:
    case FDI_JBE:
    case FDI_JA:
    case FDI_JS:
    case FDI_JNS:
    case FDI_JP:
    case FDI_JNP:
    case FDI_JL:
    case FDI_JGE:
    case FDI_JLE:
    case FDI_JG:
    case FDI_JCXZ:
    case FDI_LOOP:
    case FDI_LOOPZ:
    case FDI_LOOPNZ:
        return true;
    default:
        return false;
    }
}

bool Instr::HasAbsJumpTarget() const {
    switch (type()) {
    case FDI_JO:
    case FDI_JNO:
    case FDI_JC:
    case FDI_JNC:
    case FDI_JZ:
    case FDI_JNZ:
    case FDI_JBE:
    case FDI_JA:
    case FDI_JS:
    case FDI_JNS:
    case FDI_JP:
    case FDI_JNP:
    case FDI_JL:
    case FDI_JGE:
    case FDI_JLE:
    case FDI_JG:
    case FDI_JCXZ:
    case FDI_LOOP:
    case FDI_LOOPZ:
    case FDI_LOOPNZ:
        return true;
    case FDI_JMP:
    case FDI_CALL:
        return op(0).is_imm();
    default:
        return false;
    }
}

LLReg Instr::MapFdReg(unsigned idx, unsigned type) {
    if (idx == FD_REG_NONE)
        return LLReg{ LL_RT_None, LL_RI_None };
    if (type == FD_RT_GPL)
        return LLReg{ LL_RT_GP, (uint16_t) idx };
    if (type == FD_RT_GPH)
        return LLReg{ LL_RT_GP8High, (uint16_t) idx };
    if (type == FD_RT_VEC)
        return LLReg{ LL_RT_XMM, (uint16_t) idx };
    if (type == FD_RT_SEG)
        return LLReg{ LL_RT_SEG, (uint16_t) idx };
    if (type == FD_RT_BND)
        return LLReg{ LL_RT_BND, (uint16_t) idx };

    fprintf(stderr, "Unknown reg convert %d/%d\n", type, idx);
    return LLReg{ LL_RT_None, LL_RI_None };
}

}

/**
 * @}
 **/
