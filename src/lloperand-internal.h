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

#ifndef LL_OPERAND_H
#define LL_OPERAND_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <llvm-c/Core.h>

#include <llcommon-internal.h>
#include <llinstr-internal.h>
#include <llstate-internal.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup LLInstrOp
 * \brief The data type of an operand
 **/
enum OperandDataType {
    /**
     * \brief Single Integer, length chosen appropriately
     **/
    OP_SI,
    /**
     * \brief Single 32-bit Integer
     **/
    OP_SI32,
    /**
     * \brief Single 64-bit Integer
     **/
    OP_SI64,
    /**
     * \brief Vector of 8-bit integers
     **/
    OP_VI8,
    /**
     * \brief Vector of 32-bit integers
     **/
    OP_VI32,
    /**
     * \brief Vector of 8-bit integers
     **/
    OP_VI64,
    /**
     * \brief Single 32-bit Real
     **/
    OP_SF32,
    /**
     * \brief Single 64-bit Real
     **/
    OP_SF64,
    /**
     * \brief Vector of floats
     **/
    OP_VF32,
    OP_V1F32,
    OP_V2F32,
    OP_V4F32,
    /**
     * \brief Vector of doubles
     **/
    OP_VF64,
    OP_V1F64,
    OP_V2F64,
};

typedef enum OperandDataType OperandDataType;

/**
 * \ingroup LLInstrOp
 * \brief The alignment of an operand
 **/
enum Alignment {
    /**
     * \brief Maximum alignment based on data type size
     **/
    ALIGN_MAXIMUM = 0,
    /**
     * \brief 1-byte alignment
     **/
    ALIGN_1 = 1,
    /**
     * \brief 2-byte alignment
     **/
    ALIGN_2 = 2,
    /**
     * \brief 4-byte alignment
     **/
    ALIGN_4 = 4,
    /**
     * \brief 8-byte alignment
     **/
    ALIGN_8 = 8,
};

typedef enum Alignment Alignment;

/**
 * \ingroup LLInstrOp
 * \brief The handling when storing a partial register
 **/
enum PartialRegisterHandling {
    /**
     * \brief Default handling for general purpose registers
     *
     * For general purpose registers with a 32-bit operand the upper part is
     * zeroed, otherwise it is kept. For SSE registers, this is handling is not
     * allowed since there is no default (depending on VEX prefix).
     **/
    REG_DEFAULT,
    /**
     * \brief Zero the unused half of the SSE register.
     **/
    REG_ZERO_UPPER_SSE,
    /**
     * \brief Zero the unused half of the full SSE/AVX register.
     **/
    REG_ZERO_UPPER_AVX,
    /**
     * \brief Keep the unused half. This might produce less optimizable code.
     **/
    REG_KEEP_UPPER
};

typedef enum PartialRegisterHandling PartialRegisterHandling;

LLVMValueRef ll_operand_get_address(OperandDataType, LLInstrOp*, LLState*);
LLVMValueRef ll_operand_load(OperandDataType, Alignment, LLInstrOp*, LLState*);
void ll_operand_store(OperandDataType, Alignment, LLInstrOp*, PartialRegisterHandling, LLVMValueRef, LLState*);
void ll_operand_construct_args(LLVMTypeRef, LLVMValueRef*, LLState*);

#ifdef __cplusplus
}
#endif

#endif
