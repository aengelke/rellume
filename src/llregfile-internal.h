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

#ifndef LL_REGFILE_H
#define LL_REGFILE_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <llvm-c/Core.h>

#include <llcommon-internal.h>
#include <llinstr-internal.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup LLRegFile
 **/
enum {
    /**
     * \brief The zero flag
     **/
    RFLAG_ZF = 0,
    /**
     * \brief The sign flag
     **/
    RFLAG_SF,
    /**
     * \brief The parity flag
     **/
    RFLAG_PF,
    /**
     * \brief The carry flag
     **/
    RFLAG_CF,
    /**
     * \brief The overflow flag
     **/
    RFLAG_OF,
    /**
     * \brief The auxiliary carry flag
     **/
    RFLAG_AF,
    RFLAG_Max
};

/**
 * \ingroup LLRegFile
 * \brief Flag cache storing additional information about the flag register
 **/
struct LLFlagCache {
    /**
     * \brief Whether the information is valid
     **/
    bool valid;
    /**
     * \brief The first operand of the subtraction
     **/
    LLVMValueRef operand1;
    /**
     * \brief The second operand of the subtraction
     **/
    LLVMValueRef operand2;
    /**
     * \brief The result of the subtraction
     **/
    LLVMValueRef result;
};

typedef struct LLFlagCache LLFlagCache;

/**
 * \ingroup LLRegFile
 **/
enum RegisterFacet {
    FACET_PTR,
    FACET_I8,
    FACET_I8H,
    FACET_I16,
    FACET_I32,
    FACET_I64,
    FACET_I128,
    FACET_I256,
    FACET_F32,
    FACET_F64,

    FACET_V2F32,

    FACET_V16I8,
    FACET_V8I16,
    FACET_V4I32,
    FACET_V2I64,
    FACET_V4F32,
    FACET_V2F64,

#if LL_VECTOR_REGISTER_SIZE >= 256
    FACET_V32I8,
    FACET_V16I16,
    FACET_V8I32,
    FACET_V4I64,
    FACET_V8F32,
    FACET_V4F64,
#endif

    FACET_COUNT,
};

#if LL_VECTOR_REGISTER_SIZE == 128
#define FACET_IVEC FACET_I128
#define FACET_VI8 FACET_V16I8
#define FACET_VI16 FACET_V8I16
#define FACET_VI32 FACET_V4I32
#define FACET_VI64 FACET_V2I64
#define FACET_VF32 FACET_V4F32
#define FACET_VF64 FACET_V2F64
#elif LL_VECTOR_REGISTER_SIZE == 256
#define FACET_IVEC FACET_I256
#define FACET_VI8 FACET_V32I8
#define FACET_VI16 FACET_V16I16
#define FACET_VI32 FACET_V8I32
#define FACET_VI64 FACET_V4I64
#define FACET_VF32 FACET_V8F32
#define FACET_VF64 FACET_V4F64
#endif

typedef enum RegisterFacet RegisterFacet;


LLRegisterFile* ll_regfile_new(LLBasicBlock*);
void ll_regfile_dispose(LLRegisterFile*);
LLVMValueRef ll_regfile_get(LLRegisterFile*, RegisterFacet, LLReg, LLState*);
void ll_regfile_clear(LLRegisterFile*, LLReg, LLState*);
void ll_regfile_zero(LLRegisterFile*, LLReg, LLState*);
void ll_regfile_rename(LLRegisterFile*, LLReg, LLReg, LLState*);
void ll_regfile_set(LLRegisterFile*, RegisterFacet, LLReg, LLVMValueRef, bool, LLState*);
LLVMValueRef ll_regfile_get_flag(LLRegisterFile*, int);
void ll_regfile_set_flag(LLRegisterFile*, int, LLVMValueRef);
LLFlagCache* ll_regfile_get_flag_cache(LLRegisterFile*);

LLVMTypeRef ll_register_facet_type(RegisterFacet, LLState*);

#ifdef __cplusplus
}
#endif

#endif
