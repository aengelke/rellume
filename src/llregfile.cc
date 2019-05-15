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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <llvm-c/Core.h>

#include <llregfile-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llflags-internal.h>
#include <llinstr-internal.h>
#include <llinstruction-internal.h>

/**
 * \defgroup LLRegFile Register File
 * \brief Representation of a register file
 *
 * @{
 **/

struct LLRegister {
    LLVMValueRef facets[FACET_COUNT];
};

typedef struct LLRegister LLRegister;

struct LLRegisterFile {
    LLVMBasicBlockRef llvm_block;

    /**
     * \brief The LLVM values of the architectural general purpose registers
     *
     * The registers always store integers with 64 bits length.
     **/
    LLRegister gpRegisters[LL_RI_GPMax];

    /**
     * \brief The LLVM values of the SSE registers
     *
     * The vector length depends on #LL_VECTOR_REGISTER_SIZE.
     **/
    LLRegister sseRegisters[LL_RI_XMMMax];

    /**
     * \brief The LLVM values of the architectural general purpose registers
     **/
    LLVMValueRef flags[RFLAG_Max];

    /**
     * \brief The LLVM value of the current instruction address
     **/
    LLRegister ipRegister;

    /**
     * \brief The flag cache
     **/
    LLFlagCache flagCache;
};


LLVMTypeRef
ll_register_facet_type(RegisterFacet facet, LLVMContextRef ctx)
{
    LLVMTypeRef i8 = LLVMInt8TypeInContext(ctx);
    LLVMTypeRef i16 = LLVMInt16TypeInContext(ctx);
    LLVMTypeRef i32 = LLVMInt32TypeInContext(ctx);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(ctx);
    LLVMTypeRef f32 = LLVMFloatTypeInContext(ctx);
    LLVMTypeRef f64 = LLVMDoubleTypeInContext(ctx);

    switch (facet)
    {
        case FACET_I8: return i8;
        case FACET_I8H: return i8;
        case FACET_I16: return i16;
        case FACET_I32: return i32;
        case FACET_I64: return i64;
        case FACET_I128: return LLVMIntTypeInContext(ctx, 128);
        case FACET_I256: return LLVMIntTypeInContext(ctx, 256);
        case FACET_F32: return f32;
        case FACET_F64: return f64;
        case FACET_V16I8: return LLVMVectorType(i8, 16);
        case FACET_V8I16: return LLVMVectorType(i16, 8);
        case FACET_V4I32: return LLVMVectorType(i32, 4);
        case FACET_V2I64: return LLVMVectorType(i64, 2);
        case FACET_V2F32: return LLVMVectorType(f32, 2);
        case FACET_V4F32: return LLVMVectorType(f32, 4);
        case FACET_V2F64: return LLVMVectorType(f64, 2);
#if LL_VECTOR_REGISTER_SIZE >= 256
        case FACET_V32I8: return LLVMVectorType(i8, 32);
        case FACET_V16I16: return LLVMVectorType(i16, 16);
        case FACET_V8I32: return LLVMVectorType(i32, 8);
        case FACET_V4I64: return LLVMVectorType(i64, 4);
        case FACET_V8F32: return LLVMVectorType(f32, 8);
        case FACET_V4F64: return LLVMVectorType(f64, 4);
#endif
        case FACET_PTR: return LLVMPointerType(LLVMInt8TypeInContext(ctx), 0);
        case FACET_COUNT:
        default:
            warn_if_reached();
    }

    return NULL;
}

static const char*
ll_register_name_for_facet(RegisterFacet facet, LLReg reg)
{
    if (regIsGP(reg))
    {
        switch (facet)
        {
            case FACET_I8: return ll_reg_name(ll_reg(LL_RT_GP8, reg.ri));
            case FACET_I8H:
                if (reg.rt == LL_RT_GP8Leg)
                    return ll_reg_name(reg);
                else if (reg.ri < 4)
                    return ll_reg_name(ll_reg(LL_RT_GP8Leg, reg.ri + LL_RI_AH));
                else
                    return "XXX";
            case FACET_I16: return ll_reg_name(ll_reg(LL_RT_GP16, reg.ri));
            case FACET_I32: return ll_reg_name(ll_reg(LL_RT_GP32, reg.ri));
            case FACET_I64:
            case FACET_PTR:
            case FACET_I128:
            case FACET_I256:
            case FACET_F32:
            case FACET_F64:
            case FACET_V16I8:
            case FACET_V8I16:
            case FACET_V4I32:
            case FACET_V2I64:
            case FACET_V2F32:
            case FACET_V4F32:
            case FACET_V2F64:
    #if LL_VECTOR_REGISTER_SIZE >= 256
            case FACET_V32I8:
            case FACET_V16I16:
            case FACET_V8I32:
            case FACET_V4I64:
            case FACET_V8F32:
            case FACET_V4F64:
    #endif
                return ll_reg_name(ll_reg(LL_RT_GP64, reg.ri));
            case FACET_COUNT:
            default:
                warn_if_reached();
        }
    }
    else if (regIsV(reg))
    {
        switch (facet)
        {
            case FACET_I8:
            case FACET_I8H:
            case FACET_I16:
            case FACET_I32:
            case FACET_I64:
            case FACET_PTR:
            case FACET_I128:
            case FACET_F32:
            case FACET_F64:
            case FACET_V16I8:
            case FACET_V8I16:
            case FACET_V4I32:
            case FACET_V2I64:
            case FACET_V2F32:
            case FACET_V4F32:
            case FACET_V2F64:
                return ll_reg_name(ll_reg(LL_RT_XMM, reg.ri));
            case FACET_I256:
    #if LL_VECTOR_REGISTER_SIZE >= 256
            case FACET_V32I8:
            case FACET_V16I16:
            case FACET_V8I32:
            case FACET_V4I64:
            case FACET_V8F32:
            case FACET_V4F64:
    #endif
                return ll_reg_name(ll_reg(LL_RT_YMM, reg.ri));
            case FACET_COUNT:
            default:
                warn_if_reached();
        }
    }

    return ll_reg_name(reg);
}

LLRegisterFile*
ll_regfile_new(LLVMBasicBlockRef llvm_block)
{
    LLRegisterFile* regfile = (LLRegisterFile*) malloc(sizeof(LLRegisterFile));
    regfile->llvm_block = llvm_block;
    regfile->flagCache.valid = false;

    return regfile;
}

void
ll_regfile_dispose(LLRegisterFile* regfile)
{
    free(regfile);
}

/**
 * Get a pointer for a given register in the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param reg The register
 * \returns A pointer to the LLVM value in the register file.
 **/
static LLRegister*
ll_regfile_get_ptr(LLRegisterFile* regfile, LLReg reg)
{
    switch (reg.rt)
    {
        case LL_RT_GP8:
        case LL_RT_GP16:
        case LL_RT_GP32:
        case LL_RT_GP64:
            return &regfile->gpRegisters[reg.ri];
        case LL_RT_GP8Leg:
            if (ll_reg_high(reg))
                return &regfile->gpRegisters[reg.ri - LL_RI_AH];
            return &regfile->gpRegisters[reg.ri];
        case LL_RT_XMM:
        case LL_RT_YMM:
            return &regfile->sseRegisters[reg.ri];
        case LL_RT_IP:
            return &regfile->ipRegister;
        case LL_RT_X87:
        case LL_RT_MMX:
        case LL_RT_ZMM:
        case LL_RT_Max:
        case LL_RT_None:
        default:
            warn_if_reached();
    }

    return NULL;
}

/**
 * Get a register value of the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param reg The register
 * \returns The register value in the given facet
 **/
LLVMValueRef
ll_regfile_get(LLRegisterFile* regfile, RegisterFacet facet, LLReg reg, LLState* state)
{
    LLRegister* regFileEntry = ll_regfile_get_ptr(regfile, reg);
    LLVMTypeRef facetType = ll_register_facet_type(facet, state->context);
    LLVMValueRef value = regFileEntry->facets[facet];

    if (value != NULL)
    {
        if (LLVMTypeOf(value) != facetType)
            warn_if_reached();

        return value;
    }

    LLVMValueRef terminator = LLVMGetBasicBlockTerminator(regfile->llvm_block);
    if (terminator != NULL)
        LLVMPositionBuilderBefore(state->builder, terminator);
    else
        LLVMPositionBuilderAtEnd(state->builder, regfile->llvm_block);

    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);
    LLVMTypeRef i128 = LLVMIntTypeInContext(state->context, 128);
    LLVMTypeRef pi8 = LLVMPointerType(i8, 0);

    if (regIsGP(reg) || reg.rt == LL_RT_IP)
    {
        LLVMValueRef native = regFileEntry->facets[FACET_I64];

        switch (facet)
        {
            case FACET_PTR:
                value = LLVMBuildIntToPtr(state->builder, native, pi8, "");
                break;
            case FACET_I8:
            case FACET_I16:
            case FACET_I32:
                value = LLVMBuildTrunc(state->builder, native, facetType, "");
                break;
            case FACET_I8H:
                value = LLVMBuildLShr(state->builder, native, LLVMConstInt(LLVMTypeOf(native), 8, false), "");
                value = LLVMBuildTrunc(state->builder, value, i8, "");
                break;
            case FACET_I64:
            case FACET_I128:
            case FACET_I256:
            case FACET_F32:
            case FACET_F64:
            case FACET_V2F32:
            case FACET_V16I8:
            case FACET_V8I16:
            case FACET_V4I32:
            case FACET_V2I64:
            case FACET_V4F32:
            case FACET_V2F64:
#if LL_VECTOR_REGISTER_SIZE >= 256
            case FACET_V32I8:
            case FACET_V16I16:
            case FACET_V8I32:
            case FACET_V4I64:
            case FACET_V8F32:
            case FACET_V4F64:
#endif
            case FACET_COUNT:
            default:
                value = LLVMGetUndef(ll_register_facet_type(facet, state->context));
        }
    }
    else if (regIsV(reg))
    {
        int targetBits = 0;

        switch (facet)
        {
            case FACET_I8:
                value = ll_regfile_get(regfile, FACET_V16I8, reg, state);
                value = LLVMBuildExtractElement(state->builder, value, LLVMConstInt(i32, 0, false), "");
                break;
            case FACET_I16:
                value = ll_regfile_get(regfile, FACET_V8I16, reg, state);
                value = LLVMBuildExtractElement(state->builder, value, LLVMConstInt(i32, 0, false), "");
                break;
            case FACET_I32:
                value = ll_regfile_get(regfile, FACET_V4I32, reg, state);
                value = LLVMBuildExtractElement(state->builder, value, LLVMConstInt(i32, 0, false), "");
                break;
            case FACET_I64:
                value = ll_regfile_get(regfile, FACET_V2I64, reg, state);
                value = LLVMBuildExtractElement(state->builder, value, LLVMConstInt(i32, 0, false), "");
                break;
            case FACET_F32:
                value = ll_regfile_get(regfile, FACET_V4F32, reg, state);
                value = LLVMBuildExtractElement(state->builder, value, LLVMConstInt(i32, 0, false), "");
                break;
            case FACET_F64:
                value = ll_regfile_get(regfile, FACET_V2F64, reg, state);
                value = LLVMBuildExtractElement(state->builder, value, LLVMConstInt(i32, 0, false), "");
                break;
            case FACET_V2F32:
                targetBits = 64;
                break;
            case FACET_V16I8:
            case FACET_V8I16:
            case FACET_V4I32:
            case FACET_V2I64:
            case FACET_V4F32:
            case FACET_V2F64:
                targetBits = 128;
                break;
#if LL_VECTOR_REGISTER_SIZE >= 256
            case FACET_V32I8:
            case FACET_V16I16:
            case FACET_V8I32:
            case FACET_V4I64:
            case FACET_V8F32:
            case FACET_V4F64:
                targetBits = 256;
                break;
#endif
            case FACET_I128:
                // TODO: Try to induce from other 128-bit facets first
                value = LLVMBuildTruncOrBitCast(state->builder, regFileEntry->facets[FACET_IVEC], i128, "");
                break;
            case FACET_PTR:
            case FACET_I8H:
            case FACET_I256:
            case FACET_COUNT:
            default:
                value = LLVMGetUndef(ll_register_facet_type(facet, state->context));
        }

        // Its a vector.
#if LL_VECTOR_REGISTER_SIZE >= 256
        if (value == NULL && targetBits == 128 && regFileEntry->facets[FACET_I128] != NULL)
        {
            LLVMValueRef native = regFileEntry->facets[FACET_I128];
            value = LLVMBuildBitCast(state->builder, native, facetType, "");
        }
#endif
        if (value == NULL)
        {
            LLVMValueRef native = regFileEntry->facets[FACET_IVEC];

            int targetCount = LLVMGetVectorSize(facetType);
            int nativeCount = targetCount * LL_VECTOR_REGISTER_SIZE / targetBits;

            LLVMTypeRef elementType = LLVMGetElementType(facetType);
            LLVMTypeRef nativeVectorType = LLVMVectorType(elementType, nativeCount);

            value = LLVMBuildBitCast(state->builder, native, nativeVectorType, "");

            if (nativeCount > targetCount)
            {
                LLVMValueRef maskElements[targetCount];
                for (int i = 0; i < targetCount; i++)
                    maskElements[i] = LLVMConstInt(i32, i, false);

                LLVMValueRef mask = LLVMConstVector(maskElements, targetCount);
                value = LLVMBuildShuffleVector(state->builder, value, LLVMGetUndef(nativeVectorType), mask, "");
            }
        }
    }

    if (value == NULL || LLVMTypeOf(value) != facetType)
        warn_if_reached();

    regFileEntry->facets[facet] = value;

    return value;
}

/**
 * Clear a register to undefined of the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param reg The register
 * \param value The new value
 **/
void
ll_regfile_clear(LLRegisterFile* regfile, LLReg reg, LLVMContextRef ctx)
{
    LLRegister* regFileEntry = ll_regfile_get_ptr(regfile, reg);

    for (size_t i = 0; i < FACET_COUNT; i++)
        regFileEntry->facets[i] = LLVMGetUndef(ll_register_facet_type((RegisterFacet) i, ctx));
}

/**
 * Set a register in all facets to zero within the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param reg The name of the new register
 * \param state The state
 **/
void
ll_regfile_zero(LLRegisterFile* regfile, LLReg reg, LLVMContextRef ctx)
{
    LLRegister* regFileEntry = ll_regfile_get_ptr(regfile, reg);

    for (size_t i = 0; i < FACET_COUNT; i++)
        regFileEntry->facets[i] = LLVMConstNull(ll_register_facet_type((RegisterFacet) i, ctx));
}

/**
 * Rename a register to another register of the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param reg The name of the new register
 * \param current The name of the current register
 * \param state The state
 **/
void
ll_regfile_rename(LLRegisterFile* regfile, LLReg reg, LLReg current)
{
    LLRegister* regFileEntry1 = ll_regfile_get_ptr(regfile, reg);
    LLRegister* regFileEntry2 = ll_regfile_get_ptr(regfile, current);

    memcpy(regFileEntry1, regFileEntry2, sizeof(LLRegister));
}

/**
 * Set a register value of the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param reg The register
 * \param value The new value
 **/
void
ll_regfile_set(LLRegisterFile* regfile, RegisterFacet facet, LLReg reg, LLVMValueRef value, bool clearOthers, LLState* state)
{
    if (!LLVMIsConstant(value))
    {
        char buffer[20];
        int len = snprintf(buffer, sizeof(buffer), "asm.reg.%s", ll_register_name_for_facet(facet, reg));
        LLVMSetMetadata(value, LLVMGetMDKindIDInContext(state->context, buffer, len), state->emptyMD);
    }

    if (LLVMTypeOf(value) != ll_register_facet_type(facet, state->context))
        warn_if_reached();

    LLRegister* regFileEntry = ll_regfile_get_ptr(regfile, reg);

    if (clearOthers)
    {
        for (size_t i = 0; i < FACET_COUNT; i++)
            regFileEntry->facets[i] = NULL;

        if (regIsGP(reg) && facet != FACET_I64)
        {
            if (facet != FACET_PTR)
                warn_if_reached();

            LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
            regFileEntry->facets[FACET_I64] = LLVMBuildPtrToInt(state->builder, value, i64, "");
        }
        else if (regIsV(reg) && facet != FACET_IVEC)
            warn_if_reached();
    }

    regFileEntry->facets[facet] = value;
}

/**
 * Get a flag value of the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param flag The flag
 * \returns The current flag value
 **/
LLVMValueRef
ll_regfile_get_flag(LLRegisterFile* regfile, int flag)
{
    return regfile->flags[flag];
}

/**
 * Set a flag value of the register file.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \param flag The flag
 * \param value The new value
 **/
void
ll_regfile_set_flag(LLRegisterFile* regfile, int flag, LLVMValueRef value)
{
    regfile->flags[flag] = value;
}

/**
 * Get the flag cache.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param regfile The register file
 * \returns The flag cache
 **/
LLFlagCache*
ll_regfile_get_flag_cache(LLRegisterFile* regfile)
{
    return &regfile->flagCache;
}

/**
 * @}
 **/
