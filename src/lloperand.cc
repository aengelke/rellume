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

#include <lloperand-internal.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llinstr-internal.h>
#include <llregfile-internal.h>
#include <llstate-internal.h>

/**
 * \defgroup LLInstrOp LLInstrOp
 * \brief Handling of instruction operands
 *
 * @{
 **/

/**
 * Infer the size of the operand.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param dataType The data type
 * \param operand The operand
 * \returns The bit length of the operand
 **/
static int
ll_operand_get_type_length(OperandDataType dataType, LLInstrOp* operand)
{
    int bits = -1;

    switch (dataType)
    {
        case OP_SI:
        case OP_VI8:
        case OP_VI64:
        case OP_VI32:
        case OP_VF32:
        case OP_VF64:
            bits = operand->size * 8;
            break;
        case OP_SI32:
        case OP_SF32:
        case OP_V1F32:
            bits = 32;
            break;
        case OP_SI64:
        case OP_SF64:
        case OP_V2F32:
        case OP_V1F64:
            bits = 64;
            break;
        case OP_V4F32:
        case OP_V2F64:
            bits = 128;
            break;
        default:
            warn_if_reached();
            break;

    }

    return bits;
}

static RegisterFacet
ll_operand_get_facet(OperandDataType dataType, LLInstrOp* operand)
{
    int bits = operand->size * 8;
    switch (dataType)
    {
        case OP_SI:
            if (bits == 8)
                return ll_reg_high(operand->reg) ? FACET_I8H : FACET_I8;
            if (bits == 16) return FACET_I16;
            if (bits == 32) return FACET_I32;
            if (bits == 64) return FACET_I64;
            warn_if_reached();
            break;
        case OP_SI32: return FACET_I32;
        case OP_SI64: return FACET_I64;
        case OP_VI8:
            if (bits == 128) return FACET_V16I8;
            warn_if_reached();
            break;
        case OP_VI32:
            if (bits == 128) return FACET_V4I32;
            warn_if_reached();
            break;
        case OP_VI64:
            if (bits == 128) return FACET_V2I64;
            warn_if_reached();
            break;
        case OP_VF32:
            if (bits == 32) return FACET_V1F32;
            if (bits == 64) return FACET_V2F32;
            if (bits == 128) return FACET_V4F32;
            warn_if_reached();
            break;
        case OP_V1F32: return FACET_V1F32;
        case OP_V2F32: return FACET_V2F32;
        case OP_V4F32: return FACET_V4F32;
        case OP_VF64:
            if (bits == 128) return FACET_V2F64;
            warn_if_reached();
            break;
        case OP_V1F64: return FACET_V1F64;
        case OP_V2F64: return FACET_V2F64;
        case OP_SF32: return FACET_F32;
        case OP_SF64: return FACET_F64;
        default:
            warn_if_reached();
    }

    return (RegisterFacet) INT32_MAX;
}

/**
 * Infer the LLVM type from the requested data type and the number of bits.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param dataType The data type
 * \param bits The number of bits
 * \param state The module state
 * \returns An LLVM type matching the data type and the number of bits
 **/
static LLVMTypeRef
ll_operand_get_type(OperandDataType dataType, int bits, LLState* state)
{
    LLVMTypeRef type = NULL;

    switch (dataType)
    {
        case OP_SI:
            type = LLVMIntTypeInContext(state->context, bits);
            break;
        case OP_SI32:
            type = LLVMInt32TypeInContext(state->context);
            break;
        case OP_SI64:
            type = LLVMInt64TypeInContext(state->context);
            break;
        case OP_VI8:
            if (bits % 8 == 0)
                type = LLVMVectorType(LLVMInt8TypeInContext(state->context), bits / 8);
            else
                warn_if_reached();
            break;
        case OP_VI64:
            if (bits % 64 == 0)
                type = LLVMVectorType(LLVMInt64TypeInContext(state->context), bits / 64);
            else
                warn_if_reached();
            break;
        case OP_VI32:
            if (bits % 32 == 0)
                type = LLVMVectorType(LLVMInt32TypeInContext(state->context), bits / 32);
            else
                warn_if_reached();
            break;
        case OP_SF32:
            type = LLVMFloatTypeInContext(state->context);
            break;
        case OP_SF64:
            type = LLVMDoubleTypeInContext(state->context);
            break;
        case OP_VF32:
            if (bits % 32 == 0)
                type = LLVMVectorType(LLVMFloatTypeInContext(state->context), bits / 32);
            else
                warn_if_reached();
            break;
        case OP_V1F32:
            return LLVMVectorType(LLVMFloatTypeInContext(state->context), 1);
        case OP_V2F32:
            return LLVMVectorType(LLVMFloatTypeInContext(state->context), 2);
        case OP_V4F32:
            return LLVMVectorType(LLVMFloatTypeInContext(state->context), 4);
        case OP_VF64:
            if (bits % 64 == 0)
                type = LLVMVectorType(LLVMDoubleTypeInContext(state->context), bits / 64);
            else
                warn_if_reached();
            break;
        case OP_V1F64:
            return LLVMVectorType(LLVMDoubleTypeInContext(state->context), 1);
        case OP_V2F64:
            return LLVMVectorType(LLVMDoubleTypeInContext(state->context), 2);
        default:
            warn_if_reached();
            break;
    }

    return type;
}

/**
 * Store a value in a general purpose register.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param value The value to cast
 * \param dataType The data type
 * \param operand The register operand
 * \param state The module state
 **/
static void
ll_operand_store_gp(LLVMValueRef value, OperandDataType dataType, LLInstrOp* operand, LLState* state)
{
    int operandWidth = ll_operand_get_type_length(dataType, operand);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
    LLVMTypeRef operandIntType = LLVMIntTypeInContext(state->context, operandWidth);

    LLVMValueRef result = NULL;

    value = LLVMBuildSExtOrBitCast(state->builder, value, operandIntType, "");
    LLVMValueRef value64 = LLVMBuildZExtOrBitCast(state->builder, value, i64, "");
    if (operand->reg.rt == LL_RT_GP32 || operand->reg.rt == LL_RT_GP64)
        result = value64;
    else
    {
        uint64_t mask = 0;
        if (ll_reg_high(operand->reg))
        {
            mask = 0xff00;
            value64 = LLVMBuildShl(state->builder, value64, LLVMConstInt(i64, 8, false), "");
        }
        else if (operand->reg.rt == LL_RT_GP8 || operand->reg.rt == LL_RT_GP8Leg)
            mask = 0xff;
        else if (operand->reg.rt == LL_RT_GP16)
            mask = 0xffff;
        else
            warn_if_reached();

        LLVMValueRef current = ll_get_register(operand->reg, FACET_I64, state);
        LLVMValueRef masked = LLVMBuildAnd(state->builder, current, LLVMConstInt(i64, ~mask, false), "");
        result = LLVMBuildOr(state->builder, masked, value64, "");
    }

    ll_set_register(operand->reg, FACET_I64, result, true, state);
    if (operand->reg.rt == LL_RT_GP32)
        ll_set_register(operand->reg, FACET_I32, value, false, state);
    if (operand->reg.rt == LL_RT_GP16)
        ll_set_register(operand->reg, FACET_I16, value, false, state);
    if (operand->reg.rt == LL_RT_GP8)
        ll_set_register(operand->reg, FACET_I8, value, false, state);
    if (operand->reg.rt == LL_RT_GP8Leg)
        ll_set_register(operand->reg, FACET_I8H, value, false, state);
}

/**
 * Store a value in a vector (SSE/AVX) register.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param value The value to cast
 * \param dataType The data type
 * \param operand The register operand
 * \param zeroHandling Handling of unused upper parts of the register
 * \param state The module state
 **/
static void
ll_operand_store_vreg(LLVMValueRef value, OperandDataType dataType, LLInstrOp* operand, PartialRegisterHandling zeroHandling, LLState* state)
{
    LLVMTypeRef i32 = LLVMInt32TypeInContext(state->context);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);

    int operandWidth = ll_operand_get_type_length(dataType, operand);
    LLVMValueRef result = NULL;

    LLVMTypeRef iVec = LLVMIntTypeInContext(state->context, LL_VECTOR_REGISTER_SIZE);
    LLVMTypeRef i128 = LLVMIntTypeInContext(state->context, 128);

    LLVMValueRef current = ll_get_register(operand->reg, FACET_IVEC, state);
    if (zeroHandling == REG_ZERO_UPPER_AVX)
    {
        current = LLVMConstNull(iVec);
    }
    else if (zeroHandling == REG_ZERO_UPPER_SSE)
    {
        // Ehem, we have to construct the mask first.
        // It is all-ones with the lowest 128-bit being zero.
        LLVMValueRef mask = LLVMConstNot(LLVMConstZExtOrBitCast(LLVMConstAllOnes(i128), iVec));
        current = LLVMBuildAnd(state->builder, current, mask, "");
    }

#if LL_VECTOR_REGISTER_SIZE >= 256
    LLVMValueRef current128 = ll_get_register(operand->reg, FACET_I128, state);
    if (zeroHandling == REG_ZERO_UPPER_AVX || zeroHandling == REG_ZERO_UPPER_SSE)
        current128 = LLVMConstNull(i128);
#endif

    LLVMTypeRef valueType = LLVMTypeOf(value);
    bool valueIsVector = LLVMGetTypeKind(valueType) == LLVMVectorTypeKind;

    int elementCount = valueIsVector ? LLVMGetVectorSize(valueType) : 1;
    int totalCount = elementCount * LL_VECTOR_REGISTER_SIZE / operandWidth;

    if (valueIsVector)
    {
        LLVMTypeRef vectorType = LLVMVectorType(LLVMGetElementType(valueType), totalCount);

        if (totalCount == elementCount)
        {
            result = value;
        }
        else
        {
            LLVMValueRef vectorCurrent = LLVMBuildBitCast(state->builder, current, vectorType, "");
            LLVMValueRef maskElements[totalCount];
            for (int i = 0; i < totalCount; i++)
                maskElements[i] = LLVMConstInt(i32, i, false);
            for (int i = elementCount; i < totalCount; i++)
                maskElements[i] = LLVMConstInt(i32, elementCount, false);
            LLVMValueRef mask = LLVMConstVector(maskElements, totalCount);
            LLVMValueRef enlarged = LLVMBuildShuffleVector(state->builder, value, LLVMConstNull(valueType), mask, "");

            for (int i = elementCount; i < totalCount; i++)
                maskElements[i] = LLVMConstInt(i32, totalCount + i, false);
            mask = LLVMConstVector(maskElements, totalCount);
            result = LLVMBuildShuffleVector(state->builder, enlarged, vectorCurrent, mask, "");
        }

        result = LLVMBuildBitCast(state->builder, result, iVec, "");
        ll_set_register(operand->reg, FACET_IVEC, result, true, state);

#if LL_VECTOR_REGISTER_SIZE >= 256
        // Induce some common facets via i128 for better SSE support
        if (operandWidth == 128)
        {
            LLVMValueRef sseReg = LLVMBuildBitCast(state->builder, value, i128, "");
            ll_set_register(operand->reg, FACET_I128, sseReg, false, state);
        }
#endif
    }
    else
    {
        LLVMTypeRef vectorType = LLVMVectorType(valueType, totalCount);
        LLVMValueRef vectorCurrent = LLVMBuildBitCast(state->builder, current, vectorType, "");

        LLVMValueRef constZero = LLVMConstInt(i64, 0, false);
        result = LLVMBuildInsertElement(state->builder, vectorCurrent, value, constZero, "");
        result = LLVMBuildBitCast(state->builder, result, iVec, "");

        ll_set_register(operand->reg, FACET_IVEC, result, true, state);

#if LL_VECTOR_REGISTER_SIZE >= 256
        // Induce some common facets via i128 for better SSE support
        LLVMTypeRef vectorType128 = LLVMVectorType(valueType, 128 / operandWidth);
        LLVMValueRef vectorCurrent128 = LLVMBuildBitCast(state->builder, current128, vectorType128, "");

        LLVMValueRef sseReg = LLVMBuildInsertElement(state->builder, vectorCurrent128, value, constZero, "");
        sseReg = LLVMBuildBitCast(state->builder, sseReg, i128, "");
        ll_set_register(operand->reg, FACET_I128, sseReg, false, state);
#endif
    }
}

/**
 * Get a pointer to the a known global constant
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param constGlobal The constant global address
 * \param state The module state
 * \returns A pointer of the given type which represents the address
 **/
static LLVMValueRef
ll_get_const_pointer(uintptr_t ptr, LLState* state)
{
    LLVMTypeRef i8 = LLVMInt8TypeInContext(state->context);
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);
    LLVMTypeRef pi8 = LLVMPointerType(i8, 0);

    if (ptr == 0)
        return LLVMConstPointerNull(pi8);

    // if (state->globalOffsetBase == 0)
    // {
    //     state->globalOffsetBase = ptr;
    //     state->globalBase = LLVMAddGlobal(state->module, i8, "__ll_global_base__");
    //     LLVMAddGlobalMapping(state->engine, state->globalBase, (void*) ptr);
    // }

    LLVMValueRef pointer;
    if (state->cfg.globalBase != NULL)
    {
        uintptr_t offset = ptr - state->cfg.globalOffsetBase;
        LLVMValueRef llvmOffset = LLVMConstInt(i64, offset, false);
        pointer = LLVMBuildGEP(state->builder, state->cfg.globalBase, &llvmOffset, 1, "");
    }
    else
    {
        LLVMValueRef llvmOffset = LLVMConstInt(i64, ptr, false);
        pointer = LLVMBuildIntToPtr(state->builder, llvmOffset, pi8, "");
    }

    return pointer;
}

/**
 * Get the pointer corresponding to an operand.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param dataType The data type used to create the appropriate pointer type
 * \param operand The operand, must be Ind32 or Ind64
 * \param state The module state
 * \returns The pointer which corresponds to the operand
 **/
LLVMValueRef
ll_operand_get_address(OperandDataType dataType, LLInstrOp* operand, LLState* state)
{
    LLVMValueRef result;
    LLVMTypeRef i64 = LLVMInt64TypeInContext(state->context);

    LLVMTypeRef pointerType;
    int addrspace;
    int bits = ll_operand_get_type_length(dataType, operand);

    switch (operand->seg)
    {
        case LL_RI_None:
            addrspace = 0;
            break;
        case LL_RI_GS:
            addrspace = 256;
            break;
        case LL_RI_FS:
            addrspace = 257;
            break;
        default:
            addrspace = 0;
            warn_if_reached();
            break;
    }

    pointerType = LLVMPointerType(ll_operand_get_type(dataType, bits, state), addrspace);

    if (operand->reg.rt != LL_RT_None)
    {
        result = ll_get_register(operand->reg, FACET_PTR, state);

        if (LLVMIsConstant(result))
        {
            result = ll_get_register(operand->reg, FACET_I64, state);

            if (!LLVMIsConstant(result))
                warn_if_reached();

            uintptr_t constPtr = LLVMConstIntGetZExtValue(result);
            result = ll_get_const_pointer(constPtr + operand->val, state);
        }
        else if (operand->val != 0)
        {
            LLVMValueRef offset = LLVMConstInt(i64, operand->val, false);

            if (operand->scale != 0 && (operand->val % operand->scale) == 0)
            {
                LLVMTypeRef scaleType = LLVMPointerType(LLVMIntTypeInContext(state->context, operand->scale * 8), 0);
                result = LLVMBuildPointerCast(state->builder, result, scaleType, "");
                offset = LLVMConstInt(i64, operand->val / operand->scale, false);
            }

            result = LLVMBuildGEP(state->builder, result, &offset, 1, "");
        }
    }
    else
    {
        result = ll_get_const_pointer(operand->val, state);
    }

    if (operand->scale != 0)
    {
        LLVMValueRef offset = ll_get_register(operand->ireg, FACET_I64, state);

        if (LLVMIsNull(result))
        {
            // Fallback to inttoptr if this is definitly not-a-pointer.
            // Therefore, we don't need to use ll_get_const_pointer.
            offset = LLVMBuildMul(state->builder, offset, LLVMConstInt(i64, operand->scale, false), "");
            result = LLVMBuildIntToPtr(state->builder, offset, pointerType, "");
        }
        else
        {
            LLVMTypeRef scaleType = LLVMPointerType(LLVMIntTypeInContext(state->context, operand->scale * 8), 0);

            if (operand->scale * 8 == bits)
                scaleType = pointerType;

            result = LLVMBuildPointerCast(state->builder, result, scaleType, "");
            result = LLVMBuildGEP(state->builder, result, &offset, 1, "");
        }
    }

    result = LLVMBuildPointerCast(state->builder, result, pointerType, "");

    return result;
}

/**
 * Create the value corresponding to an operand.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param dataType The data type used to create the appropriate type
 * \param alignment Additional alignment information
 * \param operand The operand
 * \param state The module state
 * \returns The value which corresponds to the operand
 **/
LLVMValueRef
ll_operand_load(OperandDataType dataType, Alignment alignment, LLInstrOp* operand, LLState* state)
{
    int operandWidth = ll_operand_get_type_length(dataType, operand);
    LLVMValueRef result = NULL;

    if (operand->type == LL_OP_IMM)
    {
        LLVMTypeRef type = ll_operand_get_type(dataType, operandWidth, state);
        result = LLVMConstInt(type, operand->val, false);
    }
    else if (operand->type == LL_OP_REG)
    {
        result = ll_get_register(operand->reg, ll_operand_get_facet(dataType, operand), state);
    }
    else if (operand->type == LL_OP_MEM)
    {
        LLVMValueRef address = ll_operand_get_address(dataType, operand, state);
        result = LLVMBuildLoad(state->builder, address, "");
        if (alignment == ALIGN_MAXIMUM)
            LLVMSetAlignment(result, operandWidth / 8);
        else
            LLVMSetAlignment(result, alignment);
    }
    else
    {
        warn_if_reached();
    }

    return result;
}

/**
 * Store the value in an operand.
 *
 * \private
 *
 * \author Alexis Engelke
 *
 * \param dataType The data type used to create the pointer type, if necessary
 * \param alignment Additional alignment information
 * \param operand The operand
 * \param zeroHandling Handling of unused upper parts of the register
 * \param value The value to store
 * \param state The module state
 **/
void
ll_operand_store(OperandDataType dataType, Alignment alignment, LLInstrOp* operand, PartialRegisterHandling zeroHandling, LLVMValueRef value, LLState* state)
{
    if (operand->type == LL_OP_REG)
    {
        if (zeroHandling == REG_DEFAULT)
            ll_operand_store_gp(value, dataType, operand, state);
        else
            ll_operand_store_vreg(value, dataType, operand, zeroHandling, state);
    }
    else if (operand->type == LL_OP_MEM)
    {
        int operandWidth = ll_operand_get_type_length(dataType, operand);
        LLVMValueRef address = ll_operand_get_address(dataType, operand, state);
        LLVMValueRef result = LLVMBuildBitCast(state->builder, value, LLVMGetElementType(LLVMTypeOf(address)), "");
        LLVMValueRef store = LLVMBuildStore(state->builder, result, address);

        if (alignment == ALIGN_MAXIMUM)
            LLVMSetAlignment(store, operandWidth / 8);
        else
            LLVMSetAlignment(store, alignment);
    }
    else
    {
        warn_if_reached();
    }
}

void
ll_operand_construct_args(LLVMTypeRef fnType, LLVMValueRef* args, LLState* state)
{
    LLReg gpRegisters[6] = {
        ll_reg(LL_RT_GP64, LL_RI_DI),
        ll_reg(LL_RT_GP64, LL_RI_SI),
        ll_reg(LL_RT_GP64, LL_RI_D),
        ll_reg(LL_RT_GP64, LL_RI_C),
        ll_reg(LL_RT_GP64, 8),
        ll_reg(LL_RT_GP64, 9),
    };
    int gpRegisterIndex = 0;

    size_t argCount = LLVMCountParamTypes(fnType);
    LLVMTypeRef argTypes[argCount];
    LLVMGetParamTypes(fnType, argTypes);

    for (uintptr_t i = 0; i < argCount; i++)
    {
        LLVMTypeKind argTypeKind = LLVMGetTypeKind(argTypes[i]);

        switch (argTypeKind)
        {
            case LLVMIntegerTypeKind:
            case LLVMPointerTypeKind:
                {
                    if (gpRegisterIndex >= 6)
                        warn_if_reached();

                    LLVMValueRef reg = ll_get_register(gpRegisters[gpRegisterIndex], FACET_I64, state);
                    gpRegisterIndex++;

                    if (argTypeKind == LLVMIntegerTypeKind)
                        args[i] = LLVMBuildTruncOrBitCast(state->builder, reg, argTypes[i], "");
                    else
                        args[i] = LLVMBuildIntToPtr(state->builder, reg, argTypes[i], "");
                }
                break;
            case LLVMVoidTypeKind:
            case LLVMHalfTypeKind:
            case LLVMFloatTypeKind:
            case LLVMDoubleTypeKind:
            case LLVMX86_FP80TypeKind:
            case LLVMFP128TypeKind:
            case LLVMPPC_FP128TypeKind:
            case LLVMLabelTypeKind:
            case LLVMFunctionTypeKind:
            case LLVMStructTypeKind:
            case LLVMArrayTypeKind:
            case LLVMVectorTypeKind:
            case LLVMMetadataTypeKind:
            case LLVMX86_MMXTypeKind:
            case LLVMTokenTypeKind:
            default:
                warn_if_reached();
        }
    }
}

/**
 * @}
 **/
