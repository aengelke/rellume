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

#include "llregfile-internal.h"

#include "facet.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm-c/Core.h>
#include <cassert>
#include <cstdint>
#include <cstring>



/**
 * \defgroup LLRegFile Register File
 * \brief Representation of a register file
 *
 * @{
 **/

llvm::Value*
RegFile::GetReg(LLReg reg, Facet facet)
{
    llvm::LLVMContext& ctx = llvm_block->getContext();

    llvm::IRBuilder<> builder(ctx);
    llvm::Instruction* terminator = llvm_block->getTerminator();
    if (terminator != NULL)
        builder.SetInsertPoint(terminator);
    else
        builder.SetInsertPoint(llvm_block);

    llvm::Type* facetType = facet.Type(ctx);

    if (reg.IsGp())
    {
        auto& entry = regs_gp[reg.ri - (reg.IsGpHigh() ? LL_RI_AH : 0)];
        llvm::Value* res = entry.at(facet);
        if (res != nullptr)
            return res;

        llvm::Value* native = entry.at(Facet::I64);
        assert(native && "native gp-reg facet is null");
        switch (facet)
        {
        case Facet::I64:
        case Facet::I32:
        case Facet::I16:
        case Facet::I8:
            res = builder.CreateTrunc(native, facetType);
            break;
        case Facet::I8H:
            res = builder.CreateLShr(native, builder.getInt64(8));
            res = builder.CreateTrunc(native, builder.getInt8Ty());
            break;
        case Facet::PTR:
            res = builder.CreateIntToPtr(native, facetType);
            break;
        default:
            assert(false && "invalid facet for gp-reg");
            break;
        }

        entry.at(facet) = res;
        return res;
    }
    else if (reg.rt == LL_RT_IP)
    {
        if (facet == Facet::I64)
            return reg_ip;
        else if (facet == Facet::PTR)
            return builder.CreateIntToPtr(reg_ip, facetType);
        else
            assert(false && "invalid facet for ip-reg");
    }
    else if (reg.IsVec())
    {
        assert(reg.IsVec());
        auto& entry = regs_sse[reg.ri];
        auto& res = entry.at(facet);
        if (res != nullptr)
            return res;

        llvm::Value* native = entry.at(Facet::IVEC);
        assert(native && "native sse-reg facet is null");
        switch (facet)
        {
        case Facet::I128:
            res = builder.CreateTrunc(native, facetType);
            break;
        case Facet::I8:
            res = GetReg(reg, Facet::V16I8);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::I16:
            res = GetReg(reg, Facet::V8I16);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::I32:
            res = GetReg(reg, Facet::V4I32);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::I64:
            res = GetReg(reg, Facet::V2I64);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::F32:
            res = GetReg(reg, Facet::V4F32);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::F64:
            res = GetReg(reg, Facet::V2F64);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::V1I8:
        case Facet::V2I8:
        case Facet::V4I8:
        case Facet::V8I8:
        case Facet::V16I8:
        case Facet::V1I16:
        case Facet::V2I16:
        case Facet::V4I16:
        case Facet::V8I16:
        case Facet::V1I32:
        case Facet::V2I32:
        case Facet::V4I32:
        case Facet::V1I64:
        case Facet::V2I64:
        case Facet::V1F32:
        case Facet::V2F32:
        case Facet::V4F32:
        case Facet::V1F64:
        case Facet::V2F64: {
            int targetBits = facetType->getPrimitiveSizeInBits();
            llvm::Type* elementType = facetType->getVectorElementType();
            int elementBits = elementType->getPrimitiveSizeInBits();

            // Prefer 128-bit SSE facet over full vector register.
            llvm::Value* native = entry.at(Facet::IVEC);
            if (targetBits <= 128 && entry.at(Facet::I128) != nullptr)
                native = entry.at(Facet::I128);
            int nativeBits = native->getType()->getPrimitiveSizeInBits();

            int targetCnt = facetType->getVectorNumElements();
            int nativeCnt = nativeBits / elementBits;

            // Cast native facet to appropriate type
            llvm::Type* nativeVecType = llvm::VectorType::get(elementType, nativeCnt);
            res = builder.CreateBitCast(native, nativeVecType);

            // If a shorter vector is required, use shufflevector.
            if (nativeCnt > targetCnt)
            {
                llvm::SmallVector<uint32_t, 16> mask;
                for (int i = 0; i < targetCnt; i++)
                    mask.push_back(i);
                llvm::Value* undef = llvm::UndefValue::get(nativeVecType);
                res = builder.CreateShuffleVector(res, undef, mask);
            }
            break;
        }
        default:
            assert(false && "invalid facet for sse-reg");
            break;
        }

        entry.at(facet) = res;
        return res;
    }

    warn_if_reached();

    return nullptr;
}

void
RegFile::Rename(LLReg reg_dst, LLReg reg_src)
{
    // TODO: copy smaller facets for partial registers only.
    if (reg_dst.IsGp())
    {
        assert(reg_dst.IsGp());
        regs_gp[reg_dst.ri] = regs_gp[reg_src.ri];
    }
    else
    {
        warn_if_reached();
    }
}

void
RegFile::SetReg(LLReg reg, Facet facet, llvm::Value* value, bool clearOthers)
{
    llvm::LLVMContext& ctx = llvm_block->getContext();
    llvm::IRBuilder<> builder(ctx);
    builder.SetInsertPoint(llvm_block);

    if (llvm::isa<llvm::Instruction>(value))
    {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "asm.reg.%s", reg.Name());
        llvm::MDNode* md = llvm::MDNode::get(ctx, {});
        llvm::cast<llvm::Instruction>(value)->setMetadata(buffer, md);
    }

    assert(value->getType() == facet.Type(ctx));

    if (reg.IsGp())
    {
        auto& entry = regs_gp[reg.ri - (reg.IsGpHigh() ? LL_RI_AH : 0)];
        if (clearOthers)
        {
            assert(facet == Facet::I64 || facet == Facet::PTR);
            entry.clear();
            if (facet == Facet::PTR)
                entry.at(Facet::I64) = builder.CreatePtrToInt(value, builder.getInt64Ty());
        }
        entry.at(facet) = value;
    }
    else if (reg.rt == LL_RT_IP)
    {
        if (facet == Facet::I64)
            reg_ip = value;
        else if (facet == Facet::PTR)
            reg_ip = builder.CreatePtrToInt(value, builder.getInt64Ty());
        else
            assert(false && "invalid facet for ip-reg");
    }
    else if (reg.IsVec())
    {
        auto& entry = regs_sse[reg.ri];
        assert(!clearOthers || facet == Facet::IVEC);
        if (clearOthers)
            entry.clear();
        entry.at(facet) = value;
    }
    else
    {
        warn_if_reached();
    }
}

llvm::Value*
RegFile::GetFlag(int flag)
{
    return flags[flag];
}

void
RegFile::SetFlag(int flag, llvm::Value* value)
{
    if (llvm::isa<llvm::Instruction>(value))
    {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "asm.reg.%cf", "zspcoa"[flag]);
        llvm::MDNode* md = llvm::MDNode::get(llvm_block->getContext(), {});
        llvm::cast<llvm::Instruction>(value)->setMetadata(buffer, md);
    }

    flags[flag] = value;
    flag_cache.valid = false;
}

/**
 * @}
 **/
