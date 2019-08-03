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

#include "regfile.h"

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
#include <tuple>



/**
 * \defgroup LLRegFile Register File
 * \brief Representation of a register file
 *
 * @{
 **/

namespace rellume {

void RegFile::InitAll(InitGenerator init_gen) {
    if (!init_gen)
        init_gen = [](const LLReg reg, const Facet facet) { return nullptr; };

    // Set create_phi to true for all registers.
    for (unsigned i = 0; i < LL_RI_GPMax; i++) {
        for (Facet facet : regs_gp[i].facets())
            regs_gp[i][facet] = init_gen(LLReg(LL_RT_GP64, i), facet);
    }
    for (unsigned i = 0; i < LL_RI_XMMMax; i++) {
        for (Facet facet : regs_sse[i].facets())
            regs_sse[i][facet] = init_gen(LLReg(LL_RT_XMM, i), facet);
    }
    for (Facet facet : flags.facets())
        flags[facet] = init_gen(LLReg(LL_RT_EFLAGS, 0), facet);
    reg_ip = init_gen(LLReg(LL_RT_IP, 0), Facet::I64);
}

RegFile::Entry* RegFile::AccessRegFacet(LLReg reg, Facet facet) {
    if (reg.IsGp())
    {
        auto& map_entry = regs_gp[reg.ri - (reg.IsGpHigh() ? LL_RI_AH : 0)];
        if (map_entry.has(facet))
            return &map_entry[facet];
    }
    else if (reg.rt == LL_RT_IP)
    {
        if (facet == Facet::I64)
            return &reg_ip;
    }
    else if (reg.rt == LL_RT_EFLAGS)
    {
        if (flags.has(facet))
            return &flags[facet];
    }
    else if (reg.IsVec())
    {
        auto& map_entry = regs_sse[reg.ri];
        if (map_entry.has(facet))
            return &map_entry[facet];
    }

    return nullptr;
}

void RegFile::UpdateAll(llvm::Value* buf_ptr, bool store_mem) {
    static constexpr std::tuple<size_t, LLReg, Facet> entries[] = {
#define RELLUME_PARAM_REG(off,sz,reg,facet,name) std::make_tuple(off,reg,facet),
#include <rellume/regs.inc>
#undef RELLUME_PARAM_REG
    };

    assert(llvm_block->getTerminator() == nullptr && "update terminated block");
    llvm::IRBuilder<> irb(llvm_block);

    // Clear all register facets
    if (!store_mem)
        InitAll(nullptr);

    size_t offset;
    LLReg reg;
    Facet facet;
    for (auto& entry : entries) {
        std::tie(offset, reg, facet) = entry;
        llvm::Type* ptr_ty = facet.Type(irb.getContext())->getPointerTo();
        llvm::Value* ptr = irb.CreateConstGEP1_64(buf_ptr, offset);
        ptr = irb.CreatePointerCast(ptr, ptr_ty);
        Entry* facet_entry = AccessRegFacet(reg, facet);

        assert(facet_entry != nullptr && "required facet doesn't exist");

        if (store_mem) // store to mem, basically GetReg
            irb.CreateStore(facet_entry->get(), ptr);
        else // load from mem, basically SetReg
            *facet_entry = irb.CreateLoad(ptr);
    }
}

llvm::Value*
RegFile::GetReg(LLReg reg, Facet facet)
{
    Entry* facet_entry = AccessRegFacet(reg, facet);
    // If we store the selected facet in our register file and the facet is
    // valid, return it immediately.
    if (facet_entry)
        if (llvm::Value* res = facet_entry->get())
            return res;

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
        llvm::Value* res = nullptr;
        llvm::Value* native = AccessRegFacet(reg, Facet::I64)->get();
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
            res = builder.CreateTrunc(res, builder.getInt8Ty());
            break;
        case Facet::PTR:
            res = builder.CreateIntToPtr(native, facetType);
            break;
        default:
            assert(false && "invalid facet for gp-reg");
            break;
        }

        if (facet_entry != nullptr)
            *facet_entry = res;
        return res;
    }
    else if (reg.rt == LL_RT_IP)
    {
        llvm::Value* native = AccessRegFacet(reg, Facet::I64)->get();
        if (facet == Facet::I64)
            return native;
        else if (facet == Facet::PTR)
            return builder.CreateIntToPtr(native, facetType);
        else
            assert(false && "invalid facet for ip-reg");
    }
    else if (reg.IsVec())
    {
        llvm::Value* res = nullptr;
        llvm::Value* native = AccessRegFacet(reg, Facet::IVEC)->get();
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
            if (targetBits <= 128)
                if (Entry* entry_128 = AccessRegFacet(reg, Facet::I128))
                    if (llvm::Value* value_128 = entry_128->get())
                        native = value_128;
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

        if (facet_entry != nullptr)
            *facet_entry = res;
        return res;
    }

    assert(0);

    return nullptr;
}

void
RegFile::SetReg(LLReg reg, Facet facet, llvm::Value* value, bool clearOthers)
{
#ifdef RELLUME_ANNOTATE_METADATA
    if (llvm::isa<llvm::Instruction>(value))
    {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "asm.reg.%s", reg.Name());
        llvm::MDNode* md = llvm::MDNode::get(llvm_block->getContext(), {});
        llvm::cast<llvm::Instruction>(value)->setMetadata(buffer, md);
    }
#endif

    assert(value->getType() == facet.Type(llvm_block->getContext()));

    if (clearOthers) {
        if (reg.IsGp()) {
            assert(facet == Facet::I64);
            regs_gp[reg.ri - (reg.IsGpHigh() ? LL_RI_AH : 0)].clear();
        } else if (reg.IsVec()) {
            assert(facet == Facet::IVEC);
            regs_sse[reg.ri].clear();
        }
    }

    Entry* facet_entry = AccessRegFacet(reg, facet);
    assert(facet_entry && "attempt to store invalid facet");
    *facet_entry = value;
}

static Facet regfile_flag_to_facet(int flag) {
    switch(flag) {
    case RFLAG_ZF: return Facet::ZF;
    case RFLAG_SF: return Facet::SF;
    case RFLAG_PF: return Facet::PF;
    case RFLAG_CF: return Facet::CF;
    case RFLAG_OF: return Facet::OF;
    case RFLAG_AF: return Facet::AF;
    default: return Facet::MAX;
    }
}

llvm::Value*
RegFile::GetFlag(int flag)
{
    Facet facet = regfile_flag_to_facet(flag);
    return AccessRegFacet(LLReg(LL_RT_EFLAGS, 0), facet)->get();
}

void
RegFile::SetFlag(int flag, llvm::Value* value)
{
#ifdef RELLUME_ANNOTATE_METADATA
    if (llvm::isa<llvm::Instruction>(value))
    {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "asm.reg.%cf", "zspcoa"[flag]);
        llvm::MDNode* md = llvm::MDNode::get(llvm_block->getContext(), {});
        llvm::cast<llvm::Instruction>(value)->setMetadata(buffer, md);
    }
#endif

    Facet facet = regfile_flag_to_facet(flag);
    *AccessRegFacet(LLReg(LL_RT_EFLAGS, 0), facet) = value;
}

} // namespace

/**
 * @}
 **/
