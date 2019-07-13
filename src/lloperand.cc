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

#include "llstate-internal.h"

#include "facet.h"
#include "rellume/instr.h"
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm-c/Core.h>
#include <cstddef>
#include <cstdint>


/**
 * \defgroup LLInstrOp LLInstrOp
 * \brief Handling of instruction operands
 *
 * @{
 **/

llvm::Value*
LLStateBase::OpAddrConst(uint64_t addr)
{
    if (addr == 0)
        return llvm::ConstantPointerNull::get(irb.getInt8PtrTy());

    if (cfg.global_base_value != nullptr)
    {
        uintptr_t offset = addr - cfg.global_base_addr;
        return irb.CreateGEP(cfg.global_base_value, {irb.getInt64(offset)});
    }

    return irb.CreateIntToPtr(irb.getInt64(addr), irb.getInt8PtrTy());
}

llvm::Value*
LLStateBase::OpAddr(const LLInstrOp& op, llvm::Type* element_type)
{
    int addrspace = 0;
    switch (op.seg)
    {
    case LL_RI_None: addrspace = 0; break;
    case LL_RI_GS: addrspace = 256; break;
    case LL_RI_FS: addrspace = 257; break;
    default: assert(false); return nullptr;
    }

    llvm::Type* scale_type = nullptr;
    if (op.scale * 8u == element_type->getPrimitiveSizeInBits())
        scale_type = element_type->getPointerTo(addrspace);
    else if (op.scale != 0)
        scale_type = irb.getIntNTy(op.scale*8)->getPointerTo(addrspace);

    llvm::Value* base;
    if (op.reg.rt != LL_RT_None)
    {
        base = GetReg(op.reg, Facet::PTR);
        if (llvm::isa<llvm::Constant>(base))
        {
            auto base_addr = llvm::cast<llvm::ConstantInt>(GetReg(op.reg, Facet::I64));
            base = OpAddrConst(base_addr->getZExtValue() + op.val);
        }
        else if (op.val != 0)
        {
            if (op.scale != 0 && (op.val % op.scale) == 0)
            {
                base = irb.CreatePointerCast(base, scale_type);
                base = irb.CreateGEP(base, {irb.getInt64(op.val/op.scale)});
            }
            else
            {
                base = irb.CreateGEP(base, {irb.getInt64(op.val)});
            }
        }
    }
    else
    {
        base = OpAddrConst(op.val);
    }

    if (op.scale != 0)
    {
        llvm::Value* offset = GetReg(op.ireg, Facet::I64);
        // TODO: if base is constant null, use mul+inttoptr
        base = irb.CreatePointerCast(base, scale_type);
        base = irb.CreateGEP(base, {offset});
    }

    return irb.CreatePointerCast(base, element_type->getPointerTo(addrspace));
}

static void
ll_operand_set_alignment(llvm::Instruction* value, Alignment alignment, bool sse = false)
{
    if (alignment == ALIGN_IMP)
        alignment = sse ? ALIGN_MAX : ALIGN_NONE;
    if (llvm::LoadInst* load = llvm::dyn_cast<llvm::LoadInst>(value))
        load->setAlignment(alignment == ALIGN_NONE ? 1 : load->getPointerOperandType()->getPrimitiveSizeInBits() / 8);
    else if (llvm::StoreInst* store = llvm::dyn_cast<llvm::StoreInst>(value))
        store->setAlignment(alignment == ALIGN_NONE ? 1 : store->getPointerOperandType()->getPrimitiveSizeInBits() / 8);
}

llvm::Value*
LLStateBase::OpLoad(const LLInstrOp& op, Facet facet, Alignment alignment)
{
    facet = facet.Resolve(op.size * 8);
    if (op.type == LL_OP_IMM)
    {
        llvm::Type* type = facet.Type(irb.getContext());
        return llvm::ConstantInt::get(type, op.val);
    }
    else if (op.type == LL_OP_REG)
    {
        if (op.reg.IsGpHigh() && facet == Facet::I8)
            facet = Facet::I8H;
        return GetReg(op.reg, facet);
    }
    else if (op.type == LL_OP_MEM)
    {
        llvm::Type* type = facet.Type(irb.getContext());
        llvm::Value* addr = OpAddr(op, type);
        llvm::LoadInst* result = irb.CreateLoad(type, addr);
        // FIXME: forward SSE information to increase alignment.
        ll_operand_set_alignment(result, alignment, false);
        return result;
    }

    assert(false);
    return nullptr;
}

void
LLStateBase::OpStoreGp(const LLInstrOp& op, llvm::Value* value, Alignment alignment)
{
    if (op.type == LL_OP_MEM)
    {
        llvm::Value* addr = OpAddr(op, value->getType());
        llvm::StoreInst* store = irb.CreateStore(value, addr);
        ll_operand_set_alignment(store, alignment);
        return;
    }

    assert(op.type == LL_OP_REG && "gp-store to non-mem/non-reg");

    value = irb.CreateSExtOrBitCast(value, irb.getIntNTy(op.size * 8));
    if (op.reg.rt == LL_RT_GP64 || op.reg.rt == LL_RT_IP)
    {
        SetReg(op.reg, Facet::I64, value);
        return;
    }

    llvm::Value* value64 = irb.CreateZExtOrBitCast(value, irb.getInt64Ty());

    if (op.reg.rt == LL_RT_GP32)
    {
        SetReg(op.reg, Facet::I64, value64);
        SetRegFacet(op.reg, Facet::I32, value);
        return;
    }

    uint64_t mask;
    Facet store_facet;
    if (op.reg.IsGpHigh())
    {
        mask = 0xff00;
        store_facet = Facet::I8H;
        value64 = irb.CreateShl(value64, 8);
    }
    else if (op.size == 1)
    {
        mask = 0xff;
        store_facet = Facet::I8;
    }
    else if (op.size == 2)
    {
        mask = 0xffff;
        store_facet = Facet::I16;
    }
    else
    {
        assert(false);
    }

    llvm::Value* masked = irb.CreateAnd(GetReg(op.reg, Facet::I64), ~mask);
    SetReg(op.reg, Facet::I64, irb.CreateOr(value64, masked));
    SetRegFacet(op.reg, store_facet, value);
}

void
LLStateBase::OpStoreVec(const LLInstrOp& op, llvm::Value* value, bool avx,
                        Alignment alignment)
{
    if (op.type == LL_OP_MEM)
    {
        llvm::Value* addr = OpAddr(op, value->getType());
        llvm::StoreInst* store = irb.CreateStore(value, addr);
        ll_operand_set_alignment(store, alignment, !avx);
        return;
    }

    assert(op.type == LL_OP_REG && "vec-store to non-mem/non-reg");

    size_t operandWidth = value->getType()->getPrimitiveSizeInBits();
    // assert(operandWidth == Facet::Type(dataType, state->irb.getContext())->getPrimitiveSizeInBits());

    llvm::Type* iVec = irb.getIntNTy(LL_VECTOR_REGISTER_SIZE);
    llvm::Value* current = irb.getIntN(LL_VECTOR_REGISTER_SIZE, 0);
    llvm::Value* current128 = irb.getIntN(128, 0);
    if (!avx)
    {
        current = GetReg(op.reg, Facet::IVEC);
        current128 = GetReg(op.reg, Facet::I128);
    }

    llvm::Type* value_type = value->getType();
    if (value_type->isVectorTy())
    {
        llvm::Value* full_vec = value;
        if (operandWidth < LL_VECTOR_REGISTER_SIZE)
        {
            unsigned element_count = value_type->getVectorNumElements();
            unsigned total_count = element_count * LL_VECTOR_REGISTER_SIZE / operandWidth;
            llvm::Type* element_type = value_type->getVectorElementType();
            llvm::Type* full_type = llvm::VectorType::get(element_type, total_count);
            llvm::Value* current_vector = irb.CreateBitCast(current, full_type);

            // First, we enlarge the input vector to the full register length.
            llvm::SmallVector<uint32_t, 16> mask;
            for (unsigned i = 0; i < total_count; i++)
                mask.push_back(i < element_count ? i : element_count);
            full_vec = irb.CreateShuffleVector(value, llvm::Constant::getNullValue(value_type), mask);

            // Now shuffle the two vectors together
            for (unsigned i = 0; i < total_count; i++)
                mask[i] = i + (i < element_count ? 0 : total_count);
            full_vec = irb.CreateShuffleVector(full_vec, current_vector, mask);
        }

        SetReg(op.reg, Facet::IVEC, irb.CreateBitCast(full_vec, iVec));
#if LL_VECTOR_REGISTER_SIZE >= 256
        // Induce some common facets via i128 for better SSE support
        if (operandWidth == 128)
        {
            llvm::Value* sse = irb.CreateBitCast(value, irb.getInt128Ty());
            SetRegFacet(op.reg, Facet::I128, sse);
        }
#endif
    }
    else
    {
        unsigned total_count = LL_VECTOR_REGISTER_SIZE / operandWidth;
        llvm::Type* full_type = llvm::VectorType::get(value_type, total_count);
        llvm::Value* full_vector = irb.CreateBitCast(current, full_type);
        full_vector = irb.CreateInsertElement(full_vector, value, 0ul);
        SetReg(op.reg, Facet::IVEC, irb.CreateBitCast(full_vector, iVec));

#if LL_VECTOR_REGISTER_SIZE >= 256
        // Induce some common facets via i128 for better SSE support
        llvm::Type* sse_type = llvm::VectorType::get(value_type, 128 / operandWidth);
        llvm::Value* sse_vector = irb.CreateBitCast(current128, sse_type);
        sse_vector = irb.CreateInsertElement(sse_vector, value, 0ul);
        llvm::Value* sse = irb.CreateBitCast(sse_vector, irb.getInt128Ty());
        SetRegFacet(op.reg, Facet::I128, sse);
#endif
    }
}

void LLStateBase::StackPush(llvm::Value* value) {
    llvm::Value* rsp = GetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, value->getType()->getPointerTo());
    rsp = irb.CreateConstGEP1_64(rsp, -1);
    irb.CreateStore(value, rsp);

    rsp = irb.CreatePointerCast(rsp, irb.getInt8PtrTy());
    SetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR, rsp);
}

llvm::Value* LLStateBase::StackPop(const LLReg sp_src_reg) {
    llvm::Value* rsp = GetReg(sp_src_reg, Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, irb.getInt64Ty()->getPointerTo());

    llvm::Value* new_rsp = irb.CreateConstGEP1_64(rsp, 1);
    new_rsp = irb.CreatePointerCast(new_rsp, irb.getInt8PtrTy());
    SetReg(LLReg(LL_RT_GP64, LL_RI_SP), Facet::PTR, new_rsp);

    return irb.CreateLoad(rsp);
}

/**
 * @}
 **/
