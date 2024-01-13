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

#include "x86-64/lifter-private.h"

#include "callconv.h"
#include "facet.h"
#include "instr.h"

#include <llvm/IR/Constants.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>

#include <cstddef>
#include <cstdint>

/**
 * \defgroup LLInstrOp LLInstrOp
 * \brief Handling of instruction operands
 *
 * @{
 **/

namespace rellume::x86_64 {

ArchReg Lifter::MapReg(const Instr::Reg reg) {
    if (reg.rt == FD_RT_GPL)
        return reg.ri == FD_REG_IP ? ArchReg::IP : ArchReg::GP(reg.ri);
    else if (reg.rt == FD_RT_GPH)
        return ArchReg::GP(reg.ri - FD_REG_AH);
    else if (reg.rt == FD_RT_VEC)
        return ArchReg::VEC(reg.ri);
    return ArchReg();
}

llvm::Value* Lifter::OpAddr(const Instr::Op op, llvm::Type* element_type,
                                unsigned seg) {
    if (seg == 7)
        seg = op.seg();
    if (seg == FD_REG_FS || seg == FD_REG_GS || op.addrsz() != 8) {
        // For segment offsets, use inttoptr because the pointer base is stored
        // in the segment register. (And LLVM has some problems with addrspace
        // casts between pointers.) For 32-bit address size, we can't normal
        // pointers, so use integer arithmetic directly.
        Facet addrsz_facet = op.addrsz() == 8 ? Facet::I64 : Facet::I32;

        llvm::Value* res = irb.getIntN(8 * op.addrsz(), op.off());
        if (op.base())
            res = irb.CreateAdd(res, GetReg(MapReg(op.base()), addrsz_facet));
        if (op.scale() != 0) {
            llvm::Value* ireg = GetReg(MapReg(op.index()), addrsz_facet);
            llvm::Value* scaled_val = irb.getIntN(8 * op.addrsz(), op.scale());
            res = irb.CreateAdd(res, irb.CreateMul(ireg, scaled_val));
        }

        int addrspace = 0;
        if (seg == FD_REG_FS || seg == FD_REG_GS) {
            if (cfg.use_native_segment_base) {
                addrspace = seg == FD_REG_FS ? 257 : 256;
            } else {
                unsigned idx = seg == FD_REG_FS ? SptrIdx::x86_64::FSBASE
                                                : SptrIdx::x86_64::GSBASE;
                auto base = irb.CreateLoad(irb.getInt64Ty(), fi.sptr[idx]);
                res = irb.CreateAdd(res, base);
            }
        }

        res = irb.CreateZExt(res, irb.getInt64Ty());
        return irb.CreateIntToPtr(res, element_type->getPointerTo(addrspace));
    }

    llvm::PointerType* elem_ptr_ty = element_type->getPointerTo();

    llvm::Type* scale_type = nullptr;
    if (op.scale() * 8u == element_type->getPrimitiveSizeInBits())
        scale_type = element_type;
    else if (op.scale() != 0)
        scale_type = irb.getIntNTy(op.scale() * 8);

    // GEPs are safe because null-pointer-is-valid attribute is set.
    llvm::Value* base;
    if (op.base()) {
        base = GetReg(MapReg(op.base()), Facet::PTR);
        if (llvm::isa<llvm::Constant>(base)) {
            llvm::Value* base_int = GetReg(MapReg(op.base()), Facet::I64);
            base_int = irb.CreateAdd(base_int, irb.getInt64(op.off()));
            if (auto* addr = llvm::dyn_cast<llvm::ConstantInt>(base_int)) {
                base = AddrConst(addr->getZExtValue(), elem_ptr_ty);
            } else {
                base = irb.CreateIntToPtr(base_int, elem_ptr_ty);
            }
        } else if (op.off() != 0) {
            if (op.scale() != 0 && (op.off() % op.scale()) == 0) {
                base = irb.CreatePointerCast(base, scale_type->getPointerTo());
                base = irb.CreateGEP(scale_type, base, irb.getInt64(op.off() / op.scale()));
            } else {
                base = irb.CreatePointerCast(base, irb.getInt8PtrTy());
                base = irb.CreateGEP(irb.getInt8Ty(), base, irb.getInt64(op.off()));
            }
        }
    } else {
        base = AddrConst(op.off(), elem_ptr_ty);
    }

    if (op.scale() != 0) {
        bool use_mul = false;
        if (auto constval = llvm::dyn_cast<llvm::Constant>(base))
            use_mul = constval->isNullValue();

        llvm::Value* offset = GetReg(MapReg(op.index()), Facet::I64);
        if (use_mul) {
            base = irb.CreateMul(offset, irb.getInt64(op.scale()));
            base = irb.CreateIntToPtr(base, elem_ptr_ty);
        } else {
            base = irb.CreatePointerCast(base, scale_type->getPointerTo());
            base = irb.CreateGEP(scale_type, base, offset);
        }
    }

    return irb.CreatePointerCast(base, elem_ptr_ty);
}

static void ll_operand_set_alignment(llvm::Instruction* value, llvm::Type* type,
                                     Alignment alignment, bool sse = false) {
    if (alignment == ALIGN_IMP)
        alignment = sse ? ALIGN_MAX : ALIGN_NONE;
    unsigned bytes =
        alignment == ALIGN_NONE ? 1 : type->getPrimitiveSizeInBits() / 8;
    llvm::Align align(bytes);
    if (llvm::LoadInst* load = llvm::dyn_cast<llvm::LoadInst>(value))
        load->setAlignment(align);
    else if (llvm::StoreInst* store = llvm::dyn_cast<llvm::StoreInst>(value))
        store->setAlignment(align);
}

llvm::Value* Lifter::OpLoad(const Instr::Op op, Facet facet,
                                Alignment alignment, unsigned seg) {
    facet = facet.Resolve(op.bits());
    if (op.is_imm()) {
        return irb.getIntN(op.bits(), op.imm());
    } else if (op.is_pcrel()) {
        return AddrIPRel(op.pcrel(), facet);
    } else if (op.is_reg()) {
        if (facet == Facet::I8 && op.reg().rt == FD_RT_GPH)
            facet = Facet::I8H;
        return GetReg(MapReg(op.reg()), facet);
    } else if (op.is_mem()) {
        llvm::Type* type = facet.Type(irb.getContext());
        llvm::Value* addr = OpAddr(op, type, seg);
        llvm::LoadInst* result = irb.CreateLoad(type, addr);
        // FIXME: forward SSE information to increase alignment.
        ll_operand_set_alignment(result, type, alignment, false);
        return result;
    }

    assert(false);
    return nullptr;
}

void Lifter::StoreGpFacet(ArchReg reg, Facet facet, llvm::Value* value) {
    assert(reg.IsGP());
    assert(value->getType()->isIntegerTy());
    assert(value->getType() == facet.Type(irb.getContext()));

    if (facet == Facet::I8H) {
        uint64_t mask = 0xffffffffffff00ff;
        llvm::Value* maskedOld = irb.CreateAnd(GetReg(reg, Facet::I64), mask);

        value = irb.CreateShl(irb.CreateZExt(value, irb.getInt64Ty()), 8);
        SetReg(reg, irb.CreateOr(value, maskedOld));
    } else if (facet == Facet::I8 || facet == Facet::I16) {
        SetRegMerge(reg, value);
    } else {
        SetReg(reg, value);
    }
}

void Lifter::OpStoreGp(const Instr::Op op, llvm::Value* value,
                           Alignment alignment) {
    if (op.is_mem()) {
        llvm::Value* addr = OpAddr(op, value->getType());
        llvm::StoreInst* store = irb.CreateStore(value, addr);
        ll_operand_set_alignment(store, value->getType(), alignment);
    } else if (op.is_reg()) {
        assert(value->getType()->getIntegerBitWidth() == op.bits());

        Facet facet = Facet::In(value->getType()->getIntegerBitWidth());
        if (facet == Facet::I8 && op.reg().rt == FD_RT_GPH)
            facet = Facet::I8H;
        StoreGpFacet(MapReg(op.reg()), facet, value);
    } else {
        assert(false && "gp-store to non-mem/non-reg");
    }
}

void Lifter::OpStoreVec(const Instr::Op op, llvm::Value* value,
                        Alignment alignment) {
    if (op.is_mem()) {
        llvm::Value* addr = OpAddr(op, value->getType());
        llvm::StoreInst* store = irb.CreateStore(value, addr);
        ll_operand_set_alignment(store, value->getType(), alignment, true);
        return;
    }

    assert(op.is_reg() && "vec-store to non-mem/non-reg");

    ArchReg reg = MapReg(op.reg());

    Facet ivec_facet = Facet::V2I64;
    llvm::Type* ivec_ty = ivec_facet.Type(irb.getContext());
    unsigned ivec_sz = ivec_ty->getPrimitiveSizeInBits();
    llvm::Type* value_ty = value->getType();

    // Handle case where the value fills the entire register.
    if (value_ty->getPrimitiveSizeInBits() == ivec_sz) {
        SetReg(reg, irb.CreateBitCast(value, ivec_ty));
        SetRegFacet(reg, value);
        return;
    }

    // Construct the requires vector type of the vector register.
    llvm::Type* element_ty =
        value_ty->isVectorTy() ? value_ty->getScalarType() : value_ty;
    unsigned full_num = ivec_sz / element_ty->getPrimitiveSizeInBits();
    Facet full_facet = Facet::Vnt(full_num, Facet::FromType(element_ty));

    llvm::Value* full = GetReg(reg, full_facet);

    if (!value_ty->isVectorTy()) {
        // Handle scalar values with an insertelement instruction
        full = irb.CreateInsertElement(full, value, uint64_t{0});
    } else {
        // Vector-in-vector insertion require 2 x shufflevector.
        // First, we enlarge the input vector to the full register length.
        unsigned value_num_elts = VectorElementCount(value_ty);
        llvm::SmallVector<int, 16> mask;
        for (unsigned i = 0; i < full_num; i++)
            mask.push_back(i < value_num_elts ? i : value_num_elts);
        llvm::Value* zero = llvm::Constant::getNullValue(value_ty);
        llvm::Value* ext_vec = irb.CreateShuffleVector(value, zero, mask);

        // Now shuffle the two vectors together
        for (unsigned i = 0; i < full_num; i++)
            mask[i] = i + (i < value_num_elts ? 0 : full_num);
        full = irb.CreateShuffleVector(ext_vec, full, mask);
    }

    SetReg(reg, irb.CreateBitCast(full, ivec_ty));
    SetRegFacet(reg, full);
    SetRegFacet(reg, value);
}

void Lifter::StackPush(llvm::Value* value) {
    llvm::Value* rsp = GetReg(ArchReg::RSP, Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, value->getType()->getPointerTo());
    rsp = irb.CreateConstGEP1_64(value->getType(), rsp, -1);
    irb.CreateStore(value, rsp);

    SetReg(ArchReg::RSP, rsp);
}

llvm::Value* Lifter::StackPop(const ArchReg sp_src_reg) {
    llvm::Value* rsp = GetReg(sp_src_reg, Facet::PTR);
    rsp = irb.CreatePointerCast(rsp, irb.getInt64Ty()->getPointerTo());

    SetReg(ArchReg::RSP, irb.CreateConstGEP1_64(irb.getInt64Ty(), rsp, 1));

    return irb.CreateLoad(irb.getInt64Ty(), rsp);
}

} // namespace rellume::x86_64

/**
 * @}
 **/
