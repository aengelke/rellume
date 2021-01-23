/**
 * This file is part of Rellume.
 *
 * (c) 2021, Dominik Okwieka <dominik.okwieka@t-online.de>
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

#include "a64/lifter.h"
#include "a64/lifter-private.h"

#include "arch.h"
#include "facet.h"
#include "instr.h"
#include "regfile.h"

#include <cstdint>

#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>

namespace rellume::aarch64 {

bool Lifter::LiftSIMD(farmdec::Inst a64) {
    bool round = a64.flags & farmdec::SIMD_ROUND;
    bool sgn = a64.flags & farmdec::SIMD_SIGNED;
    bool scalar = a64.flags & farmdec::SIMD_SCALAR;
    farmdec::VectorArrangement va = fad_get_vec_arrangement(a64.flags);
    bool w32 = a64.flags & farmdec::W32;

    // In the order of farmdec::Op.
    switch (a64.op) {
    case farmdec::A64_LD1_MULT:
    case farmdec::A64_ST1_MULT: {
        bool load = (a64.op == farmdec::A64_LD1_MULT);
        auto vecty = TypeOf(va);

        llvm::Value* base = nullptr;
        farmdec::AddrMode mode = fad_get_addrmode(a64.flags);
        switch (mode) {
        case farmdec::AM_SIMPLE:
            base = Addr(vecty, a64.rn);
            break;
        case farmdec::AM_POST: {
            base = Addr(vecty, a64.rn);
            // Offset/increment can be an immediate or the Rm register.
            auto offset = (a64.rm == farmdec::ZERO_REG) ? irb.getInt64(a64.simd_ldst.offset) : GetGp(a64.rm, /*w32=*/false);
            SetGp(a64.rn, /*w32=*/false, irb.CreateAdd(GetGp(a64.rn, /*w32=*/false), offset)); // rn += offset
            break;
        }
        default:
            assert(false && "bad LD1/ST1 addrmode");
        }

        // For each register Vtt, starting with Vd, wrapping around V31..V0.
        farmdec::Reg tt = a64.rd;
        for (unsigned i = 0; i < a64.simd_ldst.nreg; i++) {
            auto ptr = irb.CreateConstGEP1_64(vecty, base, i);
            if (load) {
                auto vec = irb.CreateLoad(vecty, ptr);
                SetVec(tt, vec);
            } else {
                auto vec = GetVec(tt, va);
                irb.CreateStore(vec, ptr);
            }
            tt = (tt+1) % 32;
        }
        break;
    }
    case farmdec::A64_AND_VEC:
        LiftThreeSame(llvm::Instruction::And, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false);
        break;
    case farmdec::A64_BIC_VEC_IMM: {
        auto lhs = GetVec(a64.rd, va);
        unsigned bits = ElemTypeOf(va)->getPrimitiveSizeInBits();
        auto rhs = irb.CreateVectorSplat(NumElem(va), irb.getIntN(bits, a64.imm));
        SetVec(a64.rd, irb.CreateAnd(lhs, irb.CreateNot(rhs)));
        break;
    }
    case farmdec::A64_BIC_VEC_REG:
        LiftThreeSame(llvm::Instruction::And, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false, /*invert_rhs=*/true);
        break;
    case farmdec::A64_BIF:
    case farmdec::A64_BIT: {
        // "Bit Insert if True/False"
        //
        // BIT: Vd = Vd ^ ((Vd ^ Vn) & Vm);
        // BIF: Vd = Vd ^ ((Vd ^ Vn) & ~Vm);
        auto vd = GetVec(a64.rd, va);
        auto vn = GetVec(a64.rn, va);
        auto mask = (a64.op == farmdec::A64_BIT) ? GetVec(a64.rm, va) : irb.CreateNot(GetVec(a64.rm, va));
        auto vec = irb.CreateXor(vd, irb.CreateAnd(irb.CreateXor(vd, vn), mask));
        SetVec(a64.rd, vec);
        break;
    }
    case farmdec::A64_BSL: {
        // "Bit Select"
        //
        // Vd = Vm ^ ((Vm ^ Vn) & Vd);
        auto vd = GetVec(a64.rd, va);
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);
        auto vec = irb.CreateXor(vm, irb.CreateAnd(irb.CreateXor(vm, vn), vd));
        SetVec(a64.rd, vec);
        break;
    }
    case farmdec::A64_CLS_VEC:
        return false; // XXX has not been encountered yet
    case farmdec::A64_CLZ_VEC: {
        auto val = GetVec(a64.rn, va);
        auto mod = irb.GetInsertBlock()->getModule();
        auto fn = llvm::Intrinsic::getDeclaration(mod, llvm::Intrinsic::ctlz, {val->getType()});
        SetVec(a64.rd, irb.CreateCall(fn, {val, /*is_zero_undef=*/irb.getFalse()}));
        break;
    }
    case farmdec::A64_CNT:
        SetVec(a64.rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::ctpop, GetVec(a64.rn, va)));
        break;
    case farmdec::A64_EOR_VEC:
        LiftThreeSame(llvm::Instruction::Xor, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false);
        break;
    case farmdec::A64_NOT_VEC: 
        SetVec(a64.rd, irb.CreateNot(GetVec(a64.rn, va)));
        break;
    case farmdec::A64_ORN_VEC:
        LiftThreeSame(llvm::Instruction::Or, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false, /*invert_rhs=*/true);
        break;
    case farmdec::A64_ORR_VEC_IMM: {
        auto lhs = GetVec(a64.rd, va);
        unsigned bits = ElemTypeOf(va)->getPrimitiveSizeInBits();
        auto rhs = irb.CreateVectorSplat(NumElem(va), irb.getIntN(bits, a64.imm));
        SetVec(a64.rd, irb.CreateOr(lhs, rhs));
        break;
    }
    case farmdec::A64_MOV_VEC:
    case farmdec::A64_ORR_VEC_REG:
        LiftThreeSame(llvm::Instruction::Or, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false);
        break;
    case farmdec::A64_RBIT_VEC:
        SetVec(a64.rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::bitreverse, GetVec(a64.rn, va)));
        break;
    case farmdec::A64_DUP_ELEM: {
        auto elem = GetElem(a64.rn, va, a64.imm);
        if (scalar) {
            SetScalar(a64.rd, elem);
            break;
        }
        Dup(a64.rd, va, elem);
        break;
    }
    case farmdec::A64_DUP_GPR: {
        auto elem = irb.CreateTruncOrBitCast(GetGp(a64.rn, w32), ElemTypeOf(va));
        Dup(a64.rd, va, elem);
        break;
    }
    case farmdec::A64_EXT: {
        auto lower = GetVec(a64.rn, va);
        auto upper = GetVec(a64.rm, va);

        unsigned start = a64.imm;
        unsigned end = start + NumElem(va);
        llvm::SmallVector<int, 16> mask;
        for (unsigned i = start; i < end; i++) {
            mask.push_back(i);
        }

        auto vec = irb.CreateShuffleVector(lower, upper, mask);
        SetVec(a64.rd, vec);
        break;
    }
    case farmdec::A64_INS_ELEM: {
        auto elem = GetElem(a64.rn, va, a64.ins_elem.src);
        InsertElem(a64.rd, a64.ins_elem.dst, elem);
        break;
    }
    case farmdec::A64_INS_GPR: {
        auto elem = irb.CreateTruncOrBitCast(GetGp(a64.rn, w32), ElemTypeOf(va));
        InsertElem(a64.rd, a64.imm, elem);
        break;
    }
    case farmdec::A64_FMOV_VEC: {
        farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
        Dup(a64.rd, va, llvm::ConstantFP::get(TypeOf(prec), a64.fimm));
        break;
    }
    case farmdec::A64_MOVI: {
        unsigned bits = TypeOf(va)->getScalarSizeInBits();
        if (scalar) {
            SetScalar(a64.rd, irb.getInt64(a64.imm));
            break;
        }
        Dup(a64.rd, va, irb.getIntN(bits, a64.imm));
        break;
    }
    case farmdec::A64_SMOV: {
        auto elem = GetElem(a64.rn, va, a64.imm);
        SetGp(a64.rd, w32, irb.CreateSExt(elem, (w32) ? irb.getInt32Ty() : irb.getInt64Ty()));
        break;
    }
    case farmdec::A64_UMOV: {
        auto elem = GetElem(a64.rn, va, a64.imm);
        SetGp(a64.rd, w32, irb.CreateZExt(elem, (w32) ? irb.getInt32Ty() : irb.getInt64Ty()));
        break;
    }
    case farmdec::A64_CMEQ_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_EQ, a64.rd, a64.rn, a64.rm, /*zero=*/false);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_EQ, a64.rd, va, a64.rn, a64.rm, /*zero=*/false);
        break;
    case farmdec::A64_CMEQ_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_EQ, a64.rd, a64.rn, a64.rm, /*zero=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_EQ, a64.rd, va, a64.rn, a64.rm, /*zero=*/true);
        break;
    case farmdec::A64_CMGE_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_SGE, a64.rd, a64.rn, a64.rm, /*zero=*/false);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_SGE, a64.rd, va, a64.rn, a64.rm, /*zero=*/false);
        break;
    case farmdec::A64_CMGE_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_SGE, a64.rd, a64.rn, a64.rm, /*zero=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_SGE, a64.rd, va, a64.rn, a64.rm, /*zero=*/true);
        break;
    case farmdec::A64_CMGT_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_SGT, a64.rd, a64.rn, a64.rm, /*zero=*/false);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_SGT, a64.rd, va, a64.rn, a64.rm, /*zero=*/false);
        break;
    case farmdec::A64_CMGT_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_SGT, a64.rd, a64.rn, a64.rm, /*zero=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_SGT, a64.rd, va, a64.rn, a64.rm, /*zero=*/true);
        break;
    case farmdec::A64_CMHI_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_UGT, a64.rd, a64.rn, a64.rm, /*zero=*/false);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_UGT, a64.rd, va, a64.rn, a64.rm, /*zero=*/false);
        break;
    case farmdec::A64_CMHS_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_UGE, a64.rd, a64.rn, a64.rm, /*zero=*/false);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_UGE, a64.rd, va, a64.rn, a64.rm, /*zero=*/false);
        break;
    case farmdec::A64_CMLE_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_SLE, a64.rd, a64.rn, a64.rm, /*zero=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_SLE, a64.rd, va, a64.rn, a64.rm, /*zero=*/true);
        break;
    case farmdec::A64_CMLT_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::ICMP_SLT, a64.rd, a64.rn, a64.rm, /*zero=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::ICMP_SLT, a64.rd, va, a64.rn, a64.rm, /*zero=*/true);
        break;
    case farmdec::A64_CMTST:
        return false; // XXX
    case farmdec::A64_ABS_VEC:
        SetVec(a64.rd, Abs(GetVec(a64.rn, va)));
        break;
    case farmdec::A64_NEG_VEC:
        SetVec(a64.rd, irb.CreateNeg(GetVec(a64.rn, va)));
        break;
    case farmdec::A64_ADD_VEC:
        LiftThreeSame(llvm::Instruction::Add, a64.rd, va, a64.rn, a64.rm, scalar);
        break;
    case farmdec::A64_SUB_VEC:
        LiftThreeSame(llvm::Instruction::Sub, a64.rd, va, a64.rn, a64.rm, scalar);
        break;
    case farmdec::A64_MAX_VEC:
        SetVec(a64.rd, MinMax(GetVec(a64.rn, va), GetVec(a64.rm, va), sgn, /*min=*/false));
        break;
    case farmdec::A64_MIN_VEC:
        SetVec(a64.rd, MinMax(GetVec(a64.rn, va), GetVec(a64.rm, va), sgn, /*min=*/true));
        break;
    case farmdec::A64_ADDP:
        SetScalar(a64.rd, irb.CreateAddReduce(GetVec(a64.rn, farmdec::VA_2D)));
        break;
    case farmdec::A64_FADDP_VEC: {
        llvm::Value *lhs = nullptr, *rhs = nullptr;
        TransformSIMDPairwise(va, a64.rn, a64.rm, &lhs, &rhs, /*fp=*/true);
        SetVec(a64.rd, irb.CreateFAdd(lhs, rhs));
        break;
    }
    case farmdec::A64_ADDP_VEC: {
        llvm::Value *lhs = nullptr, *rhs = nullptr;
        TransformSIMDPairwise(va, a64.rn, a64.rm, &lhs, &rhs);
        SetVec(a64.rd, irb.CreateAdd(lhs, rhs));
        break;
    }
    case farmdec::A64_MAXP: {
        llvm::Value *lhs = nullptr, *rhs = nullptr;
        TransformSIMDPairwise(va, a64.rn, a64.rm, &lhs, &rhs);
        SetVec(a64.rd, MinMax(lhs, rhs, sgn, /*min=*/false));
        break;
    }
    case farmdec::A64_MINP: {
        llvm::Value *lhs = nullptr, *rhs = nullptr;
        TransformSIMDPairwise(va, a64.rn, a64.rm, &lhs, &rhs);
        SetVec(a64.rd, MinMax(lhs, rhs, sgn, /*min=*/true));
        break;
    }
    case farmdec::A64_ADDV:
        SetScalar(a64.rd, irb.CreateAddReduce(GetVec(a64.rn, va)));
        break;
    case farmdec::A64_FMAXV:
    case farmdec::A64_FMAXNMV:
        SetScalar(a64.rd, irb.CreateFPMaxReduce(GetVec(a64.rn, va, /*fp=*/true)));
        break;
    case farmdec::A64_MAXV:
        SetScalar(a64.rd, irb.CreateIntMaxReduce(GetVec(a64.rn, va), sgn));
        break;
    case farmdec::A64_FMINV:
    case farmdec::A64_FMINNMV:
        SetScalar(a64.rd, irb.CreateFPMinReduce(GetVec(a64.rn, va, /*fp=*/true)));
        break;
    case farmdec::A64_MINV:
        SetScalar(a64.rd, irb.CreateIntMinReduce(GetVec(a64.rn, va), sgn));
        break;
    default:
        return false;
    }

    return true;
}

llvm::Value* Lifter::GetVec(farmdec::Reg r, farmdec::VectorArrangement va, bool fp) {
    return GetReg(ArchReg::VEC(r), FacetOf(va, fp));
}

llvm::Value* Lifter::GetElem(farmdec::Reg r, farmdec::VectorArrangement va, unsigned i, bool fp) {
    return irb.CreateExtractElement(GetVec(r, va, fp), i);
}

void Lifter::SetVec(farmdec::Reg r, llvm::Value* vec) {
    auto vecty = llvm::cast<llvm::VectorType>(vec->getType());
    Facet fc = Facet::FromType(vecty);
    unsigned bits = fc.Size();
    Facet ivec = Facet::V2I64;

    // Full 128-bit vector
    if (bits == ivec.Size()) {
        SetReg(ArchReg::VEC(r), ivec, irb.CreateBitCast(vec, ivec.Type(irb.getContext())));
        SetRegFacet(ArchReg::VEC(r), fc, vec);
        return;
    }

    // Half vector -- upper half is zeroed.
    auto fullty = llvm::VectorType::getDoubleElementsVectorType(vecty);
    auto zero = llvm::Constant::getNullValue(vecty);

    // Shufflevector (zero, vec, {0, 1, ..., n}) simply concatenates zero and vec.
    auto full_nelem = 2 * bits / vecty->getScalarSizeInBits();
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < full_nelem; i++) {
        mask.push_back(i);
    }
    auto fullvec = irb.CreateShuffleVector(vec, zero, mask);

    SetReg(ArchReg::VEC(r), ivec, irb.CreateBitCast(fullvec, ivec.Type(irb.getContext())));
    SetRegFacet(ArchReg::VEC(r), Facet::FromType(fullty), fullvec);
    SetRegFacet(ArchReg::VEC(r), fc, vec);
}

// Vr[i] := elem, l without touching other lanes.
void Lifter::InsertElem(farmdec::Reg r, unsigned i, llvm::Value* elem) {
    auto elemty = elem->getType();

    farmdec::VectorArrangement va;
    switch(elemty->getPrimitiveSizeInBits()) {
    case  8: va = farmdec::VA_16B; break;
    case 16: va = farmdec::VA_8H; break;
    case 32: va = farmdec::VA_4S; break;
    case 64: va = farmdec::VA_2D; break;
    default:
        assert(false && "bad element bit count");
    }

    auto oldvec = GetVec(r, va, elemty->isFPOrFPVectorTy());
    auto newvec = irb.CreateInsertElement(oldvec, elem, i);
    SetVec(r, newvec);
}

// Duplicate elem to all lanes.
void Lifter::Dup(farmdec::Reg r, farmdec::VectorArrangement va, llvm::Value* elem) {
    SetVec(r, irb.CreateVectorSplat(NumElem(va), elem));
}

unsigned Lifter::NumElem(farmdec::VectorArrangement va) {
    switch (va) {
    case farmdec::VA_8B:  return 8;
    case farmdec::VA_16B: return 16;
    case farmdec::VA_4H:  return 4;
    case farmdec::VA_8H:  return 8;
    case farmdec::VA_2S:  return 2;
    case farmdec::VA_4S:  return 4;
    case farmdec::VA_1D:  return 1;
    case farmdec::VA_2D:  return 2;
    }
    assert(false && "invalid vector arrangement");
}

Facet Lifter::FacetOf(farmdec::VectorArrangement va, bool fp) {
    switch (va) {
    case farmdec::VA_8B:  return Facet::Vnt(8, Facet::I8);
    case farmdec::VA_16B: return Facet::Vnt(16, Facet::I8);
    case farmdec::VA_4H:  return Facet::Vnt(4, Facet::I16);
    case farmdec::VA_8H:  return Facet::Vnt(8, Facet::I16);
    case farmdec::VA_2S:  return Facet::Vnt(2, (fp) ? Facet::F32 : Facet::I32);
    case farmdec::VA_4S:  return Facet::Vnt(4, (fp) ? Facet::F32 : Facet::I32);
    case farmdec::VA_1D:  return Facet::Vnt(1, (fp) ? Facet::F64 : Facet::I64);
    case farmdec::VA_2D:  return Facet::Vnt(2, (fp) ? Facet::F64 : Facet::I64);
    }
    assert(false && "invalid vector arrangement");
}

llvm::Type* Lifter::TypeOf(farmdec::VectorArrangement va, bool fp) {
    return FacetOf(va, fp).Type(irb.getContext());
}

llvm::Type* Lifter::ElemTypeOf(farmdec::VectorArrangement va, bool fp) {
    return llvm::cast<llvm::VectorType>(TypeOf(va, fp))->getElementType();
}

llvm::Value* Lifter::MinMax(llvm::Value* lhs, llvm::Value* rhs, bool sgn, bool min) {
    // XXX llvm.umin.* and friends not available in LLVM 11

    llvm::Value* sel_lhs = nullptr;
    if (min) {
        sel_lhs = (sgn) ? irb.CreateICmpSLT(lhs, rhs) : irb.CreateICmpULT(lhs, rhs);
    } else {
        sel_lhs = (sgn) ? irb.CreateICmpSGT(lhs, rhs) : irb.CreateICmpUGT(lhs, rhs);
    }
    return irb.CreateSelect(sel_lhs, lhs, rhs);
}

llvm::Value* Lifter::Abs(llvm::Value* v) {
    // |v| = (v < 0) ? -v : v;
    auto is_negative = irb.CreateICmpSLT(v, llvm::Constant::getNullValue(v->getType()));
    return irb.CreateSelect(is_negative, irb.CreateNeg(v), v);
}

// Lift SIMD instructions where operands and result have the same vector arrangement
// and can be implemented by a single LLVM binary operation.
void Lifter::LiftThreeSame(llvm::Instruction::BinaryOps op, farmdec::Reg rd, farmdec::VectorArrangement va, farmdec::Reg rn, farmdec::Reg rm, bool scalar, bool invert_rhs, bool fp) {
    auto lhs = (scalar) ? GetScalar(rn, fad_size_from_vec_arrangement(va), fp) : GetVec(rn, va, fp);
    auto rhs = (scalar) ? GetScalar(rm, fad_size_from_vec_arrangement(va), fp) : GetVec(rm, va, fp);
    if (invert_rhs) {
        rhs = irb.CreateNot(rhs);
    }
    auto val = irb.CreateBinOp(op, lhs, rhs);
    if (scalar)
        SetScalar(rd, val);
    else
        SetVec(rd, val);
}

// Lift the SIMD [F]CMxx intructions. There are variants that compare two
// registers Rn and Rm (zero == false) while others compare Rn to 0/0.0
// (zero == true).
void Lifter::LiftCmXX(llvm::CmpInst::Predicate cmp, farmdec::Reg rd, farmdec::VectorArrangement va, farmdec::Reg rn, farmdec::Reg rm, bool zero, bool fp) {
    auto srcty = TypeOf(va, fp); // may be float or int
    auto lhs = GetVec(rn, va);
    auto rhs = (zero) ? llvm::Constant::getNullValue(srcty) : GetVec(rm, va);

    auto dstty = llvm::VectorType::getInteger(llvm::cast<llvm::VectorType>(srcty)); // must be int
    auto is_true = irb.CreateCmp(cmp, lhs, rhs);
    auto val = irb.CreateSExt(is_true, dstty);

    SetVec(rd, val);
}

// Like LiftCmXX, but has scalar Dd, Dn, Dm operands.
void Lifter::LiftScalarCmXX(llvm::CmpInst::Predicate cmp, farmdec::Reg rd, farmdec::Reg rn, farmdec::Reg rm, bool zero, bool fp) {
    llvm::Value* zero_val = (fp) ? llvm::ConstantFP::get(irb.getDoubleTy(), 0.0) : irb.getInt64(0);

    auto lhs = GetScalar(rn, farmdec::FSZ_D, fp);
    auto rhs = (zero) ? zero_val : GetScalar(rm, farmdec::FSZ_D, fp);

    auto is_true = irb.CreateCmp(cmp, lhs, rhs);
    auto val = irb.CreateSExt(is_true, irb.getInt64Ty());

    SetScalar(rd, val);
}

// Transform SIMD pairwise vectors into a (lhs, rhs) pair allowing normal SIMD instructions.
//
// Unlike scalar pairwise instructions (ADDP), which simply reduce a two-element vector,
// these SIMD pairwise instructions take the input vectors, concatenate them, and apply a binary
// operation for each pair of adjacent elements.
//
//     Vn: [a,b,c,d]
//     Vm: [u,v,w,x]
//  Vm:Vn: [a,b,c,d,u,v,w,x] (yes, Vn is after Vm)
//     Vd: [a+b,c+d,u+v,w+x] ('+' stands for binary op, e.g. addition, max/min)
//
// This can be expressed using two shuffled vectors and then normal vector binary operation:
//
//     lhs: [a,c,u,w] (even elements of Vn:Vm, starting with i=0)
//     rhs: [b,d,v,x] (odd elements)
// lhs+rhs: [a+b,c+d,u+v,w+x]
//
void Lifter::TransformSIMDPairwise(farmdec::VectorArrangement va, farmdec::Reg rn, farmdec::Reg rm, llvm::Value **lhs, llvm::Value** rhs, bool fp) {


    auto vn = GetVec(rn, va, fp);
    auto vm = GetVec(rm, va, fp);

    llvm::SmallVector<int, 16> odd, even;
    unsigned nelem = NumElem(va);
    for (unsigned i = 0; i < 2*nelem; i++) {
        if ((i % 2) == 0)
            even.push_back(i);
        else
            odd.push_back(i);
     }

    *lhs = irb.CreateShuffleVector(vm, vn, even);
    *rhs = irb.CreateShuffleVector(vm, vn, odd);
}

} // namespace rellume::aarch64

/**
 * @}
 **/
