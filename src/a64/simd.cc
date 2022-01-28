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

static llvm::SmallVector<int, 16> even(unsigned n) {
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < n; i++) {
        mask.push_back(2*i);
    }
    return mask;
}

static llvm::SmallVector<int, 16> odd(unsigned n) {
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < n; i++) {
        mask.push_back(2*i + 1);
    }
    return mask;
}

// [0, 0+n, 1, 1+n, ..., n/2-1]. For ZIP1, ST2.
static llvm::SmallVector<int, 16> zipmask_lower(unsigned n) {
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < n/2; i++) {
        mask.push_back(i);
        mask.push_back(i + n);
    }
    return mask;
}

// [n/2+0, n/2+0+n, n/2 + 1, n/2 + 1+n, ...]. For ZIP2, ST2.
static llvm::SmallVector<int, 16> zipmask_upper(unsigned n) {
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < n/2; i++) {
        mask.push_back(n/2 + i);
        mask.push_back(n/2 + i + n);
    }
    return mask;
}

// Reverse m runs of n elements: [a0,a1,a2,a3, b0,b1,b2,b3] -> [a3,a2,a1,a0, b3,b2,b1,b0]
// where n = 4 (subscripts 0,1,2,3) and m = 2 (a,b).
static llvm::SmallVector<int, 16> reverse(unsigned n, unsigned m) {
    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < m; i++) {
        for (unsigned j = 0; j < n; j++) {
            mask.push_back(i*n + (n-j-1));
        }
    }
    return mask;
}

bool Lifter::LiftSIMD(farmdec::Inst a64) {
    bool round = a64.flags & farmdec::SIMD_ROUND;
    bool sgn = a64.flags & farmdec::SIMD_SIGNED;
    bool scalar = a64.flags & farmdec::SIMD_SCALAR;
    farmdec::VectorArrangement va = fad_get_vec_arrangement(a64.flags);
    bool w32 = a64.flags & farmdec::W32;

    // In the order of farmdec::Op.
    switch (a64.op) {
    case farmdec::A64_LD1_MULT: {
        auto vecty = TypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, vecty);
        farmdec::Reg tt = a64.rt;

        for (unsigned i = 0; i < a64.simd_ldst.nreg; i++) {
            auto ptr = irb.CreateConstGEP1_64(vecty, addr, i);
            auto v = irb.CreateLoad(vecty, ptr);
            SetVec(tt, v);
            tt = (tt+1) % 32; // wrap around V31..V0
        }
        break;
    }
    case farmdec::A64_LD2_MULT: {
        // Load as many two-element pairs (x_i, y_i) as there are elements in a vector,
        // then deinterleave, so that we have one vector X := (x_0, x_1, ...) and one
        // vector Y := (y_0, y_1, ...).
        auto vecty = TypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, vecty);
        auto p0 = irb.CreateLoad(vecty, addr);
        auto p1 = irb.CreateLoad(vecty, irb.CreateConstGEP1_64(vecty, addr, 1));

        unsigned nelem = NumElem(va);
        auto x = irb.CreateShuffleVector(p0, p1, even(nelem));
        auto y = irb.CreateShuffleVector(p0, p1, odd(nelem));
        SetVec(a64.rt, x);
        SetVec((a64.rt+1) % 32, y);
        break;
    }
    case farmdec::A64_ST1_MULT: {
        auto vecty = TypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, vecty);
        farmdec::Reg tt = a64.rt;
        switch (a64.simd_ldst.nreg) {
        case 1: StoreMulti(addr, GetVec(tt, va)); break;
        case 2: StoreMulti(addr, GetVec(tt, va), GetVec((tt+1)%32, va)); break;
        case 3: StoreMulti(addr, GetVec(tt, va), GetVec((tt+1)%32, va), GetVec((tt+2)%32, va)); break;
        case 4: StoreMulti(addr, GetVec(tt, va), GetVec((tt+1)%32, va), GetVec((tt+2)%32, va), GetVec((tt+3)%32, va)); break;
        default:
            assert(false && "bad #regs for ST1");
        }
        break;
    }
    case farmdec::A64_ST2_MULT: {
        // Given X := [x_0, x_1, ...], Y := [y_0, y_1, ...], zip them into pairs p0, p1 and store them.
        auto vecty = TypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, vecty);
        auto x = GetVec(a64.rt, va);
        auto y = GetVec((a64.rt+1) % 32, va);

        unsigned nelem = NumElem(va);
        auto p0 = irb.CreateShuffleVector(x, y, zipmask_lower(nelem));
        auto p1 = irb.CreateShuffleVector(x, y, zipmask_upper(nelem));
        StoreMulti(addr, p0, p1);
        break;
    }
    case farmdec::A64_LD1_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        InsertElem(a64.rt, a64.simd_ldst.index, e0);
        break;
    }
    case farmdec::A64_LD2_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        auto e1 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 1));
        InsertElem(a64.rt, a64.simd_ldst.index, e0);
        InsertElem((a64.rt+1) % 32, a64.simd_ldst.index, e1);
        break;
    }
    case farmdec::A64_LD3_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        auto e1 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 1));
        auto e2 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 2));
        InsertElem(a64.rt, a64.simd_ldst.index, e0);
        InsertElem((a64.rt+1) % 32, a64.simd_ldst.index, e1);
        InsertElem((a64.rt+2) % 32, a64.simd_ldst.index, e2);
        break;
    }
    case farmdec::A64_LD4_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        auto e1 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 1));
        auto e2 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 2));
        auto e3 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 3));
        InsertElem(a64.rt, a64.simd_ldst.index, e0);
        InsertElem((a64.rt+1) % 32, a64.simd_ldst.index, e1);
        InsertElem((a64.rt+2) % 32, a64.simd_ldst.index, e2);
        InsertElem((a64.rt+3) % 32, a64.simd_ldst.index, e3);
        break;
    }
    case farmdec::A64_ST1_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = GetElem(a64.rt, va, a64.simd_ldst.index);
        StoreMulti(addr, e0);
        break;
    }
    case farmdec::A64_ST2_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = GetElem(a64.rt, va, a64.simd_ldst.index);
        auto e1 = GetElem((a64.rt+1) % 32, va, a64.simd_ldst.index);
        StoreMulti(addr, e0, e1);
        break;
    }
    case farmdec::A64_ST3_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = GetElem(a64.rt, va, a64.simd_ldst.index);
        auto e1 = GetElem((a64.rt+1) % 32, va, a64.simd_ldst.index);
        auto e2 = GetElem((a64.rt+2) % 32, va, a64.simd_ldst.index);
        StoreMulti(addr, e0, e1, e2);
        break;
    }
    case farmdec::A64_ST4_SINGLE: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = GetElem(a64.rt, va, a64.simd_ldst.index);
        auto e1 = GetElem((a64.rt+1) % 32, va, a64.simd_ldst.index);
        auto e2 = GetElem((a64.rt+2) % 32, va, a64.simd_ldst.index);
        auto e3 = GetElem((a64.rt+3) % 32, va, a64.simd_ldst.index);
        StoreMulti(addr, e0, e1, e2, e3);
        break;
    }
    case farmdec::A64_LD1R: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        Dup(a64.rt, va, e0);
        break;
    }
    case farmdec::A64_LD2R: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        auto e1 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 1));
        Dup(a64.rt, va, e0);
        Dup((a64.rt+1) % 32, va, e1);
        break;
    }
    case farmdec::A64_LD3R: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        auto e1 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 1));
        auto e2 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 2));
        Dup(a64.rt, va, e0);
        Dup((a64.rt+1) % 32, va, e1);
        Dup((a64.rt+2) % 32, va, e2);
        break;
    }
    case farmdec::A64_LD4R: {
        auto ty = ElemTypeOf(fad_get_vec_arrangement(a64.flags));
        auto addr = SIMDLoadStoreAddr(a64, ty);
        auto e0 = irb.CreateLoad(ty, addr);
        auto e1 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 1));
        auto e2 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 2));
        auto e3 = irb.CreateLoad(ty, irb.CreateConstGEP1_64(ty, addr, 3));
        Dup(a64.rt, va, e0);
        Dup((a64.rt+1) % 32, va, e1);
        Dup((a64.rt+2) % 32, va, e2);
        Dup((a64.rt+3) % 32, va, e3);
        break;
    }
    case farmdec::A64_FCVT_VEC: {
        if (scalar) {
            farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
            auto fp = GetScalar(a64.rn, prec);

            // Fixed-point, so move binary point by multiplying by 2^{fbits}.
            if (a64.fcvt.fbits > 0) {
                auto scale = llvm::ConstantFP::get(TypeOf(prec), pow(2.0, (double) a64.fcvt.fbits));
                fp = irb.CreateFMul(fp, scale);
            }

            auto rounded = Round(fp, static_cast<farmdec::FPRounding>(a64.fcvt.mode));
            auto ity = (prec == farmdec::FSZ_D) ? irb.getInt64Ty() : irb.getInt32Ty();
            auto ival = (a64.fcvt.sgn) ? irb.CreateFPToSI(rounded, ity) : irb.CreateFPToUI(rounded, ity);
            SetScalar(a64.rd, ival);
        } else {
            auto fpvec = GetVec(a64.rn, va, /*fp=*/true);

            // Fixed-point, so move binary point by multiplying by 2^{fbits}.
            if (a64.fcvt.fbits > 0) {
                farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
                auto scale = llvm::ConstantFP::get(TypeOf(prec), pow(2.0, (double) a64.fcvt.fbits));
                auto scalevec = irb.CreateVectorSplat(NumElem(va), scale);
                fpvec = irb.CreateFMul(fpvec, scalevec);
            }

            auto rounded = Round(fpvec, static_cast<farmdec::FPRounding>(a64.fcvt.mode));
            auto ity = llvm::VectorType::getInteger(llvm::cast<llvm::VectorType>(fpvec->getType()));
            auto ivec = (a64.fcvt.sgn) ? irb.CreateFPToSI(rounded, ity) : irb.CreateFPToUI(rounded, ity);
            SetVec(a64.rd, ivec);
        }
        break;
    }
    case farmdec::A64_CVTF_VEC:
        if (scalar) {
            farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
            auto ival = GetScalar(a64.rn, prec, /*fp=*/false);
            auto fp = (a64.fcvt.sgn) ? irb.CreateSIToFP(ival, TypeOf(prec)) : irb.CreateUIToFP(ival, TypeOf(prec));

            // Fixed-point, so move binary point right by dividing by 2^{fbits}.
            if (a64.fcvt.fbits > 0) {
                auto scale = llvm::ConstantFP::get(TypeOf(prec), pow(2.0, (double) a64.fcvt.fbits));
                fp = irb.CreateFDiv(fp, scale);
            }

            SetScalar(a64.rd, fp);
        } else {
            auto ivec = GetVec(a64.rn, va);
            auto fpty = TypeOf(va, /*fp=*/true);
            auto fp = (a64.fcvt.sgn) ? irb.CreateSIToFP(ivec, fpty) : irb.CreateUIToFP(ivec, fpty);

            // Fixed-point, so move binary point right by dividing by 2^{fbits}.
            if (a64.fcvt.fbits > 0) {
                farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
                auto scale = llvm::ConstantFP::get(TypeOf(prec), pow(2.0, (double) a64.fcvt.fbits));
                auto scalevec = irb.CreateVectorSplat(NumElem(va), scale);
                fp = irb.CreateFDiv(fp, scalevec);
            }

            SetVec(a64.rd, fp);
        }
        break;
    case farmdec::A64_FRINT_VEC:
    case farmdec::A64_FRINTX_VEC: {
        assert(a64.frint.bits == 0); // XXX frint32*, frint64* currently not supported

        bool exact = (a64.op == farmdec::A64_FRINTX_VEC);
        SetVec(a64.rd, Round(GetVec(a64.rn, va, /*fp=*/true), static_cast<farmdec::FPRounding>(a64.frint.mode), exact));
        break;
    }
    case farmdec::A64_FCMEQ_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OEQ, a64.rd, a64.rn, a64.rm, /*zero=*/false, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OEQ, a64.rd, va, a64.rn, a64.rm, /*zero=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FCMEQ_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OEQ, a64.rd, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OEQ, a64.rd, va, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        break;
    case farmdec::A64_FCMGE_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OGE, a64.rd, a64.rn, a64.rm, /*zero=*/false, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OGE, a64.rd, va, a64.rn, a64.rm, /*zero=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FCMGE_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OGE, a64.rd, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OGE, a64.rd, va, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        break;
    case farmdec::A64_FCMGT_REG:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OGT, a64.rd, a64.rn, a64.rm, /*zero=*/false, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OGT, a64.rd, va, a64.rn, a64.rm, /*zero=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FCMGT_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OGT, a64.rd, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OGT, a64.rd, va, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        break;
    case farmdec::A64_FCMLE_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OLE, a64.rd, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OLE, a64.rd, va, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        break;
    case farmdec::A64_FCMLT_ZERO:
        if (scalar)
            LiftScalarCmXX(llvm::CmpInst::Predicate::FCMP_OLT, a64.rd, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        else
            LiftCmXX(llvm::CmpInst::Predicate::FCMP_OLT, a64.rd, va, a64.rn, a64.rm, /*zero=*/true, /*fp=*/true);
        break;
    case farmdec::A64_FABS_VEC:
        SetVec(a64.rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::fabs, GetVec(a64.rn, va, /*fp=*/true)));
        break;
    case farmdec::A64_FABD_VEC: {
        auto lhs = GetVec(a64.rn, va, /*fp=*/true);
        auto rhs = GetVec(a64.rm, va, /*fp=*/true);
        auto diff = irb.CreateFSub(lhs, rhs);
        SetVec(a64.rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::fabs, diff));
        break;
    }
    case farmdec::A64_FNEG_VEC:
        SetVec(a64.rd, irb.CreateFNeg(GetVec(a64.rn, va, /*fp=*/true)));
        break;
    case farmdec::A64_FSQRT_VEC:
        SetVec(a64.rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, GetVec(a64.rn, va, /*fp=*/true)));
        break;
    case farmdec::A64_FMUL_ELEM:
        if (scalar) {
            farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
            auto lhs = GetScalar(a64.rn, prec);
            auto rhs = GetElem(a64.rm, va, a64.imm, /*fp=*/true);
            SetScalar(a64.rd, irb.CreateFMul(lhs, rhs));
        } else {
            LiftMulAccElem(a64, llvm::Instruction::FAdd, llvm::Constant::getNullValue(TypeOf(va, /*fp=*/true)), /*extend_long=*/false, /*fp=*/true);
        }
        break;
    case farmdec::A64_FMUL_VEC:
        LiftThreeSame(llvm::Instruction::FMul, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false, /*invert_rhs=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FDIV_VEC:
        LiftThreeSame(llvm::Instruction::FDiv, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false, /*invert_rhs=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FADD_VEC:
        LiftThreeSame(llvm::Instruction::FAdd, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false, /*invert_rhs=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FSUB_VEC:
        LiftThreeSame(llvm::Instruction::FSub, a64.rd, va, a64.rn, a64.rm, /*scalar=*/false, /*invert_rhs=*/false, /*fp=*/true);
        break;
    case farmdec::A64_FMLA_ELEM:
        if (scalar) {
            farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
            auto lhs = GetScalar(a64.rn, prec);
            auto rhs = GetElem(a64.rm, va, a64.imm, /*fp=*/true);
            auto acc = GetScalar(a64.rd, prec);
            SetScalar(a64.rd, irb.CreateFAdd(acc, irb.CreateFMul(lhs, rhs)));
        } else {
            LiftMulAccElem(a64, llvm::Instruction::FAdd, GetVec(a64.rd, va, /*fp=*/true), /*extend_long=*/false, /*fp=*/true);
        }
        break;
    case farmdec::A64_FMLA_VEC: LiftMulAcc(a64, llvm::Instruction::FAdd, GetVec(a64.rd, va, /*fp=*/true), /*extend_long=*/false, /*fp=*/true); break;
    case farmdec::A64_FMLS_ELEM:
        if (scalar) {
            farmdec::FPSize prec = fad_size_from_vec_arrangement(va);
            auto lhs = GetScalar(a64.rn, prec);
            auto rhs = GetElem(a64.rm, va, a64.imm, /*fp=*/true);
            auto acc = GetScalar(a64.rd, prec);
            SetScalar(a64.rd, irb.CreateFSub(acc, irb.CreateFMul(lhs, rhs)));
        } else {
            LiftMulAccElem(a64, llvm::Instruction::FSub, GetVec(a64.rd, va, /*fp=*/true), /*extend_long=*/false, /*fp=*/true);
        }
        break;
    case farmdec::A64_FMLS_VEC: LiftMulAcc(a64, llvm::Instruction::FSub, GetVec(a64.rd, va, /*fp=*/true), /*extend_long=*/false, /*fp=*/true); break;
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
    case farmdec::A64_REV16_VEC: {
        auto vn = GetVec(a64.rn, va);
        llvm::SmallVector<int, 16> mask = reverse(2, NumElem(va)/2); // i8 elems in 16-bit runs, [a0,a1,b0,b1, ...] -> [a1,a0,b1,b0, ...]
        SetVec(a64.rd, irb.CreateShuffleVector(vn, llvm::Constant::getNullValue(TypeOf(va)), mask));
        break;
    }
    case farmdec::A64_REV32_VEC: {
        auto vn = GetVec(a64.rn, va);
        llvm::SmallVector<int, 16> mask;

        switch (va) {
        case farmdec::VA_8B: case farmdec::VA_16B:
            mask = reverse(4, NumElem(va)/4); // i8 elems in 32-bit runs, [a0,a1,a2,a3,b0,b1,b2,b3] -> [a3,a2,a1,a0,b3,b2,b1,b0]
            break;
        case farmdec::VA_4H: case farmdec::VA_8H:
            mask = reverse(2, NumElem(va)/2); // i16 elems in 32-bit runs, [a0,a1,b0,b1] -> [a1,a0,b1,b0]
            break;
        default:
            assert(false && "bad vector arrangement for REV32");
        }

        SetVec(a64.rd, irb.CreateShuffleVector(vn, llvm::Constant::getNullValue(TypeOf(va)), mask));
        break;
    }
    case farmdec::A64_REV64_VEC: {
        auto vn = GetVec(a64.rn, va);
        llvm::SmallVector<int, 16> mask;

        switch (va) {
        case farmdec::VA_8B: case farmdec::VA_16B:
            mask = reverse(8, NumElem(va)/8); // i8 elems in 64-bit runs, [a0,a1,a2,a3,a4,a5,a6,a7, ...] -> [a7,a6,a5, ...]
            break;
        case farmdec::VA_4H: case farmdec::VA_8H:
            mask = reverse(4, NumElem(va)/4); // i16 elems in 64-bit runs, [a0,a1,a2,a3, ...] -> [a3,a2,a1,a0, ...]
            break;
        case farmdec::VA_2S: case farmdec::VA_4S:
            mask = reverse(2, NumElem(va)/2); // i32 elems in 64-bit runs, [a0,a1,b1,b2, ...] -> [a1,a0,b1,b0, ...]
            break;
        default:
            assert(false && "bad vector arrangement for REV32");
        }

        SetVec(a64.rd, irb.CreateShuffleVector(vn, llvm::Constant::getNullValue(TypeOf(va)), mask));
        break;
    }
    case farmdec::A64_SHL_IMM:
        if (scalar) {
            auto lhs = GetScalar(a64.rn, fad_size_from_vec_arrangement(va), /*fp=*/false);
            SetScalar(a64.rd, irb.CreateShl(lhs, a64.imm));
        } else {
            auto lhs = GetVec(a64.rn, va);
            unsigned bits = ElemTypeOf(va)->getPrimitiveSizeInBits();
            auto rhs = irb.CreateVectorSplat(NumElem(va), irb.getIntN(bits, a64.imm));
            SetVec(a64.rd, irb.CreateShl(lhs, rhs));
        }
        break;
    case farmdec::A64_SHL_REG: {
        assert(!round && "rshl not supported yet");
        llvm::Value* lhs = nullptr;
        llvm::Value* rhs = nullptr;
        llvm::Value* zero = nullptr;
        if (scalar) {
            lhs = GetScalar(a64.rn, farmdec::FSZ_D, /*fp=*/false);
            rhs = GetScalar(a64.rm, farmdec::FSZ_D, /*fp=*/false);
            zero = irb.getInt64(0);
        } else {
            lhs = GetVec(a64.rn, va);
            rhs = GetVec(a64.rm, va);
            zero = llvm::Constant::getNullValue(TypeOf(va));
        }

        // For every element, shift left if shift amount is positive, shift right otherwise.
        auto do_shift_left = irb.CreateICmpSGE(rhs, zero);
        auto shl = irb.CreateShl(lhs, rhs);
        auto shr = (sgn) ? irb.CreateAShr(lhs, irb.CreateNeg(rhs)) : irb.CreateLShr(lhs, irb.CreateNeg(rhs));
        auto res = irb.CreateSelect(do_shift_left, shl, shr);

        if (scalar) {
            SetScalar(a64.rd, res);
        } else {
            SetVec(a64.rd, res);
        }
        break;
    }
    case farmdec::A64_SHLL: { // XTL is an alias with shift #0
        auto vn_half = Halve(GetVec(a64.rn, va), va);
        farmdec::VectorArrangement dstva = DoubleWidth(va);

        auto extty = TypeOf(dstva);
        auto lhs = (sgn) ? irb.CreateSExt(vn_half, extty) : irb.CreateZExt(vn_half, extty);

        unsigned bits = ElemTypeOf(dstva)->getPrimitiveSizeInBits();
        auto rhs = irb.CreateVectorSplat(NumElem(dstva), irb.getIntN(bits, a64.imm));

        SetVec(a64.rd, irb.CreateShl(lhs, rhs));
        break;
    }
    case farmdec::A64_SHR:
        assert(!round && "rshr not supported yet");
        if (scalar) {
            auto lhs = GetScalar(a64.rn, fad_size_from_vec_arrangement(va), /*fp=*/false);
            SetScalar(a64.rd, (sgn) ? irb.CreateAShr(lhs, a64.imm) : irb.CreateLShr(lhs, a64.imm));
        } else {
            auto lhs = GetVec(a64.rn, va);
            unsigned bits = ElemTypeOf(va)->getPrimitiveSizeInBits();
            auto rhs = irb.CreateVectorSplat(NumElem(va), irb.getIntN(bits, a64.imm));
            SetVec(a64.rd, (sgn) ? irb.CreateAShr(lhs, rhs) : irb.CreateLShr(lhs, rhs));
        }
        break;
    case farmdec::A64_SHRN: {
        assert(!round && "rshrn not supported yet");
        farmdec::VectorArrangement srcva = DoubleWidth(va);

        auto lhs = GetVec(a64.rn, srcva);
        unsigned bits = ElemTypeOf(srcva)->getPrimitiveSizeInBits();
        auto rhs = irb.CreateVectorSplat(NumElem(srcva), irb.getIntN(bits, a64.imm));
        auto res = irb.CreateLShr(lhs, rhs);
        InsertInHalf(a64.rd, va, Narrow(res));
        break;
    }
    case farmdec::A64_SRA: {
        // Scalar mode exists, but only for FSZ_D, so we can simply handle it as VA_1D,
        // without adding a duplicative special case.
        if (scalar) {
            va = farmdec::VA_1D;
        }

        auto acc = GetVec(a64.rd, va);
        auto lhs = GetVec(a64.rn, va);

        // If rounding is active, add the round_const to make sure we round up to the next-higher
        // integer where appropriate, instead of truncating. Really, the same mechanism as for HADD,
        // except shifted up. Since adding the constant may overflow the elements, we need to extend
        // each vector element, i.e. the vector may blow up to 256 bits! We can avoid this in the
        // non-rounding case.
        if (round) {
            auto extty = llvm::VectorType::getExtendedElementVectorType(llvm::cast<llvm::VectorType>(TypeOf(va)));
            lhs = (sgn) ? irb.CreateSExt(lhs, extty) : irb.CreateZExt(lhs, extty);
            uint64_t round_const = 1 << (a64.imm - 1);
            lhs = irb.CreateAdd(lhs, irb.CreateVectorSplat(NumElem(va), irb.getIntN(extty->getScalarSizeInBits(), round_const)));
        }

        unsigned bits = lhs->getType()->getScalarSizeInBits();
        auto rhs = irb.CreateVectorSplat(NumElem(va), irb.getIntN(bits, a64.imm));
        auto shifted = (sgn) ? irb.CreateAShr(lhs, rhs) : irb.CreateLShr(lhs, rhs);
        SetVec(a64.rd, irb.CreateAdd(acc, irb.CreateTruncOrBitCast(shifted, TypeOf(va))));
        break;
    }
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
    case farmdec::A64_TBL:
    case farmdec::A64_TBX: {
        // TBL/TBX: "dynamic shufflevector". Given a table of 1..4 vector registers
        // and an index vector, construct an output vector. If the index is out of
        // range, TBL writes a 0 while TBX keeps the old value.
        //
        // We cannot use shufflevector, which requires a constant index vector.
        // Thus: concatenate input vectors into table, then use as many insertelement
        // as required by the target vector arrangement (either VA_8B or VA_16B).
        // May not be terribly efficient, but is functionally correct.

        // Table vectors always of arrangement VA_16B.
        auto zero = llvm::Constant::getNullValue(TypeOf(farmdec::VA_16B));
        llvm::SmallVector<int, 32> concat16; // 0, 1, ..., 2*16 -- for concatenating 16B and 16B
        llvm::SmallVector<int, 64> concat32; // 0, 1, ..., 4*16 -- for concatenating 32B and 32B
        for (unsigned i = 0; i < 32; i++) {
            concat16.push_back(i);
        }
        for (unsigned i = 0; i < 64; i++) {
            concat32.push_back(i);
        }

        // Construct table using concatenations. shufflevector only accepts input vectors
        // of matching size.
        llvm::Value* table = GetVec(a64.rn, farmdec::VA_16B);
        switch (a64.imm) {
        case 1:   // nothing to do, entire table already loaded
            break;
        case 2:   // r[n] :: r[n+1]
            table = irb.CreateShuffleVector(table, GetVec((a64.rn + 1) % 32, farmdec::VA_16B), concat16);
            break;
        case 3: { // (r[n] :: r[n+1]) :: (r[n+2] :: 0)
            auto upper = irb.CreateShuffleVector(GetVec((a64.rn + 2) % 32, farmdec::VA_16B), zero, concat16);
            auto lower = irb.CreateShuffleVector(table, GetVec((a64.rn + 1) % 32, farmdec::VA_16B), concat16);
            table = irb.CreateShuffleVector(lower, upper, concat32);
            break;
        }
        case 4: { // (r[n] :: r[n+1]) :: (r[n+2] :: r[n+3])
            auto upper = irb.CreateShuffleVector(GetVec((a64.rn + 2) % 32, farmdec::VA_16B), GetVec((a64.rn + 3) % 32, farmdec::VA_16B), concat16);
            auto lower = irb.CreateShuffleVector(table, GetVec((a64.rn + 1) % 32, farmdec::VA_16B), concat16);
            table = irb.CreateShuffleVector(lower, upper, concat32);
            break;
        }
        default:
            assert(false && "too many vectors for table");
        }

        bool retain_old_if_out_of_range = (a64.op == farmdec::A64_TBX);

        auto idxvec = GetVec(a64.rm, va);
        auto oldvec = GetVec(a64.rd, va);
        auto dstvec = oldvec;
        auto zero_elem = irb.getInt8(0);
        auto nentries = irb.getInt8(a64.imm * 16); // in table
        unsigned nelem = NumElem(va);

        // idx = idxvec[i];
        // out_of_range_val = (tbx) ? oldvec[i] : 0;
        // dstvec[i] = (idx >= sizeof(table)) ? out_of_range_val : table[idx];
        for (unsigned i = 0; i < nelem; i++) {
            auto idx = irb.CreateExtractElement(idxvec, i);
            // Note: this may be a poison value, if idx is out of range.
            // The interpreter complains about this, but as the value is not
            // propagated, this should actually be fine.
            auto tblval = irb.CreateExtractElement(table, idx);
            auto out_of_range_val = (retain_old_if_out_of_range) ? irb.CreateExtractElement(oldvec, i) : zero_elem;
            auto is_out_of_range = irb.CreateICmpUGE(idx, nentries);
            auto val = irb.CreateSelect(is_out_of_range, out_of_range_val, tblval);
            dstvec = irb.CreateInsertElement(dstvec, val, i);
        }

        SetVec(a64.rd, dstvec);
        break;
    }
    case farmdec::A64_UZP1: {
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);
        unsigned nelem = NumElem(va);
        SetVec(a64.rd, irb.CreateShuffleVector(vn, vm, even(nelem)));
        break;
    }
    case farmdec::A64_UZP2: {
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);
        unsigned nelem = NumElem(va);
        SetVec(a64.rd, irb.CreateShuffleVector(vn, vm, odd(nelem)));
        break;
    }
    case farmdec::A64_XTN:
        InsertInHalf(a64.rd, va, Narrow(GetVec(a64.rn, DoubleWidth(va))));
        break;
    case farmdec::A64_ZIP1: {
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);
        unsigned nelem = NumElem(va);
        SetVec(a64.rd, irb.CreateShuffleVector(vn, vm, zipmask_lower(nelem)));
        break;
    }
    case farmdec::A64_ZIP2: {
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);
        unsigned nelem = NumElem(va);
        SetVec(a64.rd, irb.CreateShuffleVector(vn, vm, zipmask_upper(nelem)));
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
    case farmdec::A64_CMTST: {
        // res = ((lhs & rhs) != 0) ? -1 : 0;
        auto lhs = (scalar) ? GetScalar(a64.rn, farmdec::FSZ_D, /*fp=*/false) : GetVec(a64.rn, va);
        auto rhs = (scalar) ? GetScalar(a64.rm, farmdec::FSZ_D, /*fp=*/false) : GetVec(a64.rm, va);
        auto lhs_and_rhs = irb.CreateAnd(lhs, rhs);
        auto ty = (scalar) ? irb.getInt64Ty() : TypeOf(va);
        auto is_true = irb.CreateICmpNE(lhs_and_rhs, llvm::Constant::getNullValue(ty));
        auto val = irb.CreateSExt(is_true, ty);
        if (scalar)
            SetScalar(a64.rd, val);
        else
            SetVec(a64.rd, val);
        break;
    }
    case farmdec::A64_ABS_VEC:
        SetVec(a64.rd, Abs(GetVec(a64.rn, va)));
        break;
    case farmdec::A64_ABD:
    case farmdec::A64_ABA: {
        auto acc = (a64.op == farmdec::A64_ABA) ? GetVec(a64.rd, va) : llvm::Constant::getNullValue(TypeOf(va));
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);

        // Need to extend to enough precision, see HADD.
        // Note that this may blow up these temporary vectors to 256 bits!
        auto extty = llvm::VectorType::getExtendedElementVectorType(llvm::cast<llvm::VectorType>(TypeOf(va)));
        auto lhs = (sgn) ? irb.CreateSExt(vn, extty) : irb.CreateZExt(vn, extty);
        auto rhs = (sgn) ? irb.CreateSExt(vm, extty) : irb.CreateZExt(vm, extty);

        auto diff = irb.CreateSub(lhs, rhs);
        auto absdiff = irb.CreateTrunc(Abs(diff), TypeOf(va));
        SetVec(a64.rd, irb.CreateAdd(acc, absdiff));
        break;
    }
    case farmdec::A64_ABDL:
    case farmdec::A64_ABAL: {
        auto extty = TypeOf(DoubleWidth(va));
        auto acc = (a64.op == farmdec::A64_ABAL) ? GetVec(a64.rd, DoubleWidth(va)) : llvm::Constant::getNullValue(extty);
        auto vn_half = Halve(GetVec(a64.rn, va), va);
        auto vm_half = Halve(GetVec(a64.rm, va), va);
        auto lhs = (sgn) ? irb.CreateSExt(vn_half, extty) : irb.CreateZExt(vn_half, extty);
        auto rhs = (sgn) ? irb.CreateSExt(vm_half, extty) : irb.CreateZExt(vm_half, extty);

        auto diff = irb.CreateSub(lhs, rhs);
        SetVec(a64.rd, irb.CreateAdd(acc, Abs(diff)));
        break;
    }
    case farmdec::A64_NEG_VEC:
        SetVec(a64.rd, irb.CreateNeg(GetVec(a64.rn, va)));
        break;
    case farmdec::A64_MUL_ELEM: LiftMulAccElem(a64, llvm::Instruction::Add, llvm::Constant::getNullValue(TypeOf(va)), /*extend_long=*/false); break;
    case farmdec::A64_MUL_VEC: LiftMulAcc(a64, llvm::Instruction::Add, llvm::Constant::getNullValue(TypeOf(va)), /*extend_long=*/false); break;
    case farmdec::A64_MULL_ELEM: LiftMulAccElem(a64, llvm::Instruction::Add, llvm::Constant::getNullValue(TypeOf(DoubleWidth(va))), /*extend_long=*/true); break;
    case farmdec::A64_MULL_VEC: LiftMulAcc(a64, llvm::Instruction::Add, llvm::Constant::getNullValue(TypeOf(DoubleWidth(va))), /*extend_long=*/true); break;
    case farmdec::A64_ADD_VEC:
        LiftThreeSame(llvm::Instruction::Add, a64.rd, va, a64.rn, a64.rm, scalar);
        break;
    case farmdec::A64_ADDL: {
        auto vn_half = Halve(GetVec(a64.rn, va), va);
        auto vm_half = Halve(GetVec(a64.rm, va), va);
        auto extty = TypeOf(DoubleWidth(va));
        auto lhs = (sgn) ? irb.CreateSExt(vn_half, extty) : irb.CreateZExt(vn_half, extty);
        auto rhs = (sgn) ? irb.CreateSExt(vm_half, extty) : irb.CreateZExt(vm_half, extty);
        SetVec(a64.rd, irb.CreateAdd(lhs, rhs));
        break;
    }
    case farmdec::A64_ADDW: {
        auto lhs = GetVec(a64.rn, DoubleWidth(va));
        auto vm_half = Halve(GetVec(a64.rm, va), va);
        auto extty = TypeOf(DoubleWidth(va));
        auto rhs = (sgn) ? irb.CreateSExt(vm_half, extty) : irb.CreateZExt(vm_half, extty);
        SetVec(a64.rd, irb.CreateAdd(lhs, rhs));
        break;
    }
    case farmdec::A64_HADD: {
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);

        // Need to extend to enough precision: for byte elements, hadd(255, 1) = 256/2 = 128 and not 0!
        // Note that this may blow up these temporary vectors to 256 bits!
        auto extty = llvm::VectorType::getExtendedElementVectorType(llvm::cast<llvm::VectorType>(TypeOf(va)));
        auto lhs = (sgn) ? irb.CreateSExt(vn, extty) : irb.CreateZExt(vn, extty);
        auto rhs = (sgn) ? irb.CreateSExt(vm, extty) : irb.CreateZExt(vm, extty);

        // Rounded halving: rhalve(x) = (x+1) / 2, rounding away instead of truncating towards zero.
        if (round) {
            rhs = irb.CreateAdd(rhs, irb.CreateVectorSplat(NumElem(va), irb.getIntN(extty->getScalarSizeInBits(), 1)));
        }
        auto sum = irb.CreateAdd(lhs, rhs);
        auto halved = (sgn) ? irb.CreateAShr(sum, 1) : irb.CreateLShr(sum, 1);
        SetVec(a64.rd, irb.CreateTrunc(halved, TypeOf(va)));
        break;
    }
    case farmdec::A64_SUB_VEC:
        LiftThreeSame(llvm::Instruction::Sub, a64.rd, va, a64.rn, a64.rm, scalar);
        break;
    case farmdec::A64_SUBL: {
        auto vn_half = Halve(GetVec(a64.rn, va), va);
        auto vm_half = Halve(GetVec(a64.rm, va), va);
        auto extty = TypeOf(DoubleWidth(va));
        auto lhs = (sgn) ? irb.CreateSExt(vn_half, extty) : irb.CreateZExt(vn_half, extty);
        auto rhs = (sgn) ? irb.CreateSExt(vm_half, extty) : irb.CreateZExt(vm_half, extty);
        SetVec(a64.rd, irb.CreateSub(lhs, rhs));
        break;
    }
    case farmdec::A64_SUBW: {
        auto lhs = GetVec(a64.rn, DoubleWidth(va));
        auto vm_half = Halve(GetVec(a64.rm, va), va);
        auto extty = TypeOf(DoubleWidth(va));
        auto rhs = (sgn) ? irb.CreateSExt(vm_half, extty) : irb.CreateZExt(vm_half, extty);
        SetVec(a64.rd, irb.CreateSub(lhs, rhs));
        break;
    }
    case farmdec::A64_HSUB: {
        auto vn = GetVec(a64.rn, va);
        auto vm = GetVec(a64.rm, va);

        // See HADD, we need enough precision.
        auto extty = llvm::VectorType::getExtendedElementVectorType(llvm::cast<llvm::VectorType>(TypeOf(va)));
        auto lhs = (sgn) ? irb.CreateSExt(vn, extty) : irb.CreateZExt(vn, extty);
        auto rhs = (sgn) ? irb.CreateSExt(vm, extty) : irb.CreateZExt(vm, extty);

        // Unlike HADD, there are no rounded variants of HSUB.
        auto diff = irb.CreateSub(lhs, rhs);
        auto halved = (sgn) ? irb.CreateAShr(diff, 1) : irb.CreateLShr(diff, 1);
        SetVec(a64.rd, irb.CreateTrunc(halved, TypeOf(va)));
        break;
    }
    case farmdec::A64_MAX_VEC:
        SetVec(a64.rd, MinMax(GetVec(a64.rn, va), GetVec(a64.rm, va), sgn, /*min=*/false));
        break;
    case farmdec::A64_MIN_VEC:
        SetVec(a64.rd, MinMax(GetVec(a64.rn, va), GetVec(a64.rm, va), sgn, /*min=*/true));
        break;
    case farmdec::A64_MLA_ELEM: LiftMulAccElem(a64, llvm::Instruction::Add, GetVec(a64.rd, va), /*extend_long=*/false); break;
    case farmdec::A64_MLA_VEC: LiftMulAcc(a64, llvm::Instruction::Add, GetVec(a64.rd, va), /*extend_long=*/false); break;
    case farmdec::A64_MLS_ELEM: LiftMulAccElem(a64, llvm::Instruction::Sub, GetVec(a64.rd, va), /*extend_long=*/false); break;
    case farmdec::A64_MLS_VEC: LiftMulAcc(a64, llvm::Instruction::Sub, GetVec(a64.rd, va), /*extend_long=*/false); break;
    case farmdec::A64_MLAL_ELEM: LiftMulAccElem(a64, llvm::Instruction::Add, GetVec(a64.rd, DoubleWidth(va)), /*extend_long=*/true); break;
    case farmdec::A64_MLAL_VEC: LiftMulAcc(a64, llvm::Instruction::Add, GetVec(a64.rd, DoubleWidth(va)), /*extend_long=*/true); break;
    case farmdec::A64_MLSL_ELEM: LiftMulAccElem(a64, llvm::Instruction::Sub, GetVec(a64.rd, DoubleWidth(va)), /*extend_long=*/true); break;
    case farmdec::A64_MLSL_VEC: LiftMulAcc(a64, llvm::Instruction::Sub, GetVec(a64.rd, DoubleWidth(va)), /*extend_long=*/true); break;
    case farmdec::A64_FADDP: {
        auto vec = GetVec(a64.rn, va, /*fp=*/true);
        auto lhs = irb.CreateExtractElement(vec, uint64_t{0});
        auto rhs = irb.CreateExtractElement(vec, uint64_t{1});
        SetScalar(a64.rd, irb.CreateFAdd(lhs, rhs));
        break;
    }
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
    case farmdec::A64_ADALP:
    case farmdec::A64_ADDLP: {
        // These instructions take _one_ source operand, extend it to double width,
        // then add its elements pairwise, so that the result has half the elements.
        farmdec::VectorArrangement dstva;
        switch (va) {
        case farmdec::VA_8B: dstva = farmdec::VA_4H; break;
        case farmdec::VA_16B: dstva = farmdec::VA_8H; break;
        case farmdec::VA_4H: dstva = farmdec::VA_2S; break;
        case farmdec::VA_8H: dstva = farmdec::VA_4S; break;
        case farmdec::VA_2S: dstva = farmdec::VA_1D; break;
        case farmdec::VA_4S: dstva = farmdec::VA_2D; break;
        default:
            assert(false && "bad ADDLP/ADALP source operand vector arrangement");
        }

        // Destination vector -> half the elements
        unsigned nelem_dst = NumElem(dstva);
        auto zero_dst = llvm::Constant::getNullValue(TypeOf(dstva));
        auto acc = (a64.op == farmdec::A64_ADALP) ? GetVec(a64.rd, dstva) : zero_dst;

        // Extended vector -> same number of elements, but of double the size
        auto extty = TypeOf(DoubleWidth(va));
        auto zero_ext = llvm::Constant::getNullValue(extty);
        auto vn = GetVec(a64.rn, va);
        auto extended = (sgn) ? irb.CreateSExt(vn, extty) : irb.CreateZExt(vn, extty);

        auto lhs = irb.CreateShuffleVector(extended, zero_ext, even(nelem_dst));
        auto rhs = irb.CreateShuffleVector(extended, zero_ext, odd(nelem_dst));
        SetVec(a64.rd, irb.CreateAdd(acc, irb.CreateAdd(lhs, rhs)));
        break;
    }
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
    // Always get element from full-sized vector.
    switch (va) {
    case farmdec::VA_8B: va = farmdec::VA_16B; break;
    case farmdec::VA_4H: va = farmdec::VA_8H; break;
    case farmdec::VA_2S: va = farmdec::VA_4S; break;
    case farmdec::VA_1D: va = farmdec::VA_2D; break;
    default: break;
    }
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

// If the vector arrangement va indicates a full vector, insert narrow into Vd's
// upper half without touching other bits. If va indicates a short vector, write
// narrow to Vd's lower half and clear the upper half.
//
// This is the behavior required for the instructions that narrow to a shorter element
// size, e.g. XTN, XTN2. The "2" suffix indicates inserting into the upper half, but the
// behavior can be derived from the vector arrangement just as well.  Consequently, there's
// only one farmdec::Op value A64_XTN.
void Lifter::InsertInHalf(farmdec::Reg rd, farmdec::VectorArrangement va, llvm::Value* narrow) {
    bool upper_half = (va == farmdec::VA_4S || va == farmdec::VA_8H || va == farmdec::VA_16B);

    // SetVec already clears upper half when inserting narrow vector.
    if (!upper_half) {
        SetVec(rd, narrow);
        return;
    }

    // Insert into upper half without modifying lower half.
    // (1) Extend narrow vector V into full vector V:0, to make shufflevector happy.
    // (2) Concatenate the the lower half L and V.
    auto old = GetVec(rd, va);
    unsigned nelem = NumElem(va);

    auto narrow_zero = llvm::Constant::getNullValue(narrow->getType());
    llvm::SmallVector<int, 16> extend;
    for (unsigned i = 0; i < nelem; i++) {
        extend.push_back(i);
    }
    auto extended = irb.CreateShuffleVector(narrow, narrow_zero, extend);

    llvm::SmallVector<int, 16> mask; // nelem = 8: 0, 1, 2, 3, 8, 9, 10, 11
    for (unsigned i = 0; i < nelem; i++) {
        mask.push_back((i < nelem/2) ? i : i + nelem/2);
    }
    SetVec(rd, irb.CreateShuffleVector(old, extended, mask));
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

// DoubleWidth returns the full-vector arrangement with double the element width.
// This is needed for Narrowing instructions.
//
// Examples:
//
//    xtn  v0.2s, v1.2d
//    xtn2 v0.4s, v1.2d
//
// xtn narrows the v1 elements and writes them into the lower half of v0 while
// xtn2 writes the two elements into the high half instead. Farmdec stores the
// more important "2s" and "4s" as the arrangement since "2d" can be derived
// using DoubleWidth.
farmdec::VectorArrangement Lifter::DoubleWidth(farmdec::VectorArrangement va) {
    switch (va) {
    case farmdec::VA_8B:  return farmdec::VA_8H;
    case farmdec::VA_16B: return farmdec::VA_8H;
    case farmdec::VA_4H:  return farmdec::VA_4S;
    case farmdec::VA_8H:  return farmdec::VA_4S;
    case farmdec::VA_2S:  return farmdec::VA_2D;
    case farmdec::VA_4S:  return farmdec::VA_2D;
    default:
        break;
    }
    assert(false && "vector arrangement cannot be doubled");
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

// Truncate every vector element to half its size.
llvm::Value* Lifter::Narrow(llvm::Value* v) {
    auto srcty = llvm::cast<llvm::VectorType>(v->getType());
    auto dstty = llvm::VectorType::getTruncatedElementVectorType(srcty);
    return irb.CreateTrunc(v, dstty);
}

// Return the lower or upper half of vector v of arrangement va.
llvm::Value* Lifter::Halve(llvm::Value* v, farmdec::VectorArrangement va) {
    // Long vector arrangements <==> upper half of the vector is used (mnemonic suffix 2, as in ADDW2, ADDL2, ...)
    bool upper_half = (va == farmdec::VA_4S || va == farmdec::VA_8H || va == farmdec::VA_16B);

    // A long vector arrangement <==> upper half, so we need to take nelem/2..nelem,
    // but conversely: lower half <==> already short vector arrangement, so take 0..nelem.
    llvm::SmallVector<int, 16> half;
    unsigned nelem = NumElem(va);
    unsigned start = (upper_half) ? nelem/2 : 0;     // inclusive
    unsigned end   = (upper_half) ? nelem   : nelem; // exclusive
    for (unsigned i = start; i < end; i++) {
        half.push_back(i);
    }

    return irb.CreateShuffleVector(v, llvm::Constant::getNullValue(v->getType()), half);
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
    auto lhs = GetVec(rn, va, fp);
    auto rhs = (zero) ? llvm::Constant::getNullValue(srcty) : GetVec(rm, va, fp);

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

// Lift a "normal" multiply-accumulate: acc ± (Vn * Vm). This implements MLA_VEC, MLS_VEC, MUL_VEC.
// If extend_long is true, only halves of Vn and Vm are loaded, extended to double width,
// multiply-accumulated, and stored to Vd. This implements MLAL_VEC, MLSL_VEC, MULL_VEC. In the
// same vein, if fp is true, this also covers FMLA_VEC, FMLS_VEC, and FMUL_VEC.
//
// If extend_long is true, acc must of the extended destination data type.
//
// (FMLAL_VEC, FMLSL_VEC use unsupported half-precision and are not handled).
void Lifter::LiftMulAcc(farmdec::Inst a64, llvm::Instruction::BinaryOps addsub, llvm::Value* acc, bool extend_long, bool fp) {
    farmdec::VectorArrangement va = fad_get_vec_arrangement(a64.flags);
    bool sgn = a64.flags & farmdec::SIMD_SIGNED;
    llvm::Value* lhs = nullptr;
    llvm::Value* rhs = nullptr;

    if (extend_long) {
        assert(!fp && "FMLAL, FMLSL not supported");

        auto vn_half = Halve(GetVec(a64.rn, va), va);
        auto vm_half = Halve(GetVec(a64.rm, va), va);
        auto extty = TypeOf(DoubleWidth(va));
        lhs = (sgn) ? irb.CreateSExt(vn_half, extty) : irb.CreateZExt(vn_half, extty);
        rhs = (sgn) ? irb.CreateSExt(vm_half, extty) : irb.CreateZExt(vm_half, extty);
    } else {
        lhs = GetVec(a64.rn, va, fp);
        rhs = GetVec(a64.rm, va, fp);
    }

    // XXX use fmuladd intrinsic where possible

    llvm::Value* product = (fp) ? irb.CreateFMul(lhs, rhs) : irb.CreateMul(lhs, rhs);
    SetVec(a64.rd, irb.CreateBinOp(addsub, acc, product)); // acc ± (lhs * rhs)
}

// Lift a multiply-accumulate with element: acc ± (Vn * Vm[i]). This alone implements MLA_ELEM,
// MLS_ELEM, MUL_ELEM. If extend_long is true, only halves of Vn and Vm are loaded, extended to
// double width, multiply-accumulated, and stored to Vd. This implements MLAL_ELEM, MLSL_ELEM,
// MULL_ELEM. In the same vein, if fp is true, this also covers FMLA_ELEM, FMLS_ELEM, FMUL_ELEM
// in their non-scalar variants.
//
// If extend_long is true, acc must of the extended destination data type.
//
// (FMLAL_ELEM, FMLSL_ELEM use unsupported half-precision and are not handled).
void Lifter::LiftMulAccElem(farmdec::Inst a64, llvm::Instruction::BinaryOps addsub, llvm::Value* acc, bool extend_long, bool fp) {
    farmdec::VectorArrangement va = fad_get_vec_arrangement(a64.flags);
    bool sgn = a64.flags & farmdec::SIMD_SIGNED;
    llvm::Value* lhs = nullptr;
    llvm::Value* rhs = nullptr;

    if (extend_long) {
        assert(!fp && "FMLAL, FMLSL not supported");

        auto vn_half = Halve(GetVec(a64.rn, va), va);
        auto elem = GetElem(a64.rm, va, a64.imm);
        // See Halve() for a discussion of NumElem differences.
        bool upper_half = (va == farmdec::VA_4S || va == farmdec::VA_8H || va == farmdec::VA_16B);
        unsigned nelem = upper_half ? NumElem(va)/2 : NumElem(va);
        auto elemsplat = irb.CreateVectorSplat(nelem, elem);
        auto extty = TypeOf(DoubleWidth(va));
        lhs = (sgn) ? irb.CreateSExt(vn_half, extty) : irb.CreateZExt(vn_half, extty);
        rhs = (sgn) ? irb.CreateSExt(elemsplat, extty) : irb.CreateZExt(elemsplat, extty);
    } else {
        lhs = GetVec(a64.rn, va, fp);
        auto elem = GetElem(a64.rm, va, a64.imm, fp);
        rhs = irb.CreateVectorSplat(NumElem(va), elem);

    }

    // XXX use fmuladd intrinsic where possible

    llvm::Value* product = (fp) ? irb.CreateFMul(lhs, rhs) : irb.CreateMul(lhs, rhs);
    SetVec(a64.rd, irb.CreateBinOp(addsub, acc, product)); // acc ± (lhs * rhs)
}

// Transform SIMD pairwise vectors into a (lhs, rhs) pair allowing normal SIMD instructions.
//
// Unlike scalar pairwise instructions (ADDP), which simply reduce a two-element vector,
// these SIMD pairwise instructions take the input vectors, concatenate them, and apply a binary
// operation for each pair of adjacent elements.
//
//     Vn: [a,b,c,d]
//     Vm: [u,v,w,x]
//  Vm:Vn: [a,b,c,d,u,v,w,x]
//     Vd: [a+b,c+d,u+v,w+x] ('+' stands for binary op, e.g. addition, max/min)
//
// This can be expressed using two shuffled vectors and then normal vector binary operation:
//
//     lhs: [a,c,u,w] (even elements of Vm:Vn, starting with i=0)
//     rhs: [b,d,v,x] (odd elements)
// lhs+rhs: [a+b,c+d,u+v,w+x]
//
void Lifter::TransformSIMDPairwise(farmdec::VectorArrangement va, farmdec::Reg rn, farmdec::Reg rm, llvm::Value **lhs, llvm::Value** rhs, bool fp) {
    auto vn = GetVec(rn, va, fp);
    auto vm = GetVec(rm, va, fp);
    unsigned nelem = NumElem(va);
    *lhs = irb.CreateShuffleVector(vn, vm, even(nelem));
    *rhs = irb.CreateShuffleVector(vn, vm, odd(nelem));
}

void Lifter::StoreMulti(llvm::Value* addr, llvm::Value* v0) {
    irb.CreateStore(v0, addr);
}

void Lifter::StoreMulti(llvm::Value* addr, llvm::Value* v0, llvm::Value* v1) {
    irb.CreateStore(v0, addr);
    irb.CreateStore(v1, irb.CreateConstGEP1_64(v0->getType(), addr, 1));
}

void Lifter::StoreMulti(llvm::Value* addr, llvm::Value* v0, llvm::Value* v1, llvm::Value* v2) {
    irb.CreateStore(v0, addr);
    irb.CreateStore(v1, irb.CreateConstGEP1_64(v0->getType(), addr, 1));
    irb.CreateStore(v2, irb.CreateConstGEP1_64(v0->getType(), addr, 2));
}

void Lifter::StoreMulti(llvm::Value* addr, llvm::Value* v0, llvm::Value* v1, llvm::Value* v2, llvm::Value* v3) {
    irb.CreateStore(v0, addr);
    irb.CreateStore(v1, irb.CreateConstGEP1_64(v0->getType(), addr, 1));
    irb.CreateStore(v2, irb.CreateConstGEP1_64(v0->getType(), addr, 2));
    irb.CreateStore(v3, irb.CreateConstGEP1_64(v0->getType(), addr, 3));
}

// SIMDLoadStoreAddr returns the base address of the SIMD LDx/STx (x=1,2,3,4)
// load/store. We cannot use the usual Addr because these instructions have a
// post-increment mode distinct from the usual one, and store their immediate
// in a64.simd_ldst.offset instead of a64.offset.
llvm::Value* Lifter::SIMDLoadStoreAddr(farmdec::Inst a64, llvm::Type* ty) {
    farmdec::AddrMode mode = fad_get_addrmode(a64.flags);
    switch (mode) {
    case farmdec::AM_SIMPLE:
        return Addr(ty, a64.rn);
    case farmdec::AM_POST: {
        auto base = Addr(ty, a64.rn);

        // Offset/increment can be an immediate or the Rm register.
        auto offset = (a64.rm == farmdec::ZERO_REG) ? irb.getInt64(a64.simd_ldst.offset) : GetGp(a64.rm, /*w32=*/false);
        SetGp(a64.rn, /*w32=*/false, irb.CreateAdd(GetGp(a64.rn, /*w32=*/false), offset)); // rn += offset

        return base;
    }
    default:
       assert(false && "bad LDx/STx addrmode");
    }

    assert("can't happen");
}

} // namespace rellume::aarch64

/**
 * @}
 **/
