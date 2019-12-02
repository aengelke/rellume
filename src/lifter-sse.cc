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

#include "lifter.h"

#include "facet.h"
#include "rellume/instr.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>
#include <algorithm>



/**
 * \defgroup LLInstructionSSE SSE Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

namespace rellume {

void Lifter::LiftFence(const LLInstr& inst) {
    // TODO: distinguish also lfence and sfence.
    irb.CreateFence(llvm::AtomicOrdering::SequentiallyConsistent);
}

void Lifter::LiftPrefetch(const LLInstr& inst, unsigned rw, unsigned locality) {
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    auto id = llvm::Intrinsic::prefetch;
    llvm::Function* intrinsic = llvm::Intrinsic::getDeclaration(module, id, {});

    llvm::Value* addr = OpAddr(inst.ops[0], irb.getInt8Ty());
    // Prefetch addr for read/write with given locality into the data cache.
    irb.CreateCall(intrinsic, {addr, irb.getInt32(rw), irb.getInt32(locality),
                               irb.getInt32(1)});
}

void Lifter::LiftFxsave(const LLInstr& inst) {
    llvm::Value* buf = OpAddr(inst.ops[0], irb.getInt8Ty());
    llvm::Module* mod = irb.GetInsertBlock()->getModule();
    irb.CreateAlignmentAssumption(mod->getDataLayout(), buf, 16);

    // Zero FPU status
    // TODO: FCW=0x37f, MXCSR=0x1f80, MXCSR_MASK=0xffff
    irb.CreateMemSet(buf, irb.getInt8(0), 0xa0, 16);
    for (unsigned i = 0; i < 16; i++) {
        llvm::Value* ptr = irb.CreateConstGEP1_32(buf, 0xa0 + 0x10*i);
        ptr = irb.CreatePointerCast(ptr, irb.getIntNTy(128)->getPointerTo());
        irb.CreateStore(GetReg(LLReg(LL_RT_XMM, i), Facet::I128), ptr);
    }
}

void Lifter::LiftFxrstor(const LLInstr& inst) {
    llvm::Value* buf = OpAddr(inst.ops[0], irb.getInt8Ty());
    llvm::Module* mod = irb.GetInsertBlock()->getModule();
    irb.CreateAlignmentAssumption(mod->getDataLayout(), buf, 16);

    for (unsigned i = 0; i < 16; i++) {
        llvm::Value* ptr = irb.CreateConstGEP1_32(buf, 0xa0 + 0x10*i);
        ptr = irb.CreatePointerCast(ptr, irb.getIntNTy(128)->getPointerTo());
        SetReg(LLReg(LL_RT_XMM, i), Facet::I128, irb.CreateLoad(ptr));
    }
}

void Lifter::LiftFstcw(const LLInstr& inst) {
    OpStoreGp(inst.ops[0], irb.getInt16(0x37f));
}

void Lifter::LiftFstsw(const LLInstr& inst) {
    OpStoreGp(inst.ops[0], irb.getInt16(0));
}

void Lifter::LiftStmxcsr(const LLInstr& inst) {
    OpStoreGp(inst.ops[0], irb.getInt32(0x1f80));
}

void Lifter::LiftSseMovq(const LLInstr& inst, Facet type)
{
    llvm::Value* op1 = OpLoad(inst.ops[1], type);
    if (inst.ops[0].type == LL_OP_REG && inst.ops[0].reg.IsVec()) {
        llvm::Type* el_ty = op1->getType();
        llvm::Type* vector_ty = llvm::VectorType::get(el_ty, 128 / el_ty->getPrimitiveSizeInBits());
        llvm::Value* zero = llvm::Constant::getNullValue(vector_ty);
        llvm::Value* zext = irb.CreateInsertElement(zero, op1, 0ul);
        OpStoreVec(inst.ops[0], zext);
    } else {
        OpStoreGp(inst.ops[0], op1);
    }
}

void Lifter::LiftSseMovScalar(const LLInstr& inst, Facet facet) {
    llvm::Value* src = OpLoad(inst.ops[1], facet);
    if (inst.ops[1].type == LL_OP_MEM) {
        llvm::Type* el_ty = src->getType();
        llvm::Type* vector_ty = llvm::VectorType::get(el_ty, 128 / el_ty->getPrimitiveSizeInBits());
        llvm::Value* zero = llvm::Constant::getNullValue(vector_ty);
        llvm::Value* zext = irb.CreateInsertElement(zero, src, 0ul);
        OpStoreVec(inst.ops[0], zext);
    } else {
        OpStoreVec(inst.ops[0], src);
    }
}

void Lifter::LiftSseMovdq(const LLInstr& inst, Facet facet,
                           Alignment alignment) {
    OpStoreVec(inst.ops[0], OpLoad(inst.ops[1], facet, alignment), alignment);
}

void Lifter::LiftSseMovlp(const LLInstr& inst) {
    if (inst.ops[0].type == LL_OP_REG && inst.ops[1].type == LL_OP_REG) {
        // move high 64-bit from src to low 64-bit from dst
        assert(inst.type == LL_INS_MOVLPS); // the official mnemonic is MOVHLPS.
        llvm::Value* op2 = OpLoad(inst.ops[1], Facet::V4F32);
        llvm::Value* zero = llvm::Constant::getNullValue(op2->getType());
        OpStoreVec(inst.ops[0], irb.CreateShuffleVector(op2, zero, {2, 3}));
    } else {
        // move (low) 64-bit from src to (low) 64-bit from dst
        auto facet = inst.type == LL_INS_MOVLPS ? Facet::V2F32 : Facet::F64;
        OpStoreVec(inst.ops[0], OpLoad(inst.ops[1], facet));
    }
}

void Lifter::LiftSseMovhps(const LLInstr& inst) {
    if (inst.ops[0].type == LL_OP_MEM) {
        // move high 64-bit from src to (low) 64-bit from dst (in memory)
        llvm::Value* op2 = OpLoad(inst.ops[1], Facet::V4F32);
        llvm::Value* zero = llvm::Constant::getNullValue(op2->getType());
        llvm::Value* res = irb.CreateShuffleVector(op2, zero, {2, 3});
        OpStoreVec(inst.ops[0], res);
    } else {
        // move low 64-bit from src to high 64-bit from dst
        // for reg-reg, the official mnemonic is MOVLHPS.
        llvm::Value* op1 = OpLoad(inst.ops[0], Facet::V4F32);
        llvm::Value* op2 = OpLoad(inst.ops[1], Facet::V2F32);

        // first, enlarge op2 to full vector width
        llvm::Value* zero_half = llvm::Constant::getNullValue(op2->getType());
        op2 = irb.CreateShuffleVector(op2, zero_half, {0, 1, 2, 3});
        // and then shuffle the two operands together
        llvm::Value* res = irb.CreateShuffleVector(op1, op2, {0, 1, 4, 5});
        OpStoreVec(inst.ops[0], res);
    }
}

void Lifter::LiftSseMovhpd(const LLInstr& inst) {
    if (inst.ops[0].type == LL_OP_MEM) {
        // move high 64-bit from src to (low) 64-bit from dst (in memory)
        llvm::Value* op2 = OpLoad(inst.ops[1], Facet::V2F64);
        OpStoreVec(inst.ops[0], irb.CreateExtractElement(op2, 1u));
    } else {
        // move low 64-bit from src to high 64-bit from dst
        llvm::Value* op1 = OpLoad(inst.ops[0], Facet::V2F64);
        llvm::Value* op2 = OpLoad(inst.ops[1], Facet::F64);
        OpStoreVec(inst.ops[0], irb.CreateInsertElement(op1, op2, 1u));
    }
}

void Lifter::LiftSseBinOp(const LLInstr& inst, llvm::Instruction::BinaryOps op,
                           Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type, ALIGN_IMP);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_IMP);
    OpStoreVec(inst.ops[0], irb.CreateBinOp(op, op1, op2), /*avx=*/false,
               ALIGN_IMP);
}

void Lifter::LiftSseAndn(const LLInstr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    OpStoreVec(inst.ops[0], irb.CreateAnd(irb.CreateNot(op1), op2));
}

void Lifter::LiftSseComis(const LLInstr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type);
    SetFlag(Facet::ZF, irb.CreateFCmpUEQ(op1, op2));
    SetFlag(Facet::CF, irb.CreateFCmpULT(op1, op2));
    SetFlag(Facet::PF, irb.CreateFCmpUNO(op1, op2));
    SetFlag(Facet::AF, irb.getFalse());
    SetFlag(Facet::OF, irb.getFalse());
    SetFlag(Facet::SF, irb.getFalse());
}

void Lifter::LiftSseCmp(const LLInstr& inst, Facet op_type) {
    llvm::FCmpInst::Predicate pred;
    switch (inst.ops[2].val) {
    case 0: pred = llvm::FCmpInst::FCMP_OEQ; break; // EQ_OQ
    case 1: pred = llvm::FCmpInst::FCMP_OLT; break; // LT_OS
    case 2: pred = llvm::FCmpInst::FCMP_OLE; break; // LE_OS
    case 3: pred = llvm::FCmpInst::FCMP_UNO; break; // UNORD_Q
    case 4: pred = llvm::FCmpInst::FCMP_UNE; break; // NEQ_UQ
    case 5: pred = llvm::FCmpInst::FCMP_UGE; break; // NLT_US
    case 6: pred = llvm::FCmpInst::FCMP_UGT; break; // NLE_US
    case 7: pred = llvm::FCmpInst::FCMP_ORD; break; // ORD_Q
    }
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* eq = irb.CreateFCmp(pred, op1, op2);
    llvm::Type* cmp_ty = op1->getType();
    llvm::Type* res_ty;
    if (cmp_ty->isVectorTy()) {
        unsigned elem_cnt = cmp_ty->getVectorNumElements();
        unsigned elem_size = cmp_ty->getVectorElementType()->getScalarSizeInBits();
        res_ty = llvm::VectorType::get(irb.getIntNTy(elem_size), elem_cnt);
    } else {
        res_ty = irb.getIntNTy(cmp_ty->getScalarSizeInBits());
    }
    OpStoreVec(inst.ops[0], irb.CreateSExt(eq, res_ty));
}

void Lifter::LiftSseMinmax(const LLInstr& inst, llvm::CmpInst::Predicate pred,
                            Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* cmp = irb.CreateFCmp(pred, op1, op2);
    OpStoreVec(inst.ops[0], irb.CreateSelect(cmp, op1, op2));
}

void Lifter::LiftSseSqrt(const LLInstr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[1], op_type);
    OpStoreVec(inst.ops[0], CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, op1));
}

void Lifter::LiftSseCvt(const LLInstr& inst, Facet src_type, Facet dst_type) {
    if (dst_type == Facet::I)
        dst_type = dst_type.Resolve(inst.ops[0].size * 8);
    llvm::Type* dst_ty = dst_type.Type(irb.getContext());

    llvm::Value* src = OpLoad(inst.ops[1], src_type);
    auto cast_op = llvm::CastInst::getCastOpcode(src, true, dst_ty, true);
    llvm::Value* dst = irb.CreateCast(cast_op, src, dst_ty);

    if (dst_ty->isIntegerTy()) {
        OpStoreGp(inst.ops[0], dst);
    } else if (dst_ty->isFloatingPointTy()) {
        OpStoreVec(inst.ops[0], dst);
    } else if (dst_ty->isVectorTy()) {
        if (dst_ty->getPrimitiveSizeInBits() == 64) {
            // Zero upper half
            llvm::Value* zero_half = llvm::Constant::getNullValue(dst_ty);
            dst = irb.CreateShuffleVector(dst, zero_half, {0, 1, 2, 3});
        } else {
            assert(dst_ty->getPrimitiveSizeInBits() == 128);
        }
        OpStoreVec(inst.ops[0], dst);
    } else {
        assert(false && "invalid cvt target type");
    }
}

void Lifter::LiftSseUnpck(const LLInstr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type);
    // We always fetch 128 bits, as per SDM.
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* res = nullptr;
    if (inst.type == LL_INS_PUNPCKLBW)
        res = irb.CreateShuffleVector(op1, op2, {0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23});
    else if (inst.type == LL_INS_PUNPCKHBW)
        res = irb.CreateShuffleVector(op1, op2, {8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31});
    else if (inst.type == LL_INS_PUNPCKLWD)
        res = irb.CreateShuffleVector(op1, op2, {0, 8, 1, 9, 2, 10, 3, 11});
    else if (inst.type == LL_INS_PUNPCKHWD)
        res = irb.CreateShuffleVector(op1, op2, {4, 12, 5, 13, 6, 14, 7, 15});
    else if (inst.type == LL_INS_UNPCKLPS || inst.type == LL_INS_PUNPCKLDQ)
        res = irb.CreateShuffleVector(op1, op2, {0, 4, 1, 5});
    else if (inst.type == LL_INS_UNPCKLPD || inst.type == LL_INS_PUNPCKLQDQ)
        res = irb.CreateShuffleVector(op1, op2, {0, 2});
    else if (inst.type == LL_INS_UNPCKHPS || inst.type == LL_INS_PUNPCKHDQ)
        res = irb.CreateShuffleVector(op1, op2, {2, 6, 3, 7});
    else if (inst.type == LL_INS_UNPCKHPD || inst.type == LL_INS_PUNPCKHQDQ)
        res = irb.CreateShuffleVector(op1, op2, {1, 3});
    else
        assert(0);
    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSseShufpd(const LLInstr& inst) {
    uint32_t mask[2] = { inst.ops[2].val&1 ? 1u:0u, inst.ops[2].val&2 ? 3u:2u };
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::VF64);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::VF64, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(op1, op2, mask);
    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSseShufps(const LLInstr& inst) {
    uint32_t mask[4];
    for (int i = 0; i < 4; i++)
        mask[i] = (i < 2 ? 0 : 4) + ((inst.ops[2].val >> 2*i) & 3);
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::VF32);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::VF32, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(op1, op2, mask);
    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSsePshufd(const LLInstr& inst) {
    uint32_t mask[4];
    for (int i = 0; i < 4; i++)
        mask[i] = ((inst.ops[2].val >> 2*i) & 3);
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::VI32);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::VI32, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(op1, op2, mask);
    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSseInsertps(const LLInstr& inst) {
    int count_s = (inst.ops[2].val >> 6) & 3;
    int count_d = (inst.ops[2].val >> 4) & 3;
    int zmask = inst.ops[2].val & 0xf;

    llvm::Value* dst = OpLoad(inst.ops[0], Facet::V4F32);
    llvm::Value* src;
    // If src is a reg, extract element, otherwise load scalar from memory.
    if (inst.ops[1].type == LL_OP_REG) {
        src = OpLoad(inst.ops[1], Facet::V4F32);
        src = irb.CreateExtractElement(src, count_s);
    } else {
        src = OpLoad(inst.ops[1], Facet::F32);
    }

    dst = irb.CreateInsertElement(dst, src, count_d);

    if (zmask) {
        uint32_t mask[4];
        for (int i = 0; i < 4; i++)
            mask[i] = zmask & (1 << i) ? 4 : i;
        llvm::Value* zero = llvm::Constant::getNullValue(dst->getType());
        dst = irb.CreateShuffleVector(dst, zero, mask);
    }

    OpStoreVec(inst.ops[0], dst);
}

void Lifter::LiftSsePinsr(const LLInstr& inst, Facet vec_op, Facet src_op,
                          unsigned mask) {
    llvm::Value* dst = OpLoad(inst.ops[0], vec_op);
    llvm::Value* src = OpLoad(inst.ops[1], src_op);
    unsigned count = inst.ops[2].val & mask;
    OpStoreVec(inst.ops[0], irb.CreateInsertElement(dst, src, count));
}

void Lifter::LiftSsePshiftElement(const LLInstr& inst,
                                  llvm::Instruction::BinaryOps op,
                                  Facet op_type) {
    llvm::Value* src = OpLoad(inst.ops[0], op_type);
    llvm::Value* shift = OpLoad(inst.ops[1], Facet::I64);

    llvm::Type* elem_ty = src->getType()->getVectorElementType();
    unsigned elem_size = elem_ty->getIntegerBitWidth();
    unsigned elem_cnt = src->getType()->getVectorNumElements();

    // For arithmetical shifts, if shift >= elem_size, result is sign bit.
    // So, we max shift at elem_size-1
    if (op == llvm::Instruction::AShr) {
        llvm::Value* max = irb.getInt64(elem_size-1);
        shift = irb.CreateSelect(irb.CreateICmpUGT(shift, max), max, shift);
    }

    llvm::Value* shift_trunc = irb.CreateTrunc(shift, elem_ty);
    llvm::Value* shift_vec = irb.CreateVectorSplat(elem_cnt, shift_trunc);
    llvm::Value* res = irb.CreateBinOp(op, src, shift_vec);

    // For logical shifts, if shift >= elem_size, result is zero.
    if (op != llvm::Instruction::AShr) {
        llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
        llvm::Value* cmp = irb.CreateICmpULT(shift, irb.getInt64(elem_size));
        res = irb.CreateSelect(cmp, res, zero);
    }

    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSsePshiftBytes(const LLInstr& inst) {
    uint32_t shift = std::min(static_cast<uint32_t>(inst.ops[1].val), 16u);
    bool right = inst.type == LL_INS_PSRLDQ;
    uint32_t mask[16];
    for (int i = 0; i < 16; i++)
        mask[i] = i + (right ? shift : (16 - shift));
    llvm::Value* src = OpLoad(inst.ops[0], Facet::V16I8);
    llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
    if (right)
        OpStoreVec(inst.ops[0], irb.CreateShuffleVector(src, zero, mask));
    else
        OpStoreVec(inst.ops[0], irb.CreateShuffleVector(zero, src, mask));
}

void Lifter::LiftSsePavg(const LLInstr& inst, Facet op_type) {
    llvm::Value* src1 = OpLoad(inst.ops[0], op_type);
    llvm::Value* src2 = OpLoad(inst.ops[1], op_type);

    llvm::Type* elem_ty = src1->getType()->getVectorElementType();
    unsigned elem_size = elem_ty->getIntegerBitWidth();
    unsigned elem_cnt = src1->getType()->getVectorNumElements();

    llvm::Type* ext_ty = llvm::VectorType::get(irb.getIntNTy(elem_size*2), elem_cnt);
    llvm::Value* ones = irb.CreateVectorSplat(elem_cnt, irb.getIntN(elem_size*2, 1));
    llvm::Value* ext1 = irb.CreateZExt(src1, ext_ty);
    llvm::Value* ext2 = irb.CreateZExt(src2, ext_ty);
    llvm::Value* sum = irb.CreateAdd(irb.CreateAdd(ext1, ext2), ones);
    llvm::Value* res = irb.CreateTrunc(irb.CreateLShr(sum, ones), src1->getType());
    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSsePmulhw(const LLInstr& inst, llvm::Instruction::CastOps cast) {
    llvm::Value* src1 = OpLoad(inst.ops[0], Facet::VI16);
    llvm::Value* src2 = OpLoad(inst.ops[1], Facet::VI16);

    unsigned elem_cnt = src1->getType()->getVectorNumElements();
    llvm::Type* ext_ty = llvm::VectorType::get(irb.getInt32Ty(), elem_cnt);

    llvm::Value* ext1 = irb.CreateCast(cast, src1, ext_ty);
    llvm::Value* ext2 = irb.CreateCast(cast, src2, ext_ty);
    llvm::Value* mul = irb.CreateMul(ext1, ext2);
    llvm::Value* shift = irb.CreateVectorSplat(elem_cnt, irb.getInt32(16));
    llvm::Value* res = irb.CreateTrunc(irb.CreateLShr(mul, shift), src1->getType());
    OpStoreVec(inst.ops[0], res);
}

void Lifter::LiftSsePack(const LLInstr& inst, Facet src_type, bool sign) {
    llvm::Value* op1 = OpLoad(inst.ops[0], src_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], src_type, ALIGN_MAX);
    llvm::Type* src_ty = op1->getType();
    unsigned src_size = src_ty->getScalarSizeInBits();
    unsigned src_elems = src_ty->getVectorNumElements();
    unsigned dst_size = src_size / 2;
    unsigned dst_elems = src_elems * 2;
    auto* dst_ty = llvm::VectorType::get(irb.getIntNTy(dst_size), dst_elems);

    llvm::APInt min, max;
    if (sign) {
        min = llvm::APInt::getSignedMinValue(dst_size).sext(src_size);
        max = llvm::APInt::getSignedMaxValue(dst_size).sext(src_size);
    } else {
        min = llvm::APInt::getMinValue(dst_size).zext(src_size);
        max = llvm::APInt::getMaxValue(dst_size).zext(src_size);
    }

    llvm::Constant* minv = llvm::Constant::getIntegerValue(src_ty, min);
    llvm::Constant* maxv = llvm::Constant::getIntegerValue(src_ty, max);
    op1 = irb.CreateSelect(irb.CreateICmpSLT(op1, minv), minv, op1);
    op2 = irb.CreateSelect(irb.CreateICmpSLT(op2, minv), minv, op2);
    op1 = irb.CreateSelect(irb.CreateICmpSGT(op1, maxv), maxv, op1);
    op2 = irb.CreateSelect(irb.CreateICmpSGT(op2, maxv), maxv, op2);

    llvm::SmallVector<unsigned, 16> mask;
    for (unsigned i = 0; i < src_elems; i++)
        mask.push_back(i);
    for (unsigned i = 0; i < src_elems; i++)
        mask.push_back(i + src_elems);

    llvm::Value* shuffle = irb.CreateShuffleVector(op1, op2, mask);
    OpStoreVec(inst.ops[0], irb.CreateTrunc(shuffle, dst_ty));
}

void Lifter::LiftSsePcmp(const LLInstr& inst, llvm::CmpInst::Predicate pred,
                         Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* eq = irb.CreateICmp(pred, op1, op2);
    OpStoreVec(inst.ops[0], irb.CreateSExt(eq, op1->getType()));
}

void Lifter::LiftSsePminmax(const LLInstr& inst, llvm::CmpInst::Predicate pred,
                            Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* cmp = irb.CreateICmp(pred, op1, op2);
    OpStoreVec(inst.ops[0], irb.CreateSelect(cmp, op1, op2));
}

void Lifter::LiftSseMovmsk(const LLInstr& inst, Facet op_type) {
    llvm::Value* src = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
    llvm::Value* bitvec = irb.CreateICmpSLT(src, zero);
    unsigned bit_count = src->getType()->getVectorNumElements();
    llvm::Value* bits = irb.CreateBitCast(bitvec, irb.getIntNTy(bit_count));
    OpStoreGp(inst.ops[0], irb.CreateZExt(bits, irb.getInt64Ty()));
}

} // namespace

/**
 * @}
 **/
