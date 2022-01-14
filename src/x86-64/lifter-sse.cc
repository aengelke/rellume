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

#include "facet.h"
#include "instr.h"

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Value.h>

#include <algorithm>

/**
 * \defgroup LLInstructionSSE SSE Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

namespace rellume::x86_64 {

void Lifter::LiftFence(const Instr& inst) {
    // TODO: distinguish also lfence and sfence.
    irb.CreateFence(llvm::AtomicOrdering::SequentiallyConsistent);
}

void Lifter::LiftPrefetch(const Instr& inst, unsigned rw, unsigned locality) {
    llvm::Module* module = irb.GetInsertBlock()->getModule();
    llvm::SmallVector<llvm::Type*, 1> tys;
#if LL_LLVM_MAJOR >= 10
    tys.push_back(irb.getInt8PtrTy());
#endif
    auto id = llvm::Intrinsic::prefetch;
    llvm::Function* intrinsic = llvm::Intrinsic::getDeclaration(module, id, tys);

    llvm::Value* addr = OpAddr(inst.op(0), irb.getInt8Ty());
    // Prefetch addr for read/write with given locality into the data cache.
    irb.CreateCall(intrinsic, {addr, irb.getInt32(rw), irb.getInt32(locality),
                               irb.getInt32(1)});
}

void Lifter::LiftFxsave(const Instr& inst) {
    llvm::Type* i8 = irb.getInt8Ty();
    llvm::Value* buf = OpAddr(inst.op(0), i8);
    llvm::Module* mod = irb.GetInsertBlock()->getModule();
    irb.CreateAlignmentAssumption(mod->getDataLayout(), buf, 16);

    // Zero FPU status
    // TODO: FCW=0x37f, MXCSR=0x1f80, MXCSR_MASK=0xffff
#if LL_LLVM_MAJOR < 10
    unsigned align = 16;
#else
    llvm::Align align(16);
#endif
    irb.CreateMemSet(buf, irb.getInt8(0), 0xa0, align);
    for (unsigned i = 0; i < 16; i++) {
        llvm::Value* ptr = irb.CreateConstGEP1_32(i8, buf, 0xa0 + 0x10 * i);
        ptr = irb.CreatePointerCast(ptr, irb.getIntNTy(128)->getPointerTo());
        irb.CreateStore(GetReg(ArchReg::VEC(i), Facet::I128), ptr);
    }
}

void Lifter::LiftFxrstor(const Instr& inst) {
    llvm::Type* i8 = irb.getInt8Ty();
    llvm::Type* i128 = irb.getIntNTy(128);
    llvm::Value* buf = OpAddr(inst.op(0), i8);
    llvm::Module* mod = irb.GetInsertBlock()->getModule();
    irb.CreateAlignmentAssumption(mod->getDataLayout(), buf, 16);

    for (unsigned i = 0; i < 16; i++) {
        llvm::Value* ptr = irb.CreateConstGEP1_32(i8, buf, 0xa0 + 0x10 * i);
        ptr = irb.CreatePointerCast(ptr, i128->getPointerTo());
        SetReg(ArchReg::VEC(i), Facet::I128, irb.CreateLoad(i128, ptr));
    }
}

void Lifter::LiftFstcw(const Instr& inst) {
    OpStoreGp(inst.op(0), irb.getInt16(0x37f));
}

void Lifter::LiftFstsw(const Instr& inst) {
    OpStoreGp(inst.op(0), irb.getInt16(0));
}

void Lifter::LiftStmxcsr(const Instr& inst) {
    OpStoreGp(inst.op(0), irb.getInt32(0x1f80));
}

void Lifter::LiftSseMovq(const Instr& inst, Facet type) {
    llvm::Value* op1 = OpLoad(inst.op(1), type);
    if (inst.op(0).is_reg() && inst.op(0).reg().rt == FD_RT_VEC) {
        llvm::Type* el_ty = op1->getType();
        unsigned el_sz = el_ty->getPrimitiveSizeInBits();
        llvm::Type* vector_ty = llvm::VectorType::get(el_ty, 128 / el_sz, false);
        llvm::Value* zero = llvm::Constant::getNullValue(vector_ty);
        llvm::Value* zext = irb.CreateInsertElement(zero, op1, 0ul);
        OpStoreVec(inst.op(0), zext);
    } else {
        OpStoreGp(inst.op(0), op1);
    }
}

void Lifter::LiftSseMovScalar(const Instr& inst, Facet facet) {
    llvm::Value* src = OpLoad(inst.op(1), facet);
    if (inst.op(1).is_mem()) {
        llvm::Type* el_ty = src->getType();
        unsigned el_sz = el_ty->getPrimitiveSizeInBits();
        llvm::Type* vector_ty = llvm::VectorType::get(el_ty, 128 / el_sz, false);
        llvm::Value* zero = llvm::Constant::getNullValue(vector_ty);
        llvm::Value* zext = irb.CreateInsertElement(zero, src, 0ul);
        OpStoreVec(inst.op(0), zext);
    } else {
        OpStoreVec(inst.op(0), src);
    }
}

void Lifter::LiftSseMovdq(const Instr& inst, Facet facet, Alignment alignment) {
    OpStoreVec(inst.op(0), OpLoad(inst.op(1), facet, alignment), alignment);
}

void Lifter::LiftSseMovntStore(const Instr& inst, Facet facet) {
    llvm::Value* value = OpLoad(inst.op(1), facet, ALIGN_MAX);
    llvm::Value* addr = OpAddr(inst.op(0), value->getType());
    llvm::StoreInst* store = irb.CreateStore(value, addr);
    unsigned align = value->getType()->getPrimitiveSizeInBits() / 8;
#if LL_LLVM_MAJOR < 10
    store->setAlignment(align);
#else
    store->setAlignment(llvm::Align(align));
#endif

    llvm::Metadata* const_1 = llvm::ConstantAsMetadata::get(irb.getInt32(1));
    llvm::MDNode* node = llvm::MDNode::get(store->getContext(), const_1);
    store->setMetadata(GetModule()->getMDKindID("nontemporal"), node);
}

void Lifter::LiftSseMovlp(const Instr& inst) {
    if (inst.op(0).is_reg() && inst.op(1).is_reg()) {
        // move high 64-bit from src to low 64-bit from dst
        assert(inst.type() == FDI_SSE_MOVHLPS);
        llvm::Value* op2 = OpLoad(inst.op(1), Facet::V4F32);
        llvm::Value* zero = llvm::Constant::getNullValue(op2->getType());
        OpStoreVec(inst.op(0), CreateShuffleVector(op2, zero, {2, 3}));
    } else {
        // move (low) 64-bit from src to (low) 64-bit from dst
        auto facet = inst.type() == FDI_SSE_MOVLPS ? Facet::V2F32 : Facet::F64;
        OpStoreVec(inst.op(0), OpLoad(inst.op(1), facet));
    }
}

void Lifter::LiftSseMovhps(const Instr& inst) {
    if (inst.op(0).is_mem()) {
        // move high 64-bit from src to (low) 64-bit from dst (in memory)
        llvm::Value* op2 = OpLoad(inst.op(1), Facet::V4F32);
        llvm::Value* zero = llvm::Constant::getNullValue(op2->getType());
        llvm::Value* res = CreateShuffleVector(op2, zero, {2, 3});
        OpStoreVec(inst.op(0), res);
    } else {
        // move low 64-bit from src to high 64-bit from dst
        // for reg-reg, the official mnemonic is MOVLHPS.
        llvm::Value* op1 = OpLoad(inst.op(0), Facet::V4F32);
        llvm::Value* op2 = OpLoad(inst.op(1), Facet::V2F32);

        // first, enlarge op2 to full vector width
        llvm::Value* zero_half = llvm::Constant::getNullValue(op2->getType());
        op2 = CreateShuffleVector(op2, zero_half, {0, 1, 2, 3});
        // and then shuffle the two operands together
        llvm::Value* res = CreateShuffleVector(op1, op2, {0, 1, 4, 5});
        OpStoreVec(inst.op(0), res);
    }
}

void Lifter::LiftSseMovhpd(const Instr& inst) {
    if (inst.op(0).is_mem()) {
        // move high 64-bit from src to (low) 64-bit from dst (in memory)
        llvm::Value* op2 = OpLoad(inst.op(1), Facet::V2F64);
        OpStoreVec(inst.op(0), irb.CreateExtractElement(op2, 1u));
    } else {
        // move low 64-bit from src to high 64-bit from dst
        llvm::Value* op1 = OpLoad(inst.op(0), Facet::V2F64);
        llvm::Value* op2 = OpLoad(inst.op(1), Facet::F64);
        OpStoreVec(inst.op(0), irb.CreateInsertElement(op1, op2, 1u));
    }
}

void Lifter::LiftSseBinOp(const Instr& inst, llvm::Instruction::BinaryOps op,
                           Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_IMP);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_IMP);
    OpStoreVec(inst.op(0), irb.CreateBinOp(op, op1, op2), /*avx=*/false,
               ALIGN_IMP);
}

void Lifter::LiftSseHorzOp(const Instr& inst, llvm::Instruction::BinaryOps op,
                           Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    auto cnt = VectorElementCount(op1->getType());

    llvm::SmallVector<int, 16> mask1, mask2;
    for (unsigned i = 0; i < cnt; i++) {
        mask1.push_back(2 * i);
        mask2.push_back(2 * i + 1);
    }

    llvm::Value* shuf1 = irb.CreateShuffleVector(op1, op2, mask1);
    llvm::Value* shuf2 = irb.CreateShuffleVector(op1, op2, mask2);
    OpStoreVec(inst.op(0), irb.CreateBinOp(op, shuf1, shuf2));
}

void Lifter::LiftSseAndn(const Instr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    OpStoreVec(inst.op(0), irb.CreateAnd(irb.CreateNot(op1), op2));
}

void Lifter::LiftSseComis(const Instr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type);
    SetFlag(Facet::ZF, irb.CreateFCmpUEQ(op1, op2));
    SetFlag(Facet::CF, irb.CreateFCmpULT(op1, op2));
    SetFlag(Facet::PF, irb.CreateFCmpUNO(op1, op2));
    SetFlag(Facet::AF, irb.getFalse());
    SetFlag(Facet::OF, irb.getFalse());
    SetFlag(Facet::SF, irb.getFalse());
}

void Lifter::LiftSseCmp(const Instr& inst, Facet op_type) {
    llvm::FCmpInst::Predicate pred;
    switch (inst.op(2).imm()) {
    case 0: pred = llvm::FCmpInst::FCMP_OEQ; break; // EQ_OQ
    case 1: pred = llvm::FCmpInst::FCMP_OLT; break; // LT_OS
    case 2: pred = llvm::FCmpInst::FCMP_OLE; break; // LE_OS
    case 3: pred = llvm::FCmpInst::FCMP_UNO; break; // UNORD_Q
    case 4: pred = llvm::FCmpInst::FCMP_UNE; break; // NEQ_UQ
    case 5: pred = llvm::FCmpInst::FCMP_UGE; break; // NLT_US
    case 6: pred = llvm::FCmpInst::FCMP_UGT; break; // NLE_US
    case 7: pred = llvm::FCmpInst::FCMP_ORD; break; // ORD_Q
    default: assert(false); return;
    }
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    llvm::Value* eq = irb.CreateFCmp(pred, op1, op2);
    llvm::Type* cmp_ty = op1->getType();
    llvm::Type* res_ty;
    if (cmp_ty->isVectorTy()) {
        res_ty = llvm::VectorType::getInteger(llvm::cast<llvm::VectorType>(cmp_ty));
    } else {
        res_ty = irb.getIntNTy(cmp_ty->getScalarSizeInBits());
    }
    OpStoreVec(inst.op(0), irb.CreateSExt(eq, res_ty));
}

void Lifter::LiftSseMinmax(const Instr& inst, llvm::CmpInst::Predicate pred,
                            Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    llvm::Value* cmp = irb.CreateFCmp(pred, op1, op2);
    OpStoreVec(inst.op(0), irb.CreateSelect(cmp, op1, op2));
}

void Lifter::LiftSseSqrt(const Instr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(1), op_type);
    OpStoreVec(inst.op(0), CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, op1));
}

void Lifter::LiftSseCvt(const Instr& inst, Facet src_type, Facet dst_type) {
    if (dst_type == Facet::I)
        dst_type = dst_type.Resolve(inst.op(0).bits());
    llvm::Type* dst_ty = dst_type.Type(irb.getContext());

    llvm::Value* src = OpLoad(inst.op(1), src_type);
    auto cast_op = llvm::CastInst::getCastOpcode(src, true, dst_ty, true);
    llvm::Value* dst = irb.CreateCast(cast_op, src, dst_ty);

    if (dst_ty->isIntegerTy()) {
        OpStoreGp(inst.op(0), dst);
    } else if (dst_ty->isFloatingPointTy()) {
        OpStoreVec(inst.op(0), dst);
    } else if (dst_ty->isVectorTy()) {
        if (dst_ty->getPrimitiveSizeInBits() == 64) {
            // Zero upper half
            llvm::Value* zero_half = llvm::Constant::getNullValue(dst_ty);
            dst = CreateShuffleVector(dst, zero_half, {0, 1, 2, 3});
        } else {
            assert(dst_ty->getPrimitiveSizeInBits() == 128);
        }
        OpStoreVec(inst.op(0), dst);
    } else {
        assert(false && "invalid cvt target type");
    }
}

void Lifter::LiftSseUnpck(const Instr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type);
    // We always fetch 128 bits, as per SDM.
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    llvm::Value* res = nullptr;
    if (inst.type() == FDI_SSE_PUNPCKLBW)
        res = CreateShuffleVector(op1, op2, {0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23});
    else if (inst.type() == FDI_SSE_PUNPCKHBW)
        res = CreateShuffleVector(op1, op2, {8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31});
    else if (inst.type() == FDI_SSE_PUNPCKLWD)
        res = CreateShuffleVector(op1, op2, {0, 8, 1, 9, 2, 10, 3, 11});
    else if (inst.type() == FDI_SSE_PUNPCKHWD)
        res = CreateShuffleVector(op1, op2, {4, 12, 5, 13, 6, 14, 7, 15});
    else if (inst.type() == FDI_SSE_UNPCKLPS || inst.type() == FDI_SSE_PUNPCKLDQ)
        res = CreateShuffleVector(op1, op2, {0, 4, 1, 5});
    else if (inst.type() == FDI_SSE_UNPCKLPD || inst.type() == FDI_SSE_PUNPCKLQDQ)
        res = CreateShuffleVector(op1, op2, {0, 2});
    else if (inst.type() == FDI_SSE_UNPCKHPS || inst.type() == FDI_SSE_PUNPCKHDQ)
        res = CreateShuffleVector(op1, op2, {2, 6, 3, 7});
    else if (inst.type() == FDI_SSE_UNPCKHPD || inst.type() == FDI_SSE_PUNPCKHQDQ)
        res = CreateShuffleVector(op1, op2, {1, 3});
    else
        assert(0);
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSseShufpd(const Instr& inst) {
    int mask[2] = { inst.op(2).imm()&1 ? 1 : 0, inst.op(2).imm()&2 ? 3 : 2 };
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::VF64);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::VF64, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(op1, op2, mask);
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSseShufps(const Instr& inst) {
    int mask[4];
    for (int i = 0; i < 4; i++)
        mask[i] = (i < 2 ? 0 : 4) + ((inst.op(2).imm() >> 2*i) & 3);
    llvm::Value* op1 = OpLoad(inst.op(0), Facet::VF32);
    llvm::Value* op2 = OpLoad(inst.op(1), Facet::VF32, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(op1, op2, mask);
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSsePshufd(const Instr& inst) {
    int mask[4];
    for (int i = 0; i < 4; i++)
        mask[i] = ((inst.op(2).imm() >> 2*i) & 3);
    llvm::Value* src = OpLoad(inst.op(1), Facet::VI32, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(src, src, mask);
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSsePshufw(const Instr& inst, unsigned off) {
    int mask[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    for (unsigned i = off; i < off+4; i++)
        mask[i] = off + ((inst.op(2).imm() >> 2*i) & 3);
    llvm::Value* src = OpLoad(inst.op(1), Facet::VI16, ALIGN_MAX);
    llvm::Value* res = irb.CreateShuffleVector(src, src, mask);
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSseInsertps(const Instr& inst) {
    int count_s = (inst.op(2).imm() >> 6) & 3;
    int count_d = (inst.op(2).imm() >> 4) & 3;
    int zmask = inst.op(2).imm() & 0xf;

    llvm::Value* dst = OpLoad(inst.op(0), Facet::V4F32);
    llvm::Value* src;
    // If src is a reg, extract element, otherwise load scalar from memory.
    if (inst.op(1).is_reg()) {
        src = OpLoad(inst.op(1), Facet::V4F32);
        src = irb.CreateExtractElement(src, count_s);
    } else {
        src = OpLoad(inst.op(1), Facet::F32);
    }

    dst = irb.CreateInsertElement(dst, src, count_d);

    if (zmask) {
        int mask[4];
        for (int i = 0; i < 4; i++)
            mask[i] = zmask & (1 << i) ? 4 : i;
        llvm::Value* zero = llvm::Constant::getNullValue(dst->getType());
        dst = irb.CreateShuffleVector(dst, zero, mask);
    }

    OpStoreVec(inst.op(0), dst);
}

void Lifter::LiftSsePinsr(const Instr& inst, Facet vec_op, Facet src_op,
                          unsigned mask) {
    llvm::Value* dst = OpLoad(inst.op(0), vec_op);
    llvm::Value* src = OpLoad(inst.op(1), src_op);
    unsigned count = inst.op(2).imm() & mask;
    OpStoreVec(inst.op(0), irb.CreateInsertElement(dst, src, count));
}

void Lifter::LiftSsePextr(const Instr& inst, Facet vec_op, unsigned mask) {
    llvm::Value* src = OpLoad(inst.op(1), vec_op);
    unsigned count = inst.op(2).imm() & mask;
    llvm::Value* ext = irb.CreateExtractElement(src, count);
    if (inst.op(0).is_reg()) {
        assert(inst.op(0).reg().rt == FD_RT_GPL);
        if (!ext->getType()->isIntegerTy()) {
            unsigned bitsize = ext->getType()->getPrimitiveSizeInBits();
            ext = irb.CreateBitCast(ext, irb.getIntNTy(bitsize));
        }
        ext = irb.CreateZExt(ext, irb.getInt64Ty());
        StoreGp(ArchReg::GP(inst.op(0).reg().ri), ext);
    } else {
        OpStoreGp(inst.op(0), ext);
    }
}

void Lifter::LiftSseMovdup(const Instr& inst, Facet op_type, unsigned off) {
    llvm::Value* src = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    auto el_ty = llvm::cast<llvm::VectorType>(src->getType())->getElementType();

    llvm::SmallVector<int, 4> mask;
    for (unsigned i = 0; i < 128/el_ty->getPrimitiveSizeInBits(); i++)
        mask.push_back((i & 0xfe) + off);
    OpStoreVec(inst.op(0), irb.CreateShuffleVector(src, src, mask));
}

void Lifter::LiftSsePshiftElement(const Instr& inst,
                                  llvm::Instruction::BinaryOps op,
                                  Facet op_type) {
    llvm::Value* src = OpLoad(inst.op(0), op_type);
    llvm::Value* shift = OpLoad(inst.op(1), Facet::I64);

    llvm::VectorType* vec_ty = llvm::cast<llvm::VectorType>(src->getType());
    llvm::Type* elem_ty = vec_ty->getElementType();
    unsigned elem_size = elem_ty->getIntegerBitWidth();
    unsigned elem_cnt = VectorElementCount(vec_ty);

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

    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSsePshiftBytes(const Instr& inst) {
    uint32_t shift = std::min(static_cast<uint32_t>(inst.op(1).imm()), 16u);
    bool right = inst.type() == FDI_SSE_PSRLDQ;
    int mask[16];
    for (int i = 0; i < 16; i++)
        mask[i] = i + (right ? shift : (16 - shift));
    llvm::Value* src = OpLoad(inst.op(0), Facet::V16I8);
    llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
    if (right)
        OpStoreVec(inst.op(0), irb.CreateShuffleVector(src, zero, mask));
    else
        OpStoreVec(inst.op(0), irb.CreateShuffleVector(zero, src, mask));
}

void Lifter::LiftSsePavg(const Instr& inst, Facet op_type) {
    llvm::Value* src1 = OpLoad(inst.op(0), op_type);
    llvm::Value* src2 = OpLoad(inst.op(1), op_type);

    llvm::VectorType* vec_ty = llvm::cast<llvm::VectorType>(src1->getType());
    llvm::Type* elem_ty = vec_ty->getScalarType();
    unsigned elem_size = elem_ty->getIntegerBitWidth();
    unsigned elem_cnt = VectorElementCount(vec_ty);

    llvm::Type* ext_ty = llvm::VectorType::getExtendedElementVectorType(vec_ty);
    llvm::Value* ones = irb.CreateVectorSplat(elem_cnt, irb.getIntN(elem_size*2, 1));
    llvm::Value* ext1 = irb.CreateZExt(src1, ext_ty);
    llvm::Value* ext2 = irb.CreateZExt(src2, ext_ty);
    llvm::Value* sum = irb.CreateAdd(irb.CreateAdd(ext1, ext2), ones);
    llvm::Value* res = irb.CreateTrunc(irb.CreateLShr(sum, ones), src1->getType());
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSsePmulhw(const Instr& inst, llvm::Instruction::CastOps cast) {
    llvm::Value* src1 = OpLoad(inst.op(0), Facet::VI16);
    llvm::Value* src2 = OpLoad(inst.op(1), Facet::VI16);

    llvm::Type* ext_ty = llvm::VectorType::get(irb.getInt32Ty(), 8, false);

    llvm::Value* ext1 = irb.CreateCast(cast, src1, ext_ty);
    llvm::Value* ext2 = irb.CreateCast(cast, src2, ext_ty);
    llvm::Value* mul = irb.CreateMul(ext1, ext2);
    llvm::Value* shift = irb.CreateVectorSplat(8, irb.getInt32(16));
    llvm::Value* res = irb.CreateTrunc(irb.CreateLShr(mul, shift), src1->getType());
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSsePmuldq(const Instr& inst, llvm::Instruction::CastOps ext) {
    llvm::Value* src1 = OpLoad(inst.op(0), Facet::VI32);
    llvm::Value* src2 = OpLoad(inst.op(1), Facet::VI32);

    llvm::Type* ext_ty = llvm::VectorType::get(irb.getInt64Ty(), 2, false);
    src1 = irb.CreateCast(ext, CreateShuffleVector(src1, src1, {0, 2}), ext_ty);
    src2 = irb.CreateCast(ext, CreateShuffleVector(src2, src2, {0, 2}), ext_ty);

    OpStoreVec(inst.op(0), irb.CreateMul(src1, src2));
}

void Lifter::LiftSsePmaddwd(const Instr& inst) {
    llvm::Value* src1 = OpLoad(inst.op(0), Facet::VI16);
    llvm::Value* src2 = OpLoad(inst.op(1), Facet::VI16);

    llvm::Type* ext_ty = llvm::VectorType::get(irb.getInt32Ty(), 8, false);

    llvm::Value* ext1 = irb.CreateSExt(src1, ext_ty);
    llvm::Value* ext2 = irb.CreateSExt(src2, ext_ty);
    llvm::Value* mul = irb.CreateMul(ext1, ext2);
    llvm::Value* add1 = CreateShuffleVector(mul, mul, {0, 2, 4, 6});
    llvm::Value* add2 = CreateShuffleVector(mul, mul, {1, 3, 5, 7});
    OpStoreVec(inst.op(0), irb.CreateAdd(add1, add2));
}

static llvm::Value* SaturateTrunc(llvm::IRBuilder<>& irb, llvm::Value* val,
                                  bool sign) {
    llvm::VectorType* src_ty = llvm::cast<llvm::VectorType>(val->getType());
    unsigned src_size = src_ty->getScalarSizeInBits();
    llvm::Type* dst_ty = llvm::VectorType::getTruncatedElementVectorType(src_ty);

    llvm::APInt min, max;
    if (sign) {
        min = llvm::APInt::getSignedMinValue(src_size/2).sext(src_size);
        max = llvm::APInt::getSignedMaxValue(src_size/2).sext(src_size);
    } else {
        min = llvm::APInt::getMinValue(src_size/2).zext(src_size);
        max = llvm::APInt::getMaxValue(src_size/2).zext(src_size);
    }

    llvm::Constant* minv = llvm::Constant::getIntegerValue(val->getType(), min);
    llvm::Constant* maxv = llvm::Constant::getIntegerValue(val->getType(), max);
    val = irb.CreateSelect(irb.CreateICmpSLT(val, minv), minv, val);
    val = irb.CreateSelect(irb.CreateICmpSGT(val, maxv), maxv, val);

    return irb.CreateTrunc(val, dst_ty);
}

void Lifter::LiftSsePaddsubSaturate(const Instr& inst,
                                    llvm::Instruction::BinaryOps calc_op,
                                    bool sign, Facet op_ty) {
    llvm::Value* src1 = OpLoad(inst.op(0), op_ty);
    llvm::Value* src2 = OpLoad(inst.op(1), op_ty);
    llvm::Instruction::CastOps cast = sign ? llvm::Instruction::SExt
                                           : llvm::Instruction::ZExt;

    llvm::VectorType* src_ty = llvm::cast<llvm::VectorType>(src1->getType());
    llvm::Type* ext_ty = llvm::VectorType::getExtendedElementVectorType(src_ty);
    llvm::Value* ext1 = irb.CreateCast(cast, src1, ext_ty);
    llvm::Value* ext2 = irb.CreateCast(cast, src2, ext_ty);
    llvm::Value* res = irb.CreateBinOp(calc_op, ext1, ext2);

    OpStoreVec(inst.op(0), SaturateTrunc(irb, res, sign));
}

void Lifter::LiftSsePsadbw(const Instr& inst) {
    llvm::Value* src1 = OpLoad(inst.op(0), Facet::VI8);
    llvm::Value* src2 = OpLoad(inst.op(1), Facet::VI8);
    llvm::Type* ext_ty = llvm::VectorType::get(irb.getInt16Ty(), 8, false);
    llvm::Value* zero = llvm::Constant::getNullValue(src1->getType());

    llvm::Value* diff = irb.CreateSub(src1, src2);
    llvm::Value* negdiff = irb.CreateNeg(diff);
    diff = irb.CreateSelect(irb.CreateICmpUGT(src2, src1), negdiff, diff);

    llvm::Value* low = CreateShuffleVector(diff, zero,
                                               {0, 1, 2, 3, 4, 5, 6, 7});
    llvm::Value* lowr = irb.CreateAddReduce(irb.CreateZExt(low, ext_ty));
    llvm::Value* high = CreateShuffleVector(diff, zero,
                                                {8, 9, 10, 11, 12, 13, 14, 15});
    llvm::Value* highr = irb.CreateAddReduce(irb.CreateZExt(high, ext_ty));

    llvm::Value* zero_ext = llvm::Constant::getNullValue(ext_ty);
    llvm::Value* res = irb.CreateInsertElement(zero_ext, lowr, uint64_t{0});
    res = irb.CreateInsertElement(res, highr, uint64_t{4});
    OpStoreVec(inst.op(0), res);
}

void Lifter::LiftSsePack(const Instr& inst, Facet src_type, bool sign) {
    llvm::Value* op1 = OpLoad(inst.op(0), src_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), src_type, ALIGN_MAX);
    auto cnt = VectorElementCount(op1->getType());

    op1 = SaturateTrunc(irb, op1, sign);
    op2 = SaturateTrunc(irb, op2, sign);

    llvm::SmallVector<int, 16> mask;
    for (unsigned i = 0; i < cnt; i++)
        mask.push_back(i);
    for (unsigned i = 0; i < cnt; i++)
        mask.push_back(i + cnt);

    OpStoreVec(inst.op(0), irb.CreateShuffleVector(op1, op2, mask));
}

void Lifter::LiftSsePcmp(const Instr& inst, llvm::CmpInst::Predicate pred,
                         Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    llvm::Value* eq = irb.CreateICmp(pred, op1, op2);
    OpStoreVec(inst.op(0), irb.CreateSExt(eq, op1->getType()));
}

void Lifter::LiftSsePminmax(const Instr& inst, llvm::CmpInst::Predicate pred,
                            Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.op(0), op_type, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    llvm::Value* cmp = irb.CreateICmp(pred, op1, op2);
    OpStoreVec(inst.op(0), irb.CreateSelect(cmp, op1, op2));
}

void Lifter::LiftSsePabs(const Instr& inst, Facet type) {
    llvm::Value* src = OpLoad(inst.op(1), type, ALIGN_MAX);
    llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
    llvm::Value* neg = irb.CreateSub(zero, src);
    llvm::Value* cmp = irb.CreateICmpSGE(src, zero);
    OpStoreVec(inst.op(0), irb.CreateSelect(cmp, src, neg));
}

void Lifter::LiftSseMovmsk(const Instr& inst, Facet op_type) {
    llvm::Value* src = OpLoad(inst.op(1), op_type, ALIGN_MAX);
    llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
    llvm::Value* bitvec = irb.CreateICmpSLT(src, zero);
    unsigned bit_count = VectorElementCount(src->getType());
    llvm::Value* bits = irb.CreateBitCast(bitvec, irb.getIntNTy(bit_count));
    OpStoreGp(inst.op(0), irb.CreateZExt(bits, irb.getInt64Ty()));
}

void Lifter::LiftSsePmovx(const Instr& inst, llvm::Instruction::CastOps ext,
                          Facet from, Facet to) {
    llvm::Value* src = OpLoad(inst.op(1), from);
    llvm::Type* dst_ty = to.Type(irb.getContext());
    OpStoreVec(inst.op(0), irb.CreateCast(ext, src, dst_ty));
}

} // namespace::x86_64

/**
 * @}
 **/
