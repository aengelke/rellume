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



/**
 * \defgroup LLInstructionSSE SSE Instructions
 * \ingroup LLInstruction
 *
 * @{
 **/

namespace rellume {

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

void Lifter::LiftSseUnpck(const LLInstr& inst, Facet op_type) {
    llvm::Value* op1 = OpLoad(inst.ops[0], op_type);
    // We always fetch 128 bits, as per SDM.
    llvm::Value* op2 = OpLoad(inst.ops[1], op_type, ALIGN_MAX);
    llvm::Value* res = nullptr;
    if (inst.type == LL_INS_UNPCKLPS)
        res = irb.CreateShuffleVector(op1, op2, {0, 4, 1, 5});
    else if (inst.type == LL_INS_UNPCKLPD)
        res = irb.CreateShuffleVector(op1, op2, {0, 2});
    else if (inst.type == LL_INS_UNPCKHPS)
        res = irb.CreateShuffleVector(op1, op2, {2, 6, 3, 7});
    else if (inst.type == LL_INS_UNPCKHPD)
        res = irb.CreateShuffleVector(op1, op2, {1, 3});
    else
        assert(0);
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

void Lifter::LiftSsePcmpeqb(const LLInstr& inst) {
    llvm::Value* op1 = OpLoad(inst.ops[0], Facet::VI8, ALIGN_MAX);
    llvm::Value* op2 = OpLoad(inst.ops[1], Facet::VI8, ALIGN_MAX);
    llvm::Value* eq = irb.CreateICmpEQ(op1, op2);
    OpStoreVec(inst.ops[0], irb.CreateSExt(eq, op1->getType()));
}

void Lifter::LiftSsePmovmskb(const LLInstr& inst) {
    llvm::Value* src = OpLoad(inst.ops[1], Facet::VI8, ALIGN_MAX);
    llvm::Value* zero = llvm::Constant::getNullValue(src->getType());
    llvm::Value* bitvec = irb.CreateICmpSLT(src, zero);
    llvm::Value* bits = irb.CreateBitCast(bitvec, irb.getIntNTy(inst.ops[1].size));
    OpStoreGp(inst.ops[0], irb.CreateZExt(bits, irb.getInt64Ty()));
}

} // namespace

/**
 * @}
 **/
