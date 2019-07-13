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

#ifndef LL_STATE_H
#define LL_STATE_H

#include "llcommon-internal.h"
#include "llregfile-internal.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>


#ifdef __cplusplus
extern "C" {
#endif

enum Alignment {
    /// Implicit alignment -- MAX for SSE operand, 1 otherwise
    ALIGN_IMP = -1,
    /// Maximum alignment, alignment is set to the size of the value
    ALIGN_MAX = 0,
    /// No alignment (1 byte)
    ALIGN_NONE = 1,
    ALIGN_MAXIMUM = ALIGN_MAX,
    ALIGN_1 = ALIGN_NONE,
};

enum class Condition {
    O = 0, NO = 1, C = 2, NC = 3, Z = 4, NZ = 5, BE = 6, A = 7,
    S = 8, NS = 9, P = 10, NP = 11, L = 12, GE = 13, LE = 14, G = 15,
};

/**
 * \brief The LLVM state of the back-end.
 **/
class LLStateBase {
protected:
    LLStateBase(LLConfig& cfg, RegFile& rf, llvm::BasicBlock* bb) : cfg(cfg),
            regfile(rf), irb(bb) {
        // Set fast-math flags. Newer LLVM supports FastMathFlags::getFast().
        if (cfg.enableFastMath) {
            llvm::FastMathFlags fmf;
            fmf.setFast();
            irb.setFastMathFlags(fmf);
        }

        builder = llvm::wrap(&irb);
    }

public:
    LLStateBase(LLStateBase&& rhs);
    LLStateBase& operator=(LLStateBase&& rhs);

    LLStateBase(const LLStateBase&) = delete;
    LLStateBase& operator=(const LLStateBase&) = delete;

    LLConfig& cfg;

    /// DEPRECATED LLVM builder, use irb
    LLVMBuilderRef builder;

    /// Current register file
    RegFile& regfile;

    llvm::IRBuilder<> irb;


    llvm::Value* GetReg(LLReg reg, Facet::Value facet) {
        return regfile.GetReg(reg, facet);
    }
    void SetReg(LLReg reg, Facet::Value facet, llvm::Value* value) {
        regfile.SetReg(reg, facet, value, true); // clear all other facets
    }
    void SetRegFacet(LLReg reg, Facet::Value facet, llvm::Value* value) {
        regfile.SetReg(reg, facet, value, false);
    }
    llvm::Value* GetFlag(int flag) {
        return regfile.GetFlag(flag);
    }
    void SetFlag(int flag, llvm::Value* value) {
        regfile.SetFlag(flag, value);
    }

    // Operand handling implemented in lloperand.cc
private:
    llvm::Value* OpAddrConst(uint64_t addr);
public:
    llvm::Value* OpAddr(const LLInstrOp& op, llvm::Type* element_type);
    llvm::Value* OpLoad(const LLInstrOp& op, Facet::Value dataType, Alignment alignment = ALIGN_NONE);
    void OpStoreGp(const LLInstrOp& op, llvm::Value* value, Alignment alignment = ALIGN_NONE);
    void OpStoreVec(const LLInstrOp& op, llvm::Value* value, bool avx = false, Alignment alignment = ALIGN_IMP);

    // llflags.cc
    void FlagCalcZ(llvm::Value* value) {
        auto zero = llvm::Constant::getNullValue(value->getType());
        SetFlag(RFLAG_ZF, irb.CreateICmpEQ(value, zero));
    }
    void FlagCalcS(llvm::Value* value) {
        auto zero = llvm::Constant::getNullValue(value->getType());
        SetFlag(RFLAG_SF, irb.CreateICmpSLT(value, zero));
    }
    void FlagCalcP(llvm::Value* value);
    void FlagCalcA(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    void FlagCalcCAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs) {
        SetFlag(RFLAG_CF, irb.CreateICmpULT(res, lhs));
    }
    void FlagCalcCSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs) {
        SetFlag(RFLAG_CF, irb.CreateICmpULT(lhs, rhs));
    }
    void FlagCalcOAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    void FlagCalcOSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);

    llvm::Value* FlagCond(Condition cond);
    llvm::Value* FlagAsReg(unsigned size);

    // llinstruction-stack.cc
    void StackPush(llvm::Value* value);
    llvm::Value* StackPop(const LLReg sp_src_reg = LLReg(LL_RT_GP64, LL_RI_SP));
};

class LLState : public LLStateBase {
public:
    LLState(LLConfig& cfg, RegFile& rf, llvm::BasicBlock* bb) : LLStateBase(cfg, rf, bb) {}

    // llinstruction-gp.cc
    void LiftMovgp(const LLInstr&, llvm::Instruction::CastOps cast);
    void LiftAdd(const LLInstr&);
    void LiftSub(const LLInstr&);
    void LiftCmp(const LLInstr&);
    void LiftAndOrXor(const LLInstr& inst, llvm::Instruction::BinaryOps op,
                      bool writeback = true);
    void LiftNot(const LLInstr&);
    void LiftNeg(const LLInstr&);
    void LiftIncDec(const LLInstr&);
    void LiftShift(const LLInstr&, llvm::Instruction::BinaryOps);
    void LiftMul(const LLInstr&);
    void LiftLea(const LLInstr&);
    void LiftCmovcc(const LLInstr& inst, Condition cond);
    void LiftSetcc(const LLInstr& inst, Condition cond);
    void LiftCdqe(const LLInstr& inst);

    void LiftPush(const LLInstr& inst) {
        StackPush(OpLoad(inst.ops[0], Facet::I));
    }
    void LiftPushf(const LLInstr& inst) {
        StackPush(FlagAsReg(inst.operand_size * 8));
    }
    void LiftPop(const LLInstr& inst) {
        OpStoreGp(inst.ops[0], StackPop());
    }
    void LiftLeave(const LLInstr& inst) {
        llvm::Value* val = StackPop(LLReg(LL_RT_GP64, LL_RI_BP));
        OpStoreGp(LLInstrOp::Reg(LLReg(LL_RT_GP64, LL_RI_BP)), val);
    }

    void LiftJmp(const LLInstr& inst) {
        SetReg(LLReg(LL_RT_IP, 0), Facet::I64, OpLoad(inst.ops[0], Facet::I64));
    }
    void LiftJcc(const LLInstr& inst, Condition cond) {
        SetReg(LLReg(LL_RT_IP, 0), Facet::I64, irb.CreateSelect(FlagCond(cond),
            OpLoad(inst.ops[0], Facet::I64),
            GetReg(LLReg(LL_RT_IP, 0), Facet::I64)
        ));
    }
    void LiftCall(const LLInstr& inst) {
        llvm::Value* new_rip = OpLoad(inst.ops[0], Facet::I);
        StackPush(GetReg(LLReg(LL_RT_IP, 0), Facet::I64));
        SetReg(LLReg(LL_RT_IP, 0), Facet::I64, new_rip);
    }
    void LiftRet(const LLInstr& inst) {
        OpStoreGp(LLInstrOp::Reg(LLReg(LL_RT_IP, 0)), StackPop());
    }

    // llinstruction-sse.cc
    void LiftSseMovq(const LLInstr&, Facet::Value type);
    void LiftSseBinOp(const LLInstr&, llvm::Instruction::BinaryOps op,
                      Facet::Value type);
    void LiftSseMovScalar(const LLInstr&, Facet::Value);
    void LiftSseMovdq(const LLInstr&, Facet::Value, Alignment);
    void LiftSseMovlp(const LLInstr&);
    void LiftSseMovhps(const LLInstr&);
    void LiftSseMovhpd(const LLInstr&);
    void LiftSseUnpck(const LLInstr&, Facet::Value type);
    void LiftSseShufps(const LLInstr&);
    void LiftSseInsertps(const LLInstr&);
};

// Legacy API.

#define OP_SI Facet::I
#define OP_SI32 Facet::I32
#define OP_SI64 Facet::I64
#define OP_VI8 Facet::VI8
#define OP_VI32 Facet::VI32
#define OP_VI64 Facet::VI64
#define OP_SF32 Facet::F32
#define OP_SF64 Facet::F64
#define OP_VF32 Facet::VF32
#define OP_V1F32 Facet::V1F32
#define OP_V2F32 Facet::V2F32
#define OP_V4F32 Facet::V4F32
#define OP_VF64 Facet::VF64
#define OP_V1F64 Facet::V1F64
#define OP_V2F64 Facet::V2F64
#define OperandDataType Facet::Value

enum {
    REG_DEFAULT, REG_ZERO_UPPER_AVX, REG_KEEP_UPPER,
};

#define ll_operand_load(facet,align,op,state) llvm::wrap((state)->OpLoad(*(op), facet, align))
#define ll_operand_store(facet,align,op,prh,val,state) do { \
            if ((prh) == REG_DEFAULT) \
                (state)->OpStoreGp(*(op), llvm::unwrap(val), align); \
            else \
                (state)->OpStoreVec(*(op), llvm::unwrap(val), prh == REG_ZERO_UPPER_AVX, align); \
        } while (0)


#ifdef __cplusplus
}
#endif

#endif
