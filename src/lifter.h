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

#include "config.h"
#include "facet.h"
#include "regfile.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>


namespace rellume {

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
    }

public:
    LLStateBase(LLStateBase&& rhs);
    LLStateBase& operator=(LLStateBase&& rhs);

    LLStateBase(const LLStateBase&) = delete;
    LLStateBase& operator=(const LLStateBase&) = delete;

    LLConfig& cfg;

    /// Current register file
    RegFile& regfile;

    llvm::IRBuilder<> irb;


    llvm::Value* GetReg(LLReg reg, Facet facet) {
        return regfile.GetReg(reg, facet);
    }
    void SetReg(LLReg reg, Facet facet, llvm::Value* value) {
        regfile.SetReg(reg, facet, value, true); // clear all other facets
    }
    void SetRegFacet(LLReg reg, Facet facet, llvm::Value* value) {
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
    llvm::Value* OpLoad(const LLInstrOp& op, Facet dataType, Alignment alignment = ALIGN_NONE);
    void OpStoreGp(const LLInstrOp& op, llvm::Value* value, Alignment alignment = ALIGN_NONE);
    void OpStoreVec(const LLInstrOp& op, llvm::Value* value, bool avx = false, Alignment alignment = ALIGN_IMP);
    void StackPush(llvm::Value* value);
    llvm::Value* StackPop(const LLReg sp_src_reg = LLReg(LL_RT_GP64, LL_RI_SP));

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
    void LiftSseMovq(const LLInstr&, Facet type);
    void LiftSseBinOp(const LLInstr&, llvm::Instruction::BinaryOps op,
                      Facet type);
    void LiftSseMovScalar(const LLInstr&, Facet);
    void LiftSseMovdq(const LLInstr&, Facet, Alignment);
    void LiftSseMovlp(const LLInstr&);
    void LiftSseMovhps(const LLInstr&);
    void LiftSseMovhpd(const LLInstr&);
    void LiftSseUnpck(const LLInstr&, Facet type);
    void LiftSseShufps(const LLInstr&);
    void LiftSseInsertps(const LLInstr&);
};

} // namespace

#endif
