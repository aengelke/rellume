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

#ifndef RELLUME_LIFTER_H
#define RELLUME_LIFTER_H

#include "config.h"
#include "facet.h"
#include "regfile.h"
#include "rellume/instr.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Operator.h>
#include <vector>


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
class LifterBase {
protected:
    LifterBase(const LLConfig& cfg, RegFile& rf) : cfg(cfg), regfile(rf),
            irb(rf.GetInsertBlock()) {
        // Set fast-math flags. Newer LLVM supports FastMathFlags::getFast().
        if (cfg.enableFastMath) {
            llvm::FastMathFlags fmf;
#if LL_LLVM_MAJOR >= 6
            fmf.setFast();
#else
            fmf.setUnsafeAlgebra();
#endif
            irb.setFastMathFlags(fmf);
        }
    }

public:
    LifterBase(LifterBase&& rhs);
    LifterBase& operator=(LifterBase&& rhs);

    LifterBase(const LifterBase&) = delete;
    LifterBase& operator=(const LifterBase&) = delete;

protected:
    const LLConfig& cfg;

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
    llvm::Value* GetFlag(Facet facet) {
        return GetReg(LLReg(LL_RT_EFLAGS, 0), facet);
    }
    void SetFlag(Facet facet, llvm::Value* value) {
        SetRegFacet(LLReg(LL_RT_EFLAGS, 0), facet, value);
    }
    void SetFlagUndef(std::initializer_list<Facet> facets) {
        llvm::Value* undef = llvm::UndefValue::get(irb.getInt1Ty());
        for (const auto facet : facets)
            SetRegFacet(LLReg(LL_RT_EFLAGS, 0), facet, undef);
    }

    void SetInsertBlock(llvm::BasicBlock* new_block) {
        regfile.SetInsertBlock(new_block);
        irb.SetInsertPoint(new_block);
    }

    // Operand handling implemented in lloperand.cc
private:
    llvm::Value* OpAddrConst(uint64_t addr, llvm::PointerType* ptr_ty);
protected:
    llvm::Value* OpAddr(const LLInstrOp& op, llvm::Type* element_type);
    llvm::Value* OpLoad(const LLInstrOp& op, Facet dataType, Alignment alignment = ALIGN_NONE);
    void OpStoreGp(const LLInstrOp& op, llvm::Value* value, Alignment alignment = ALIGN_NONE);
    void OpStoreVec(const LLInstrOp& op, llvm::Value* value, bool avx = false, Alignment alignment = ALIGN_IMP);
    void StackPush(llvm::Value* value);
    llvm::Value* StackPop(const LLReg sp_src_reg = LLReg(LL_RT_GP64, LL_RI_SP));

    // llflags.cc
    void FlagCalcZ(llvm::Value* value) {
        auto zero = llvm::Constant::getNullValue(value->getType());
        SetFlag(Facet::ZF, irb.CreateICmpEQ(value, zero));
    }
    void FlagCalcS(llvm::Value* value) {
        auto zero = llvm::Constant::getNullValue(value->getType());
        SetFlag(Facet::SF, irb.CreateICmpSLT(value, zero));
    }
    void FlagCalcP(llvm::Value* value);
    void FlagCalcA(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    void FlagCalcCAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs) {
        SetFlag(Facet::CF, irb.CreateICmpULT(res, lhs));
    }
    void FlagCalcCSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs) {
        SetFlag(Facet::CF, irb.CreateICmpULT(lhs, rhs));
    }
    void FlagCalcOAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    void FlagCalcOSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);

    llvm::Value* FlagCond(Condition cond);
    llvm::Value* FlagAsReg(unsigned size);

    enum RepMode { REP, REPZ, REPNZ };
    template<typename F>
    void WrapRep(F func, std::initializer_list<llvm::Value**> phis_il,
                 RepMode repmode = REP) {
        auto header_block = irb.GetInsertBlock();
        auto fn = header_block->getParent();
        auto loop_block = llvm::BasicBlock::Create(irb.getContext(), "", fn);
        auto cont_block = llvm::BasicBlock::Create(irb.getContext(), "", fn);

        // In header block
        auto count = GetReg(LLReg(LL_RT_GP64, LL_RI_C), Facet::I64);
        auto zero = llvm::Constant::getNullValue(count->getType());
        auto enter_loop = irb.CreateICmpNE(count, zero);
        irb.CreateCondBr(enter_loop, loop_block, cont_block);

        // This vector contains the given variables to be wrapped as PHI nodes
        // *and* the count/rcx variable.
        std::vector<llvm::Value**> phis;
        phis.push_back(&count);
        for (auto phi_var : phis_il)
            phis.push_back(phi_var);

        // In loop block
        irb.SetInsertPoint(loop_block);
        // Create PHIs in the loop block merging the loop header and (later) the
        // end of the loop block.
        std::vector<llvm::PHINode*> loop_phis;
        for (auto phi_var : phis) {
            auto loop_phi = irb.CreatePHI((*phi_var)->getType(), 2);
            loop_phi->addIncoming(*phi_var, header_block);

            loop_phis.push_back(loop_phi);
            // Use PHI node of loop block.
            *phi_var = loop_phi;
        }

        // Decrement count and check.
        count = irb.CreateSub(count, irb.getInt64(1));
        auto continue_loop = irb.CreateICmpNE(count, zero);

        if (repmode == REP) {
            func();
        } else {
            llvm::Value* zf = func();
            if (repmode == REPNZ)
                zf = irb.CreateNot(zf);
            continue_loop = irb.CreateAnd(continue_loop, zf);
        }

        irb.CreateCondBr(continue_loop, loop_block, cont_block);

        // In continuation block
        // Now also point the regfile/block/... to the new block, as code
        // generation shall continue there afterwards.
        SetInsertBlock(cont_block);
        for (size_t i = 0; i < phis.size(); i++) {
            // Add final value of loop to loop PHI nodes
            auto& loop_phi = loop_phis[i];
            auto header_val = loop_phi->getIncomingValueForBlock(header_block);
            loop_phi->addIncoming(*(phis[i]), loop_block);

            // Create PHI node in continuation, containing the value from the
            // header block (retrieved via the loop PHI node) and the value from
            // the loop block.
            auto cont_phi = irb.CreatePHI(loop_phi->getType(), 2);
            cont_phi->addIncoming(header_val, header_block);
            cont_phi->addIncoming(*(phis[i]), loop_block);
            // Use PHI node of continuation block
            *(phis[i]) = cont_phi;
        }

        SetReg(LLReg(LL_RT_GP64, LL_RI_C), Facet::I64, count);
    }
};

class Lifter : public LifterBase {
public:
    Lifter(const LLConfig& cfg, RegFile& rf) : LifterBase(cfg, rf) {}

    // llinstruction-gp.cc
    void LiftOverride(const LLInstr&, llvm::Function* override);

    void LiftMovgp(const LLInstr&, llvm::Instruction::CastOps cast);
    void LiftAdd(const LLInstr&);
    void LiftSub(const LLInstr&);
    void LiftSbb(const LLInstr&);
    void LiftCmp(const LLInstr&);
    void LiftCmpxchg(const LLInstr&);
    void LiftXchg(const LLInstr&);
    void LiftAndOrXor(const LLInstr& inst, llvm::Instruction::BinaryOps op,
                      bool writeback = true);
    void LiftNot(const LLInstr&);
    void LiftNeg(const LLInstr&);
    void LiftIncDec(const LLInstr&);
    void LiftShift(const LLInstr&, llvm::Instruction::BinaryOps);
    void LiftRotate(const LLInstr&);
    void LiftMul(const LLInstr&);
    void LiftDiv(const LLInstr&);
    void LiftLea(const LLInstr&);
    void LiftCmovcc(const LLInstr& inst, Condition cond);
    void LiftSetcc(const LLInstr& inst, Condition cond);
    void LiftCdqe(const LLInstr& inst);
    void LiftBitscan(const LLInstr& inst, bool trailing);

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
        OpStoreGp(LLInstrOp(LLReg(LL_RT_GP64, LL_RI_BP)), val);
    }

    void LiftJmp(const LLInstr& inst);
    void LiftJcc(const LLInstr& inst, Condition cond);
    void LiftCall(const LLInstr& inst);
    void LiftRet(const LLInstr& inst);

    void LiftCld(const LLInstr& inst) { SetFlag(Facet::DF, irb.getFalse()); }
    void LiftStd(const LLInstr& inst) { SetFlag(Facet::DF, irb.getTrue()); }
    void LiftStos(const LLInstr& inst);
    void LiftMovs(const LLInstr& inst);
    void LiftScas(const LLInstr& inst);

    // llinstruction-sse.cc
    void LiftPrefetch(const LLInstr&, unsigned rw, unsigned locality);
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
    void LiftSsePshufd(const LLInstr&);
    void LiftSseInsertps(const LLInstr&);
    void LiftSsePslldq(const LLInstr&);
    void LiftSsePcmp(const LLInstr&, llvm::CmpInst::Predicate, Facet);
    void LiftSsePmovmskb(const LLInstr&);
};

} // namespace

#endif
