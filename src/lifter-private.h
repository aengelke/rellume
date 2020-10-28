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

#ifndef RELLUME_LIFTER_PRIVATE_H
#define RELLUME_LIFTER_PRIVATE_H

#include "basicblock.h"
#include "config.h"
#include "facet.h"
#include "function-info.h"
#include "instr.h"
#include "regfile.h"
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
private:
    FunctionInfo& fi;
    ArchBasicBlock& ablock;

    /// Current register file
    RegFile* regfile;

protected:
    llvm::IRBuilder<> irb;
    const LLConfig& cfg;

    LifterBase(FunctionInfo& fi, const LLConfig& cfg, ArchBasicBlock& ab)
            : fi(fi), ablock(ab),regfile(ab.GetInsertBlock()->GetRegFile()),
              irb(regfile->GetInsertBlock()), cfg(cfg) {
        // Set fast-math flags. Newer LLVM supports FastMathFlags::getFast().
        if (cfg.enableFastMath) {
            llvm::FastMathFlags fmf;
            fmf.setFast();
            irb.setFastMathFlags(fmf);
        }
    }

    LifterBase(LifterBase&& rhs);
    LifterBase& operator=(LifterBase&& rhs);

    LifterBase(const LifterBase&) = delete;
    LifterBase& operator=(const LifterBase&) = delete;

    llvm::Module* GetModule() {
        return irb.GetInsertBlock()->getModule();
    }

    ArchReg MapReg(const Instr::Reg reg);

    llvm::Value* GetReg(ArchReg reg, Facet facet) {
        return regfile->GetReg(reg, facet);
    }
    void SetReg(ArchReg reg, Facet facet, llvm::Value* value) {
        regfile->SetReg(reg, facet, value, true); // clear all other facets
    }
    void SetRegFacet(ArchReg reg, Facet facet, llvm::Value* value) {
        regfile->SetReg(reg, facet, value, false);
    }
    void SetRegPtr(ArchReg reg, llvm::Value* value) {
        SetReg(reg, Facet::I64, irb.CreatePtrToInt(value, irb.getInt64Ty()));
        SetRegFacet(reg, Facet::PTR, value);
    }
    llvm::Value* GetFlag(Facet facet) {
        return GetReg(ArchReg::EFLAGS, facet);
    }
    void SetFlag(Facet facet, llvm::Value* value) {
        SetRegFacet(ArchReg::EFLAGS, facet, value);
    }
    void SetFlagUndef(std::initializer_list<Facet> facets) {
        llvm::Value* undef = llvm::UndefValue::get(irb.getInt1Ty());
        for (const auto facet : facets)
            SetFlag(facet, undef);
    }
    void SetIP(uint64_t inst_addr, bool nofold = false);

private:
    void SetInsertBlock(BasicBlock* block) {
        ablock.SetInsertBlock(block);
        regfile = block->GetRegFile();
        irb.SetInsertPoint(regfile->GetInsertBlock());
    }

    llvm::Value* OpAddrConst(uint64_t addr, llvm::PointerType* ptr_ty);
protected:
    llvm::Value* OpAddr(const Instr::Op op, llvm::Type* element_type, unsigned seg);
    llvm::Value* OpLoad(const Instr::Op op, Facet facet, Alignment alignment = ALIGN_NONE, unsigned force_seg = 7);
    void OpStoreGp(ArchReg reg, llvm::Value* v) {
        OpStoreGp(reg, Facet::In(v->getType()->getIntegerBitWidth()), v);
    }
    void OpStoreGp(ArchReg reg, Facet facet, llvm::Value* value);
    void OpStoreGp(const Instr::Op op, llvm::Value* value, Alignment alignment = ALIGN_NONE);
    void OpStoreVec(const Instr::Op op, llvm::Value* value, bool avx = false, Alignment alignment = ALIGN_IMP);
    void StackPush(llvm::Value* value);
    llvm::Value* StackPop(const ArchReg sp_src_reg = ArchReg::RSP);

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
    void FlagCalcAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs,
                     bool skip_carry = false);
    void FlagCalcSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs,
                     bool skip_carry = false, bool alt_zf = false);

    llvm::Value* FlagCond(Condition cond);
    llvm::Value* FlagAsReg(unsigned size);
    void FlagFromReg(llvm::Value* val);

    struct RepInfo {
        enum RepMode { NO_REP, REP, REPZ, REPNZ };
        RepMode mode;
        BasicBlock* loop_block;
        BasicBlock* cont_block;

        llvm::Value* di;
        llvm::Value* si;
        llvm::Value* ip;
    };
    RepInfo RepBegin(const Instr& inst);
    void RepEnd(RepInfo info);

    // Helper function for older LLVM versions
    llvm::Value* CreateUnaryIntrinsic(llvm::Intrinsic::ID id, llvm::Value* v) {
        // TODO: remove this helper function
        return irb.CreateUnaryIntrinsic(id, v);
    }

    void CallExternalFunction(llvm::Function* fn);

    void ForceReturn() {
        cfg.callconv.Return(ablock.GetInsertBlock(), fi);
    }
};

class Lifter : public LifterBase {
public:
    Lifter(FunctionInfo& fi, const LLConfig& cfg, ArchBasicBlock& ab) :
            LifterBase(fi, cfg, ab) {}

    // llinstruction-gp.cc
    bool Lift(const Instr&);

private:
    void LiftMovgp(const Instr&, llvm::Instruction::CastOps cast);
    void LiftArith(const Instr&, bool sub);
    void LiftCmpxchg(const Instr&);
    void LiftXchg(const Instr&);
    void LiftAndOrXor(const Instr& inst, llvm::Instruction::BinaryOps op,
                      bool writeback = true);
    void LiftNot(const Instr&);
    void LiftNeg(const Instr&);
    void LiftIncDec(const Instr&);
    void LiftShift(const Instr&, llvm::Instruction::BinaryOps);
    void LiftRotate(const Instr&);
    void LiftShiftdouble(const Instr&);
    void LiftMul(const Instr&);
    void LiftDiv(const Instr&);
    void LiftLea(const Instr&);
    void LiftXlat(const Instr&);
    void LiftCmovcc(const Instr& inst, Condition cond);
    void LiftSetcc(const Instr& inst, Condition cond);
    void LiftCext(const Instr& inst);
    void LiftCsep(const Instr& inst);
    void LiftBitscan(const Instr& inst, bool trailing);
    void LiftBittest(const Instr& inst);
    void LiftMovbe(const Instr& inst);
    void LiftBswap(const Instr& inst);

    void LiftPush(const Instr& inst) {
        StackPush(OpLoad(inst.op(0), Facet::I));
    }
    void LiftPushf(const Instr& inst) {
        StackPush(FlagAsReg(inst.opsz() * 8));
    }
    void LiftPop(const Instr& inst) {
        OpStoreGp(inst.op(0), StackPop());
    }
    void LiftPopf(const Instr& inst) {
        FlagFromReg(StackPop());
    }
    void LiftLeave(const Instr& inst) {
        llvm::Value* val = StackPop(ArchReg::RBP);
        OpStoreGp(ArchReg::RBP, val);
    }

    void LiftJmp(const Instr& inst);
    void LiftJcc(const Instr& inst, Condition cond);
    void LiftJcxz(const Instr& inst);
    void LiftLoop(const Instr& inst);
    void LiftCall(const Instr& inst);
    void LiftRet(const Instr& inst);
    void LiftSyscall(const Instr& inst);

    void LiftLods(const Instr& inst);
    void LiftStos(const Instr& inst);
    void LiftMovs(const Instr& inst);
    void LiftScas(const Instr& inst);
    void LiftCmps(const Instr& inst);

    // llinstruction-sse.cc
    void LiftFence(const Instr&);
    void LiftPrefetch(const Instr&, unsigned rw, unsigned locality);
    void LiftFxsave(const Instr&);
    void LiftFxrstor(const Instr&);
    void LiftFstcw(const Instr&);
    void LiftFstsw(const Instr&);
    void LiftStmxcsr(const Instr&);
    void LiftSseMovq(const Instr&, Facet type);
    void LiftSseBinOp(const Instr&, llvm::Instruction::BinaryOps op,
                      Facet type);
    void LiftSseMovScalar(const Instr&, Facet);
    void LiftSseMovdq(const Instr&, Facet, Alignment);
    void LiftSseMovntStore(const Instr&, Facet);
    void LiftSseMovlp(const Instr&);
    void LiftSseMovhps(const Instr&);
    void LiftSseMovhpd(const Instr&);
    void LiftSseAndn(const Instr&, Facet op_type);
    void LiftSseComis(const Instr&, Facet);
    void LiftSseCmp(const Instr&, Facet op_type);
    void LiftSseMinmax(const Instr&, llvm::CmpInst::Predicate, Facet);
    void LiftSseSqrt(const Instr&, Facet op_type);
    void LiftSseCvt(const Instr&, Facet src_type, Facet dst_type);
    void LiftSseUnpck(const Instr&, Facet type);
    void LiftSseShufpd(const Instr&);
    void LiftSseShufps(const Instr&);
    void LiftSsePshufd(const Instr&);
    void LiftSsePshufw(const Instr&, unsigned off);
    void LiftSseInsertps(const Instr&);
    void LiftSsePinsr(const Instr&, Facet, Facet, unsigned);
    void LiftSsePextr(const Instr&, Facet, unsigned);
    void LiftSsePshiftElement(const Instr&, llvm::Instruction::BinaryOps op, Facet op_type);
    void LiftSsePshiftBytes(const Instr&);
    void LiftSsePavg(const Instr&, Facet);
    void LiftSsePmulhw(const Instr&, llvm::Instruction::CastOps cast);
    void LiftSsePmuludq(const Instr&);
    void LiftSsePaddsubSaturate(const Instr& inst,
                                llvm::Instruction::BinaryOps calc_op, bool sign,
                                Facet op_ty);
    void LiftSsePack(const Instr&, Facet, bool sign);
    void LiftSsePcmp(const Instr&, llvm::CmpInst::Predicate, Facet);
    void LiftSsePminmax(const Instr&, llvm::CmpInst::Predicate, Facet);
    void LiftSseMovmsk(const Instr&, Facet op_type);
};

} // namespace

#endif
