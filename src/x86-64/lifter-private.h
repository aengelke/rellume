/**
 * This file is part of Rellume.
 *
 * (c) 2016-2019, Alexis Engelke <alexis.engelke@googlemail.com>
 * (c) 2020, Dominik Okwieka <dominik.okwieka@t-online.de>
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

#ifndef RELLUME_X64_LIFTER_PRIVATE_H
#define RELLUME_X64_LIFTER_PRIVATE_H

#include "basicblock.h"
#include "config.h"
#include "facet.h"
#include "function-info.h"
#include "instr.h"
#include "lifter-base.h"
#include "regfile.h"
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Operator.h>
#include <vector>


namespace rellume::x86_64 {

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

class Lifter : public LifterBase {
public:
    Lifter(FunctionInfo& fi, const LLConfig& cfg, ArchBasicBlock& ab) :
            LifterBase(fi, cfg, ab) {}

    bool Lift(const Instr&);

private:
    // Helper function for f***ing LLVM providing overloads for signed and
    // unsigned int, preventing the use of brace initializers.
    llvm::Value* CreateShuffleVector(llvm::Value* a, llvm::Value* b,
                                     llvm::ArrayRef<int> msk) {
        return irb.CreateShuffleVector(a, b, msk);
    }
    unsigned VectorElementCount(llvm::Type* ty) {
        auto ec = llvm::cast<llvm::VectorType>(ty)->getElementCount();
        return ec.getFixedValue();
    }
    ArchReg MapReg(const Instr::Reg reg);

    void StoreGp(ArchReg reg, llvm::Value* v) {
        StoreGpFacet(reg, Facet::In(v->getType()->getIntegerBitWidth()), v);
    }
    void StoreGpFacet(ArchReg reg, Facet facet, llvm::Value* value);
    llvm::Value* OpAddr(const Instr::Op op, llvm::Type* element_type, unsigned seg = 7);
    llvm::Value* OpLoad(const Instr::Op op, Facet facet, Alignment alignment = ALIGN_NONE, unsigned force_seg = 7);
    void OpStoreGp(const Instr::Op op, llvm::Value* value, Alignment alignment = ALIGN_NONE);
    void OpStoreVec(const Instr::Op op, llvm::Value* value, Alignment alignment = ALIGN_IMP);
    void StackPush(llvm::Value* value);
    llvm::Value* StackPop(const ArchReg sp_src_reg = ArchReg::RSP);

    void FlagCalcZ(llvm::Value* value) {
        regfile->Set(ArchReg::ZF, RegFile::Transform::IsZero, value);
    }
    // Set SF and PF according to result, AF is undefined.
    void FlagCalcSAPLogic(llvm::Value* res);
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

        llvm::Type* ty;
        llvm::Value* di;
        llvm::Value* si;
        uint64_t ip;
    };
    RepInfo RepBegin(const Instr& inst);
    void RepEnd(RepInfo info);

    void LiftMovgp(const Instr&);
    void LiftMovzx(const Instr&);
    void LiftArith(const Instr&, bool sub);
    void LiftCmpxchg(const Instr&);
    void LiftXchg(const Instr&);
    void LiftAndOrXor(const Instr& inst, llvm::Instruction::BinaryOps op,
                      llvm::AtomicRMWInst::BinOp a_op, bool writeback = true);
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
    void LiftPopcnt(const Instr& inst);
    void LiftBittest(const Instr& inst, llvm::Instruction::BinaryOps op,
                     llvm::AtomicRMWInst::BinOp atomic_op);
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
        StoreGp(ArchReg::RBP, val);
    }

    void LiftJmp(const Instr& inst);
    void LiftJcc(const Instr& inst, Condition cond);
    void LiftJcxz(const Instr& inst);
    void LiftLoop(const Instr& inst);
    void LiftCall(const Instr& inst);
    void LiftRet(const Instr& inst);
    void LiftSyscall(const Instr& inst);
    void LiftCpuid(const Instr& inst);
    void LiftRdtsc(const Instr& inst);

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
    void LiftSseHorzOp(const Instr&, llvm::Instruction::BinaryOps op,
                      Facet type);
    void LiftSseMovScalar(const Instr&, Facet);
    void LiftSseMovdq(const Instr&, Facet, Alignment);
    void LiftSseMovntStore(const Instr&, Facet);
    void LiftSseMovlp(const Instr&);
    void LiftSseMovhps(const Instr&);
    void LiftSseMovhpd(const Instr&);
    void LiftSseAddSub(const Instr&, Facet op_type);
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
    void LiftSseMovdup(const Instr&, Facet, unsigned off);
    void LiftSsePshiftElement(const Instr&, llvm::Instruction::BinaryOps op, Facet op_type);
    void LiftSsePshiftBytes(const Instr&);
    void LiftSsePavg(const Instr&, Facet);
    void LiftSsePmulhw(const Instr&, llvm::Instruction::CastOps cast);
    void LiftSsePmuldq(const Instr&, llvm::Instruction::CastOps ext);
    void LiftSsePmaddwd(const Instr&);
    void LiftSsePaddsubSaturate(const Instr& inst,
                                llvm::Instruction::BinaryOps calc_op, bool sign,
                                Facet op_ty);
    void LiftSsePsadbw(const Instr&);
    void LiftSsePmaddubsw(const Instr&);
    void LiftSsePack(const Instr&, Facet, bool sign);
    void LiftSsePcmp(const Instr&, llvm::CmpInst::Predicate, Facet);
    void LiftSsePminmax(const Instr&, llvm::CmpInst::Predicate, Facet);
    void LiftSsePabs(const Instr&, Facet);
    void LiftSseMovmsk(const Instr&, Facet op_type);
    void LiftSsePmovx(const Instr&, llvm::Instruction::CastOps ext, Facet from, Facet to);
};

} // namespace::x86_64

#endif
