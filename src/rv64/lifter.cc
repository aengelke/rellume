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

#include "rv64/lifter.h"

#include "facet.h"
#include "instr.h"
#include "lifter-base.h"
#include "regfile.h"

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Transforms/Utils/Cloning.h>

namespace rellume::rv64 {

class Lifter : public LifterBase {
public:
    Lifter(FunctionInfo& fi, const LLConfig& cfg, ArchBasicBlock& ab) :
            LifterBase(fi, cfg, ab) {}

    bool Lift(const Instr&);

    llvm::Value* LoadGp(unsigned reg, Facet facet = Facet::I64) {
        if (reg == 0)
            return llvm::Constant::getNullValue(facet.Type(irb.getContext()));
        return GetReg(ArchReg::GP(reg), facet);
    }
    llvm::Value* LoadFp(unsigned reg, Facet facet) {
        return GetReg(ArchReg::VEC(reg), facet);
    }
    void StoreGp(unsigned reg, llvm::Value* v) {
        assert(v->getType()->isIntegerTy());
        if (reg == 0)
            return;
        // TODO: do sign extension lazily
        SetReg(ArchReg::GP(reg), irb.CreateSExt(v, irb.getInt64Ty()));
    }
    void StoreFp(unsigned reg, llvm::Value* v) {
        SetReg(ArchReg::VEC(reg), v);
    }
    llvm::Value* Addr(const FrvInst* rvi) {
        llvm::Value* base = LoadGp(rvi->rs1, Facet::PTR);
        if (llvm::isa<llvm::Constant>(base)) {
            llvm::Value* base_int = LoadGp(rvi->rs1, Facet::I64);
            base_int = irb.CreateAdd(base_int, irb.getInt64(rvi->imm));
            if (auto* addr = llvm::dyn_cast<llvm::ConstantInt>(base_int)) {
                base = AddrConst(addr->getZExtValue());
            } else {
                base = irb.CreateIntToPtr(base_int, irb.getPtrTy());
            }
        } else if (rvi->imm) {
            base = irb.CreateGEP(irb.getInt8Ty(), base, irb.getInt64(rvi->imm));
        }
        return base;
    }

    void LiftBinOpR(const FrvInst* rvi, llvm::Instruction::BinaryOps op,
                    Facet f) {
        auto res = irb.CreateBinOp(op, LoadGp(rvi->rs1, f), LoadGp(rvi->rs2, f));
        StoreGp(rvi->rd, res);
    }
    void LiftBinOpI(const FrvInst* rvi, llvm::Instruction::BinaryOps op,
                    Facet f) {
        llvm::Value* imm = irb.getIntN(f.Size(), rvi->imm);
        StoreGp(rvi->rd, irb.CreateBinOp(op, LoadGp(rvi->rs1, f), imm));
    }
    void LiftShift(const FrvInst* rvi, llvm::Instruction::BinaryOps op,
                   llvm::Value* shiftop) {
        unsigned width = shiftop->getType()->getIntegerBitWidth();
        shiftop = irb.CreateAnd(shiftop, irb.getIntN(width, width - 1));
        llvm::Value* src = LoadGp(rvi->rs1, Facet::In(width));
        StoreGp(rvi->rd, irb.CreateBinOp(op, src, shiftop));
    }
    void LiftMulh(const FrvInst* rvi, llvm::Instruction::CastOps ext1,
                  llvm::Instruction::CastOps ext2) {
        llvm::Type* tgt_ty = irb.getIntNTy(128);
        auto rs1 = irb.CreateCast(ext1, LoadGp(rvi->rs1, Facet::I64), tgt_ty);
        auto rs2 = irb.CreateCast(ext2, LoadGp(rvi->rs2, Facet::I64), tgt_ty);
        llvm::Value* res = irb.CreateLShr(irb.CreateMul(rs1, rs2), int{64});
        StoreGp(rvi->rd, irb.CreateTrunc(res, irb.getInt64Ty()));
    }
    void LiftLoad(const FrvInst* rvi, llvm::Instruction::CastOps ext, Facet f) {
        llvm::Type* ty = f.Type(irb.getContext());
        llvm::Value* ld = irb.CreateLoad(ty, Addr(rvi));
        StoreGp(rvi->rd, irb.CreateCast(ext, ld, irb.getInt64Ty()));
    }
    void LiftLoadFp(const FrvInst* rvi, Facet f) {
        llvm::Type* ty = f.Type(irb.getContext());
        StoreFp(rvi->rd, irb.CreateLoad(ty, Addr(rvi)));
    }
    void LiftStore(const FrvInst* rvi, Facet f) {
        irb.CreateStore(LoadGp(rvi->rs2, f), Addr(rvi));
    }
    void LiftStoreFp(const FrvInst* rvi, Facet f) {
        irb.CreateStore(LoadFp(rvi->rs2, f), Addr(rvi));
    }
    void LiftBranch(const Instr& inst, llvm::CmpInst::Predicate pred) {
        const FrvInst* rvi = inst;
        auto cond = irb.CreateICmp(pred, LoadGp(rvi->rs1), LoadGp(rvi->rs2));
        SetReg(ArchReg::IP, irb.CreateSelect(cond,
            AddrIPRel(rvi->imm, Facet::I64),
            AddrIPRel(inst.len(), Facet::I64)
        ));
    }
    void LiftDivRem(const FrvInst* rvi, llvm::Instruction::BinaryOps op,
                     Facet f) {
        llvm::Value* zero = irb.getIntN(f.Size(), 0);
        llvm::Value* minusone = irb.getIntN(f.Size(), -1);
        llvm::Value* dividend = LoadGp(rvi->rs1, f);
        llvm::Value* divisor = LoadGp(rvi->rs2, f);

        llvm::Value* is_zero = irb.CreateICmpEQ(divisor, zero);
        llvm::Value* is_minusone = nullptr;
        llvm::Value* is_spc;
        if (op == llvm::Instruction::SDiv || op == llvm::Instruction::SRem) {
            is_minusone = irb.CreateICmpEQ(divisor, minusone);
            is_spc = irb.CreateOr(is_zero, is_minusone);
        } else {
            is_spc = is_zero;
        }

        BasicBlock* div_block = ablock.AddBlock();
        BasicBlock* spc_block = ablock.AddBlock();
        BasicBlock* cont_block = ablock.AddBlock();
        ablock.GetInsertBlock()->BranchTo(is_spc, *spc_block, *div_block);

        SetInsertBlock(div_block);
        StoreGp(rvi->rd, irb.CreateBinOp(op, dividend, divisor));
        ablock.GetInsertBlock()->BranchTo(*cont_block);

        SetInsertBlock(spc_block);
        switch (op) {
        case llvm::Instruction::UDiv: StoreGp(rvi->rd, minusone); break;
        case llvm::Instruction::URem: StoreGp(rvi->rd, dividend); break;
        case llvm::Instruction::SDiv:
            StoreGp(rvi->rd, irb.CreateSelect(is_minusone, dividend, minusone));
            break;
        case llvm::Instruction::SRem:
            StoreGp(rvi->rd, irb.CreateSelect(is_minusone, zero, dividend));
            break;
        default: assert(false);
        }
        ablock.GetInsertBlock()->BranchTo(*cont_block);

        SetInsertBlock(cont_block);
    }

    void LiftAmo(const FrvInst* rvi, llvm::AtomicRMWInst::BinOp op, Facet f) {
        llvm::AtomicOrdering ordering;
        switch (rvi->imm) {
        case 0: ordering = llvm::AtomicOrdering::Monotonic; break;
        case 1: ordering = llvm::AtomicOrdering::Release; break;
        case 2: ordering = llvm::AtomicOrdering::Acquire; break;
        case 3: ordering = llvm::AtomicOrdering::SequentiallyConsistent; break;
        default: assert(false && "invalid memory ordering");
        }
        llvm::Value* val = LoadGp(rvi->rs2, f);
        llvm::Value* ptr = LoadGp(rvi->rs1, Facet::PTR);
        ptr = irb.CreatePointerCast(ptr, f.Type(irb.getContext())->getPointerTo());
        StoreGp(rvi->rd, irb.CreateAtomicRMW(op, ptr, val, {}, ordering));
    }

    void LiftFpArith(const FrvInst* rvi, llvm::Instruction::BinaryOps op,
                     Facet f) {
        assert(rvi->misc == 7 && "only rm=DYN supported");
        auto res = irb.CreateBinOp(op, LoadFp(rvi->rs1, f), LoadFp(rvi->rs2, f));
        StoreFp(rvi->rd, res);
    }
    void LiftFcvtIToF(const FrvInst* rvi, Facet df, Facet sf,
                  llvm::Instruction::CastOps cast) {
        // TODO: support rounding modes for int-to-fp conversions.
        // For now, we assume that values are small enough to be representable
        // in the target FP type without any rounding (=exact conversions).
        llvm::Type* tgt_ty = df.Type(irb.getContext());
        StoreFp(rvi->rd, irb.CreateCast(cast, LoadGp(rvi->rs1, sf), tgt_ty));
    }
    void LiftFcvtFToI(const FrvInst* rvi, Facet df, Facet sf,
                  llvm::Instruction::CastOps cast) {
        llvm::Value* v = LoadFp(rvi->rs1, sf);
        switch (rvi->misc) {
        // case 0: // TODO: RNE requires llvm::Intrinsic::roundeven
        case 1: break; // RTZ = default for LLVM fp-to-int conversions
        case 2: // RDN
            v = irb.CreateUnaryIntrinsic(llvm::Intrinsic::floor, v); break;
        case 3: // RUP
            v = irb.CreateUnaryIntrinsic(llvm::Intrinsic::ceil, v); break;
        case 4: // RMM
            v = irb.CreateUnaryIntrinsic(llvm::Intrinsic::round, v); break;
        case 7: break; // DYN = we don't care, for now. TODO: support DYN RM
                       // Actually, this is usually the same as RNE, see above.
        default: assert(false && "unsupported rounding mode in F2I");
        }
        StoreGp(rvi->rd, irb.CreateCast(cast, v, df.Type(irb.getContext())));
    }
    void LiftFcvtFToF(const FrvInst* rvi, Facet df, Facet sf,
                  llvm::Instruction::CastOps cast) {
        assert(rvi->misc == 7 && "only rm=DYN supported");
        llvm::Type* tgt_ty = df.Type(irb.getContext());
        StoreFp(rvi->rd, irb.CreateCast(cast, LoadFp(rvi->rs1, sf), tgt_ty));
    }
    void LiftFcmp(const FrvInst* rvi, llvm::CmpInst::Predicate pred, Facet f) {
        auto res = irb.CreateFCmp(pred, LoadFp(rvi->rs1, f), LoadFp(rvi->rs2, f));
        StoreGp(rvi->rd, irb.CreateZExt(res, irb.getInt64Ty()));
    }
    void LiftFsgn(const FrvInst* rvi, Facet f, bool keep, bool zero) {
        unsigned sz = f == Facet::F32 ? 32 : 64;
        llvm::Type* int_ty = irb.getIntNTy(sz);
        llvm::Value* abs_op = irb.CreateBitCast(LoadFp(rvi->rs1, f), int_ty);
        llvm::Value* sign_op = irb.CreateBitCast(LoadFp(rvi->rs2, f), int_ty);
        if (!keep && zero)
            abs_op = irb.CreateAnd(abs_op, irb.getIntN(sz, (1ul << (sz-1)) - 1));
        else if (!keep && !zero)
            abs_op = irb.CreateOr(abs_op, irb.getIntN(sz, 1ul << (sz-1)));
        sign_op = irb.CreateAnd(sign_op, irb.getIntN(sz, 1ul << (sz-1)));
        abs_op = irb.CreateXor(abs_op, sign_op);
        StoreFp(rvi->rd, irb.CreateBitCast(abs_op, f.Type(irb.getContext())));
    }
    void LiftFminmax(const FrvInst* rvi, llvm::Intrinsic::ID id, Facet f) {
        auto res = irb.CreateBinaryIntrinsic(id, LoadFp(rvi->rs1, f), LoadFp(rvi->rs2, f));
        StoreFp(rvi->rd, res);
    }
    void LiftFclass(const FrvInst* rvi, Facet fi) {
        llvm::Value* v = LoadFp(rvi->rs1, fi);
        unsigned sz = fi.Size();
        assert(sz == 32 || sz == 64);
        uint64_t expmsk = sz == 32 ? 0x7f800000 : 0x7ff0000000000000;
        uint64_t mantmsk = sz == 32 ? 0x007fffff : 0x000fffffffffffff;
        unsigned nanidx = sz == 32 ? 22 : 51;

        // TODO: find a more performant way for FP classification
        llvm::Value* sign = irb.CreateAnd(v, irb.getIntN(sz, 1ul << (sz-1)));
        llvm::Value* exp = irb.CreateAnd(v, irb.getIntN(sz, expmsk));
        llvm::Value* expzero = irb.CreateICmpEQ(exp, irb.getIntN(sz, 0));
        llvm::Value* expmax = irb.CreateICmpEQ(exp, irb.getIntN(sz, expmsk));
        llvm::Value* mant = irb.CreateAnd(v, irb.getIntN(sz, mantmsk));
        llvm::Value* mantnotzero = irb.CreateICmpNE(mant, irb.getIntN(sz, 0));

        // First construct values 4..7
        llvm::Value* mze = irb.CreateZExt(mantnotzero, irb.getIntNTy(sz));
        mze = irb.CreateOr(mze, irb.getIntN(sz, 4));
        llvm::Value* mme = irb.CreateZExt(expmax, irb.getIntNTy(sz));
        mme = irb.CreateOr(mme, irb.getIntN(sz, 6));
        llvm::Value* res = irb.CreateSelect(expzero, mze, mme);

        // Values 0..3 are mirrored from values 4..7
        llvm::Value* resneg = irb.CreateXor(res, irb.getIntN(sz, 7));
        res = irb.CreateSelect(irb.CreateICmpEQ(sign, irb.getIntN(sz, 0)), res, resneg);

        // NaN (8/9) overrides others.
        llvm::Value* signaling = irb.CreateLShr(mant, nanidx);
        signaling = irb.CreateOr(signaling, irb.getIntN(sz, 8));
        llvm::Value* isnan = irb.CreateAnd(expmax, mantnotzero);
        res = irb.CreateSelect(isnan, signaling, res);
        StoreGp(rvi->rd, res);
    }
    void LiftFmadd(const FrvInst* rvi, bool sub, bool negprod, Facet f) {
        // TODO: actually implement as fused operation
        llvm::Value* op = irb.CreateFMul(LoadFp(rvi->rs1, f), LoadFp(rvi->rs2, f));
        if (negprod)
            op = irb.CreateFNeg(op);
        auto binop = sub ? llvm::Instruction::FSub : llvm::Instruction::FAdd;
        StoreFp(rvi->rd, irb.CreateBinOp(binop, op, LoadFp(rvi->rs3, f)));
    }
};

bool LiftInstruction(const Instr& inst, FunctionInfo& fi, const LLConfig& cfg,
                     ArchBasicBlock& ab) noexcept {
    return Lifter(fi, cfg, ab).Lift(inst);
}

bool Lifter::Lift(const Instr& inst) {
    // Set new instruction pointer register
    SetIP(inst.start());
    const FrvInst* rvi = inst;

    // TODO: Add instruction marker
    if (cfg.instr_marker) {
        llvm::Value* rip = GetReg(ArchReg::IP, Facet::I64);
        llvm::StringRef str_ref{reinterpret_cast<const char*>(rvi),
                                sizeof(FrvInst)};
        llvm::MDString* md = llvm::MDString::get(irb.getContext(), str_ref);
        llvm::Value* md_val = llvm::MetadataAsValue::get(irb.getContext(), md);
        irb.CreateCall(cfg.instr_marker, {rip, md_val});
    }

    // Check overridden implementations first.
    const auto& override = cfg.instr_overrides.find(rvi->mnem);
    if (override != cfg.instr_overrides.end()) {
        CallExternalFunction(override->second);
        return true;
    }

    switch (rvi->mnem) {
    default:
    unhandled:
        SetIP(inst.start(), /*nofold=*/true);
        return false;

    case FRV_JAL:
    case FRV_JALR: {
        llvm::Value* ret_addr = AddrIPRel(inst.len(), Facet::I64);
        if (rvi->rs1 != FRV_REG_INV) {
            auto tgt = irb.CreateAdd(LoadGp(rvi->rs1, Facet::I64), irb.getInt64(rvi->imm));
            SetReg(ArchReg::IP, tgt);
        } else {
            SetReg(ArchReg::IP, AddrIPRel(rvi->imm, Facet::I64));
        }
        StoreGp(rvi->rd, ret_addr);

        // For discussion, see x86-64 lifting of call/ret.
        if (cfg.call_function) {
            bool rdl = rvi->rd == 1 || rvi->rd == 5;
            bool rs1l = rvi->rs1 == 1 || rvi->rs1 == 5;
            if (!rdl && rs1l) {
                ForceReturn();
            } else if (rdl && (!rs1l || rvi->rs1 == rvi->rd)) {
                CallExternalFunction(cfg.call_function);
                llvm::Value* cont_addr = GetReg(ArchReg::IP, Facet::I64);
                llvm::Value* eq = irb.CreateICmpEQ(cont_addr, ret_addr);
                // This allows for optimization of the common case (equality).
                SetReg(ArchReg::IP, irb.CreateSelect(eq, ret_addr, cont_addr));
            }
        }
        return true;
    }
    case FRV_BEQ: LiftBranch(inst, llvm::CmpInst::ICMP_EQ); return true;
    case FRV_BNE: LiftBranch(inst, llvm::CmpInst::ICMP_NE); return true;
    case FRV_BLT: LiftBranch(inst, llvm::CmpInst::ICMP_SLT); return true;
    case FRV_BLTU: LiftBranch(inst, llvm::CmpInst::ICMP_ULT); return true;
    case FRV_BGE: LiftBranch(inst, llvm::CmpInst::ICMP_SGE); return true;
    case FRV_BGEU: LiftBranch(inst, llvm::CmpInst::ICMP_UGE); return true;
    case FRV_ECALL:
        if (rvi->imm == 0) {
            SetIP(inst.end(), /*nofold=*/true);
            if (cfg.syscall_implementation)
                CallExternalFunction(cfg.syscall_implementation);
            // Return here, syscall could have updated the PC.
            return true;
        } else {
            SetIP(inst.start(), /*nofold=*/true);
            return false;
        }
        break;

    case FRV_LUI: StoreGp(rvi->rd, irb.getInt64(rvi->imm)); break;
    case FRV_AUIPC: StoreGp(rvi->rd, AddrIPRel(rvi->imm, Facet::I64)); break;

    case FRV_LB: LiftLoad(rvi, llvm::Instruction::SExt, Facet::I8); break;
    case FRV_LBU: LiftLoad(rvi, llvm::Instruction::ZExt, Facet::I8); break;
    case FRV_LH: LiftLoad(rvi, llvm::Instruction::SExt, Facet::I16); break;
    case FRV_LHU: LiftLoad(rvi, llvm::Instruction::ZExt, Facet::I16); break;
    case FRV_LW: LiftLoad(rvi, llvm::Instruction::SExt, Facet::I32); break;
    case FRV_LWU: LiftLoad(rvi, llvm::Instruction::ZExt, Facet::I32); break;
    case FRV_LD: LiftLoad(rvi, llvm::Instruction::SExt, Facet::I64); break;
    case FRV_SB: LiftStore(rvi, Facet::I8); break;
    case FRV_SH: LiftStore(rvi, Facet::I16); break;
    case FRV_SW: LiftStore(rvi, Facet::I32); break;
    case FRV_SD: LiftStore(rvi, Facet::I64); break;

    case FRV_ADDI: LiftBinOpI(rvi, llvm::Instruction::Add, Facet::I64); break;
    case FRV_ADD: LiftBinOpR(rvi, llvm::Instruction::Add, Facet::I64); break;
    case FRV_ADDIW: LiftBinOpI(rvi, llvm::Instruction::Add, Facet::I32); break;
    case FRV_ADDW: LiftBinOpR(rvi, llvm::Instruction::Add, Facet::I32); break;
    case FRV_SUB: LiftBinOpR(rvi, llvm::Instruction::Sub, Facet::I64); break;
    case FRV_SUBW: LiftBinOpR(rvi, llvm::Instruction::Sub, Facet::I32); break;
    case FRV_ANDI: LiftBinOpI(rvi, llvm::Instruction::And, Facet::I64); break;
    case FRV_AND: LiftBinOpR(rvi, llvm::Instruction::And, Facet::I64); break;
    case FRV_ORI: LiftBinOpI(rvi, llvm::Instruction::Or, Facet::I64); break;
    case FRV_OR: LiftBinOpR(rvi, llvm::Instruction::Or, Facet::I64); break;
    case FRV_XORI: LiftBinOpI(rvi, llvm::Instruction::Xor, Facet::I64); break;
    case FRV_XOR: LiftBinOpR(rvi, llvm::Instruction::Xor, Facet::I64); break;
    case FRV_SLLI: LiftShift(rvi, llvm::Instruction::Shl, irb.getInt64(rvi->imm)); break;
    case FRV_SLL: LiftShift(rvi, llvm::Instruction::Shl, LoadGp(rvi->rs2, Facet::I64)); break;
    case FRV_SLLIW: LiftShift(rvi, llvm::Instruction::Shl, irb.getInt32(rvi->imm)); break;
    case FRV_SLLW: LiftShift(rvi, llvm::Instruction::Shl, LoadGp(rvi->rs2, Facet::I32)); break;
    case FRV_SRLI: LiftShift(rvi, llvm::Instruction::LShr, irb.getInt64(rvi->imm)); break;
    case FRV_SRL: LiftShift(rvi, llvm::Instruction::LShr, LoadGp(rvi->rs2, Facet::I64)); break;
    case FRV_SRLIW: LiftShift(rvi, llvm::Instruction::LShr, irb.getInt32(rvi->imm)); break;
    case FRV_SRLW: LiftShift(rvi, llvm::Instruction::LShr, LoadGp(rvi->rs2, Facet::I32)); break;
    case FRV_SRAI: LiftShift(rvi, llvm::Instruction::AShr, irb.getInt64(rvi->imm)); break;
    case FRV_SRA: LiftShift(rvi, llvm::Instruction::AShr, LoadGp(rvi->rs2, Facet::I64)); break;
    case FRV_SRAIW: LiftShift(rvi, llvm::Instruction::AShr, irb.getInt32(rvi->imm)); break;
    case FRV_SRAW: LiftShift(rvi, llvm::Instruction::AShr, LoadGp(rvi->rs2, Facet::I32)); break;
    case FRV_SLT: StoreGp(rvi->rd, irb.CreateZExt(irb.CreateICmpSLT(LoadGp(rvi->rs1), LoadGp(rvi->rs2)), irb.getInt64Ty())); break;
    case FRV_SLTI: StoreGp(rvi->rd, irb.CreateZExt(irb.CreateICmpSLT(LoadGp(rvi->rs1), irb.getInt64(rvi->imm)), irb.getInt64Ty())); break;
    case FRV_SLTU: StoreGp(rvi->rd, irb.CreateZExt(irb.CreateICmpULT(LoadGp(rvi->rs1), LoadGp(rvi->rs2)), irb.getInt64Ty())); break;
    case FRV_SLTIU: StoreGp(rvi->rd, irb.CreateZExt(irb.CreateICmpULT(LoadGp(rvi->rs1), irb.getInt64(rvi->imm)), irb.getInt64Ty())); break;

    case FRV_MUL: LiftBinOpR(rvi, llvm::Instruction::Mul, Facet::I64); break;
    case FRV_MULW: LiftBinOpR(rvi, llvm::Instruction::Mul, Facet::I32); break;
    case FRV_MULH: LiftMulh(rvi, llvm::Instruction::SExt, llvm::Instruction::SExt); break;
    case FRV_MULHSU: LiftMulh(rvi, llvm::Instruction::SExt, llvm::Instruction::ZExt); break;
    case FRV_MULHU: LiftMulh(rvi, llvm::Instruction::ZExt, llvm::Instruction::ZExt); break;
    case FRV_DIV: LiftDivRem(rvi, llvm::Instruction::SDiv, Facet::I64); break;
    case FRV_DIVU: LiftDivRem(rvi, llvm::Instruction::UDiv, Facet::I64); break;
    case FRV_DIVW: LiftDivRem(rvi, llvm::Instruction::SDiv, Facet::I32); break;
    case FRV_DIVUW: LiftDivRem(rvi, llvm::Instruction::UDiv, Facet::I32); break;
    case FRV_REM: LiftDivRem(rvi, llvm::Instruction::SRem, Facet::I64); break;
    case FRV_REMU: LiftDivRem(rvi, llvm::Instruction::URem, Facet::I64); break;
    case FRV_REMW: LiftDivRem(rvi, llvm::Instruction::SRem, Facet::I32); break;
    case FRV_REMUW: LiftDivRem(rvi, llvm::Instruction::URem, Facet::I32); break;

    case FRV_CSRRW:
    case FRV_CSRRS:
    case FRV_CSRRC:
    case FRV_CSRRWI:
    case FRV_CSRRSI:
    case FRV_CSRRCI: {
        switch (rvi->imm) {
        case 0x01: // fflags
        case 0x02: // frm
        case 0x03: // fcsr
            // TODO: actually implement FCSR writes.
            // This very complicated to map accurately to LLVM-IR.
            StoreGp(rvi->rd, irb.getInt32(0));
            break;
        default:
            goto unhandled;
        }
        break;
    }

    // TODO: implement atomic semantics for LR/SC.
    case FRV_LRW: StoreGp(rvi->rd, irb.CreateLoad(irb.getInt32Ty(), LoadGp(rvi->rs1, Facet::PTR))); break;
    case FRV_SCW: StoreGp(rvi->rd, irb.getInt32(0)); irb.CreateStore(LoadGp(rvi->rs2, Facet::I32), LoadGp(rvi->rs1, Facet::PTR)); break;
    case FRV_FENCE: break; // TODO: fences

    case FRV_AMOSWAPW: LiftAmo(rvi, llvm::AtomicRMWInst::Xchg, Facet::I32); break;
    case FRV_AMOSWAPD: LiftAmo(rvi, llvm::AtomicRMWInst::Xchg, Facet::I64); break;
    case FRV_AMOADDW: LiftAmo(rvi, llvm::AtomicRMWInst::Add, Facet::I32); break;
    case FRV_AMOADDD: LiftAmo(rvi, llvm::AtomicRMWInst::Add, Facet::I64); break;
    case FRV_AMOANDW: LiftAmo(rvi, llvm::AtomicRMWInst::And, Facet::I32); break;
    case FRV_AMOANDD: LiftAmo(rvi, llvm::AtomicRMWInst::And, Facet::I64); break;
    case FRV_AMOORW: LiftAmo(rvi, llvm::AtomicRMWInst::Or, Facet::I32); break;
    case FRV_AMOORD: LiftAmo(rvi, llvm::AtomicRMWInst::Or, Facet::I64); break;
    case FRV_AMOXORW: LiftAmo(rvi, llvm::AtomicRMWInst::Xor, Facet::I32); break;
    case FRV_AMOXORD: LiftAmo(rvi, llvm::AtomicRMWInst::Xor, Facet::I64); break;
    case FRV_AMOMAXW: LiftAmo(rvi, llvm::AtomicRMWInst::Max, Facet::I32); break;
    case FRV_AMOMAXD: LiftAmo(rvi, llvm::AtomicRMWInst::Max, Facet::I64); break;
    case FRV_AMOMAXUW: LiftAmo(rvi, llvm::AtomicRMWInst::UMax, Facet::I32); break;
    case FRV_AMOMAXUD: LiftAmo(rvi, llvm::AtomicRMWInst::UMax, Facet::I64); break;
    case FRV_AMOMINW: LiftAmo(rvi, llvm::AtomicRMWInst::Min, Facet::I32); break;
    case FRV_AMOMIND: LiftAmo(rvi, llvm::AtomicRMWInst::Min, Facet::I64); break;
    case FRV_AMOMINUW: LiftAmo(rvi, llvm::AtomicRMWInst::UMin, Facet::I32); break;
    case FRV_AMOMINUD: LiftAmo(rvi, llvm::AtomicRMWInst::UMin, Facet::I64); break;

    case FRV_FLW: LiftLoadFp(rvi, Facet::F32); break;
    case FRV_FSW: LiftStoreFp(rvi, Facet::F32); break;
    case FRV_FCVTSL: LiftFcvtIToF(rvi, Facet::F32, Facet::I64, llvm::Instruction::SIToFP); break;
    case FRV_FCVTSW: LiftFcvtIToF(rvi, Facet::F32, Facet::I32, llvm::Instruction::SIToFP); break;
    case FRV_FCVTSLU: LiftFcvtIToF(rvi, Facet::F32, Facet::I64, llvm::Instruction::UIToFP); break;
    case FRV_FCVTSWU: LiftFcvtIToF(rvi, Facet::F32, Facet::I32, llvm::Instruction::UIToFP); break;
    case FRV_FCVTLS: LiftFcvtFToI(rvi, Facet::I64, Facet::F32, llvm::Instruction::FPToSI); break;
    case FRV_FCVTWS: LiftFcvtFToI(rvi, Facet::I32, Facet::F32, llvm::Instruction::FPToSI); break;
    case FRV_FCVTLUS: LiftFcvtFToI(rvi, Facet::I64, Facet::F32, llvm::Instruction::FPToUI); break;
    case FRV_FCVTWUS: LiftFcvtFToI(rvi, Facet::I32, Facet::F32, llvm::Instruction::FPToUI); break;
    case FRV_FMVXW: StoreGp(rvi->rd, irb.CreateBitCast(LoadFp(rvi->rs1, Facet::F32), irb.getInt32Ty())); break;
    case FRV_FMVWX: StoreFp(rvi->rd, irb.CreateBitCast(LoadGp(rvi->rs1, Facet::I32), irb.getFloatTy())); break;
    case FRV_FADDS: LiftFpArith(rvi, llvm::Instruction::FAdd, Facet::F32); break;
    case FRV_FSUBS: LiftFpArith(rvi, llvm::Instruction::FSub, Facet::F32); break;
    case FRV_FMULS: LiftFpArith(rvi, llvm::Instruction::FMul, Facet::F32); break;
    case FRV_FDIVS: LiftFpArith(rvi, llvm::Instruction::FDiv, Facet::F32); break;
    // TODO: use llvm::Instrinsic::minimum/maximum
    // These are rarely supported by back-ends, though, and therefore prevent
    // lowering to hardware instructions. See also FMIND/FMAXD
    case FRV_FMINS: LiftFminmax(rvi, llvm::Intrinsic::minnum, Facet::F32); break;
    case FRV_FMAXS: LiftFminmax(rvi, llvm::Intrinsic::maxnum, Facet::F32); break;
    case FRV_FMADDS: LiftFmadd(rvi, /*sub=*/false, /*negprod=*/false, Facet::F32); break;
    case FRV_FMSUBS: LiftFmadd(rvi, /*sub=*/true, /*negprod=*/false, Facet::F32); break;
    case FRV_FNMSUBS: LiftFmadd(rvi, /*sub=*/false, /*negprod=*/true, Facet::F32); break;
    case FRV_FNMADDS: LiftFmadd(rvi, /*sub=*/true, /*negprod=*/true, Facet::F32); break;
    case FRV_FSQRTS: StoreFp(rvi->rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, LoadFp(rvi->rs1, Facet::F32))); break;
    case FRV_FSGNJS: LiftFsgn(rvi, Facet::F32, /*keep=*/false, /*zero=*/true); break;
    case FRV_FSGNJNS: LiftFsgn(rvi, Facet::F32, /*keep=*/false, /*zero=*/false); break;
    case FRV_FSGNJXS: LiftFsgn(rvi, Facet::F32, /*keep=*/true, /*zero=*/false); break;
    case FRV_FEQS: LiftFcmp(rvi, llvm::CmpInst::FCMP_OEQ, Facet::F32); break;
    case FRV_FLTS: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLT, Facet::F32); break;
    case FRV_FLES: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLE, Facet::F32); break;
    case FRV_FCLASSS: LiftFclass(rvi, Facet::I32); break;

    case FRV_FLD: LiftLoadFp(rvi, Facet::F64); break;
    case FRV_FSD: LiftStoreFp(rvi, Facet::F64); break;
    case FRV_FCVTDL: LiftFcvtIToF(rvi, Facet::F64, Facet::I64, llvm::Instruction::SIToFP); break;
    case FRV_FCVTDW: LiftFcvtIToF(rvi, Facet::F64, Facet::I32, llvm::Instruction::SIToFP); break;
    case FRV_FCVTDLU: LiftFcvtIToF(rvi, Facet::F64, Facet::I64, llvm::Instruction::UIToFP); break;
    case FRV_FCVTDWU: LiftFcvtIToF(rvi, Facet::F64, Facet::I32, llvm::Instruction::UIToFP); break;
    case FRV_FCVTLD: LiftFcvtFToI(rvi, Facet::I64, Facet::F64, llvm::Instruction::FPToSI); break;
    case FRV_FCVTWD: LiftFcvtFToI(rvi, Facet::I32, Facet::F64, llvm::Instruction::FPToSI); break;
    case FRV_FCVTLUD: LiftFcvtFToI(rvi, Facet::I64, Facet::F64, llvm::Instruction::FPToUI); break;
    case FRV_FCVTWUD: LiftFcvtFToI(rvi, Facet::I32, Facet::F64, llvm::Instruction::FPToUI); break;
    case FRV_FMVXD: StoreGp(rvi->rd, irb.CreateBitCast(LoadFp(rvi->rs1, Facet::F64), irb.getInt64Ty())); break;
    case FRV_FMVDX: StoreFp(rvi->rd, irb.CreateBitCast(LoadGp(rvi->rs1, Facet::I64), irb.getDoubleTy())); break;
    case FRV_FADDD: LiftFpArith(rvi, llvm::Instruction::FAdd, Facet::F64); break;
    case FRV_FSUBD: LiftFpArith(rvi, llvm::Instruction::FSub, Facet::F64); break;
    case FRV_FMULD: LiftFpArith(rvi, llvm::Instruction::FMul, Facet::F64); break;
    case FRV_FDIVD: LiftFpArith(rvi, llvm::Instruction::FDiv, Facet::F64); break;
    // TODO: use llvm::Instrinsic::minimum/maximum
    // See comment for FMINS/FMAXS
    case FRV_FMIND: LiftFminmax(rvi, llvm::Intrinsic::minnum, Facet::F64); break;
    case FRV_FMAXD: LiftFminmax(rvi, llvm::Intrinsic::maxnum, Facet::F64); break;
    case FRV_FMADDD: LiftFmadd(rvi, /*sub=*/false, /*negprod=*/false, Facet::F64); break;
    case FRV_FMSUBD: LiftFmadd(rvi, /*sub=*/true, /*negprod=*/false, Facet::F64); break;
    case FRV_FNMSUBD: LiftFmadd(rvi, /*sub=*/false, /*negprod=*/true, Facet::F64); break;
    case FRV_FNMADDD: LiftFmadd(rvi, /*sub=*/true, /*negprod=*/true, Facet::F64); break;
    case FRV_FSQRTD: StoreFp(rvi->rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, LoadFp(rvi->rs1, Facet::F64))); break;
    case FRV_FSGNJD: LiftFsgn(rvi, Facet::F64, /*keep=*/false, /*zero=*/true); break;
    case FRV_FSGNJND: LiftFsgn(rvi, Facet::F64, /*keep=*/false, /*zero=*/false); break;
    case FRV_FSGNJXD: LiftFsgn(rvi, Facet::F64, /*keep=*/true, /*zero=*/false); break;
    case FRV_FEQD: LiftFcmp(rvi, llvm::CmpInst::FCMP_OEQ, Facet::F64); break;
    case FRV_FLTD: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLT, Facet::F64); break;
    case FRV_FLED: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLE, Facet::F64); break;
    case FRV_FCVTSD: StoreFp(rvi->rd, irb.CreateFPTrunc(LoadFp(rvi->rs1, Facet::F64), irb.getFloatTy())); break;
    case FRV_FCVTDS: StoreFp(rvi->rd, irb.CreateFPExt(LoadFp(rvi->rs1, Facet::F32), irb.getDoubleTy())); break;
    case FRV_FCLASSD: LiftFclass(rvi, Facet::I64); break;
    }

    SetIP(inst.end());
    return true;
}

} // namespace rellume::rv64
