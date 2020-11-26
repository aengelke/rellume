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
        SetReg(ArchReg::GP(reg), Facet::I64, irb.CreateSExt(v, irb.getInt64Ty()));
        if (v->getType()->getIntegerBitWidth() == 32)
            SetRegFacet(ArchReg::GP(reg), Facet::I32, v);
    }
    void StoreFp(unsigned reg, llvm::Value* v) {
        llvm::Type* ivec_ty = Facet{Facet::I64}.Type(irb.getContext());
        unsigned ivec_sz = ivec_ty->getIntegerBitWidth();

        // Construct the requires vector type of the vector register.
        llvm::Type* element_ty = v->getType();
        unsigned full_num = ivec_sz / element_ty->getPrimitiveSizeInBits();
        llvm::VectorType* full_ty = llvm::VectorType::get(element_ty, full_num);
        llvm::Value* full = llvm::Constant::getNullValue(full_ty);
        full = irb.CreateInsertElement(full, v, 0ul);

        SetReg(ArchReg::VEC(reg), Facet::I64, irb.CreateBitCast(full, ivec_ty));
        SetRegFacet(ArchReg::VEC(reg), Facet::FromType(element_ty), v);
    }
    llvm::Value* Addr(const FrvInst* rvi, llvm::PointerType* ptr_ty) {
        llvm::Value* base = LoadGp(rvi->rs1, Facet::PTR);
        if (llvm::isa<llvm::Constant>(base)) {
            llvm::Value* base_int = LoadGp(rvi->rs1, Facet::I64);
            auto* addr = llvm::cast<llvm::ConstantInt>(base_int);
            base = AddrConst(addr->getZExtValue() + rvi->imm, ptr_ty);
        } else if (rvi->imm) {
            base = irb.CreatePointerCast(base, irb.getInt8PtrTy());
            base = irb.CreateGEP(base, irb.getInt64(rvi->imm));
        }
        return irb.CreatePointerCast(base, ptr_ty);
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
        llvm::Value* ld = irb.CreateLoad(ty, Addr(rvi, ty->getPointerTo()));
        StoreGp(rvi->rd, irb.CreateCast(ext, ld, irb.getInt64Ty()));
    }
    void LiftLoadFp(const FrvInst* rvi, Facet f) {
        llvm::Type* ty = f.Type(irb.getContext());
        StoreFp(rvi->rd, irb.CreateLoad(ty, Addr(rvi, ty->getPointerTo())));
    }
    void LiftStore(const FrvInst* rvi, Facet f) {
        llvm::Type* ty = f.Type(irb.getContext());
        irb.CreateStore(LoadGp(rvi->rs2, f), Addr(rvi, ty->getPointerTo()));
    }
    void LiftStoreFp(const FrvInst* rvi, Facet f) {
        llvm::Type* ty = f.Type(irb.getContext());
        irb.CreateStore(LoadFp(rvi->rs2, f), Addr(rvi, ty->getPointerTo()));
    }
    void LiftBranch(const Instr& inst, llvm::CmpInst::Predicate pred) {
        const FrvInst* rvi = inst;
        auto cond = irb.CreateICmp(pred, LoadGp(rvi->rs1), LoadGp(rvi->rs2));
        SetReg(ArchReg::IP, Facet::I64, irb.CreateSelect(cond,
            irb.CreateAdd(GetReg(ArchReg::IP, Facet::I64), irb.getInt64(rvi->imm)),
            irb.CreateAdd(GetReg(ArchReg::IP, Facet::I64), irb.getInt64(inst.len()))
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
        StoreGp(rvi->rd, irb.CreateAtomicRMW(op, ptr, val, ordering));
    }

    void LiftFpArith(const FrvInst* rvi, llvm::Instruction::BinaryOps op,
                     Facet f) {
        assert(rvi->misc == 7 && "only rm=DYN supported");
        auto res = irb.CreateBinOp(op, LoadFp(rvi->rs1, f), LoadFp(rvi->rs2, f));
        StoreFp(rvi->rd, res);
    }
    void LiftFcvtIToF(const FrvInst* rvi, Facet df, Facet sf,
                  llvm::Instruction::CastOps cast) {
        assert(rvi->misc == 7 && "only rm=DYN supported");
        llvm::Type* tgt_ty = df.Type(irb.getContext());
        StoreFp(rvi->rd, irb.CreateCast(cast, LoadGp(rvi->rs1, sf), tgt_ty));
    }
    void LiftFcvtFToI(const FrvInst* rvi, Facet df, Facet sf,
                  llvm::Instruction::CastOps cast) {
        assert((rvi->misc == 1 || rvi->misc == 7) && "only rm=DYN/RTZ supported");
        llvm::Type* tgt_ty = df.Type(irb.getContext());
        StoreGp(rvi->rd, irb.CreateCast(cast, LoadFp(rvi->rs1, sf), tgt_ty));
    }
    void LiftFcvtFToF(const FrvInst* rvi, Facet df, Facet sf,
                  llvm::Instruction::CastOps cast) {
        assert(rvi->misc == 7 && "only rm=DYN supported");
        llvm::Type* tgt_ty = df.Type(irb.getContext());
        StoreFp(rvi->rd, irb.CreateCast(cast, LoadFp(rvi->rs1, sf), tgt_ty));
    }
    void LiftFcmp(const FrvInst* rvi, llvm::CmpInst::Predicate pred, Facet f) {
        auto res = irb.CreateFCmp(pred, LoadFp(rvi->rs1, f), LoadFp(rvi->rs1, f));
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
        // auto intrinsic = llvm::Intrinsic::getDeclaration(module, id, {ty});
    }
    void LiftFminmax(const FrvInst* rvi, llvm::Intrinsic::ID id, Facet f) {
        auto res = irb.CreateBinaryIntrinsic(id, LoadFp(rvi->rs1, f), LoadFp(rvi->rs2, f));
        StoreFp(rvi->rd, res);
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
        SetIP(inst.start(), /*nofold=*/true);
        return false;

    case FRV_JAL:
    case FRV_JALR: {
        llvm::Value* pc = GetReg(ArchReg::IP, Facet::I64);
        auto base = rvi->rs1 != FRV_REG_INV ? LoadGp(rvi->rs1, Facet::I64) : pc;
        SetReg(ArchReg::IP, Facet::I64, irb.CreateAdd(base, irb.getInt64(rvi->imm)));
        if (rvi->rd) {
            llvm::Value* ret_addr = irb.CreateAdd(pc, irb.getInt64(inst.len()));
            StoreGp(rvi->rd, ret_addr);
            // TODO: implement correct RAS for call and return semantics.
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
            if (cfg.syscall_implementation)
                CallExternalFunction(cfg.syscall_implementation);
        } else {
            SetIP(inst.start(), /*nofold=*/true);
            return false;
        }
        break;

    case FRV_LUI: StoreGp(rvi->rd, irb.getInt64(rvi->imm)); break;
    case FRV_AUIPC: StoreGp(rvi->rd, irb.CreateAdd(GetReg(ArchReg::IP, Facet::I64), irb.getInt64(rvi->imm))); break;

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
    case FRV_SLLI: LiftBinOpI(rvi, llvm::Instruction::Shl, Facet::I64); break;
    case FRV_SLL: LiftBinOpR(rvi, llvm::Instruction::Shl, Facet::I64); break;
    case FRV_SLLIW: LiftBinOpI(rvi, llvm::Instruction::Shl, Facet::I32); break;
    case FRV_SLLW: LiftBinOpR(rvi, llvm::Instruction::Shl, Facet::I32); break;
    case FRV_SRLI: LiftBinOpI(rvi, llvm::Instruction::LShr, Facet::I64); break;
    case FRV_SRL: LiftBinOpR(rvi, llvm::Instruction::LShr, Facet::I64); break;
    case FRV_SRLIW: LiftBinOpI(rvi, llvm::Instruction::LShr, Facet::I32); break;
    case FRV_SRLW: LiftBinOpR(rvi, llvm::Instruction::LShr, Facet::I32); break;
    case FRV_SRAI: LiftBinOpI(rvi, llvm::Instruction::AShr, Facet::I64); break;
    case FRV_SRA: LiftBinOpR(rvi, llvm::Instruction::AShr, Facet::I64); break;
    case FRV_SRAIW: LiftBinOpI(rvi, llvm::Instruction::AShr, Facet::I32); break;
    case FRV_SRAW: LiftBinOpR(rvi, llvm::Instruction::AShr, Facet::I32); break;
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

    case FRV_CSRRW: break; // TODO: CSR instructions
    case FRV_CSRRS: break; // TODO: CSR instructions
    case FRV_CSRRC: break; // TODO: CSR instructions
    case FRV_CSRRWI: break; // TODO: CSR instructions
    case FRV_CSRRSI: break; // TODO: CSR instructions
    case FRV_CSRRCI: break; // TODO: CSR instructions

    // TODO: implement atomic semantics for LR/SC.
    case FRV_LRW: StoreGp(rvi->rd, irb.CreateLoad(irb.getInt32Ty(), irb.CreatePointerCast(LoadGp(rvi->rs1, Facet::PTR), irb.getInt32Ty()->getPointerTo()))); break;
    case FRV_SCW: StoreGp(rvi->rd, irb.getInt32(0)); irb.CreateStore(LoadGp(rvi->rs2, Facet::I32), irb.CreatePointerCast(LoadGp(rvi->rs1, Facet::PTR), irb.getInt32Ty()->getPointerTo())); break;
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
    case FRV_FMINS: LiftFminmax(rvi, llvm::Intrinsic::minimum, Facet::F32); break;
    case FRV_FMAXS: LiftFminmax(rvi, llvm::Intrinsic::maximum, Facet::F32); break;
    case FRV_FSQRTS: StoreFp(rvi->rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, LoadFp(rvi->rs1, Facet::F32))); break;
    case FRV_FSGNJS: LiftFsgn(rvi, Facet::F32, /*keep=*/false, /*zero=*/true); break;
    case FRV_FSGNJNS: LiftFsgn(rvi, Facet::F32, /*keep=*/false, /*zero=*/false); break;
    case FRV_FSGNJXS: LiftFsgn(rvi, Facet::F32, /*keep=*/true, /*zero=*/false); break;
    case FRV_FEQS: LiftFcmp(rvi, llvm::CmpInst::FCMP_OEQ, Facet::F32); break;
    case FRV_FLTS: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLT, Facet::F32); break;
    case FRV_FLES: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLE, Facet::F32); break;

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
    case FRV_FMIND: LiftFminmax(rvi, llvm::Intrinsic::minimum, Facet::F64); break;
    case FRV_FMAXD: LiftFminmax(rvi, llvm::Intrinsic::maximum, Facet::F64); break;
    case FRV_FSQRTD: StoreFp(rvi->rd, irb.CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, LoadFp(rvi->rs1, Facet::F64))); break;
    case FRV_FSGNJD: LiftFsgn(rvi, Facet::F64, /*keep=*/false, /*zero=*/true); break;
    case FRV_FSGNJND: LiftFsgn(rvi, Facet::F64, /*keep=*/false, /*zero=*/false); break;
    case FRV_FSGNJXD: LiftFsgn(rvi, Facet::F64, /*keep=*/true, /*zero=*/false); break;
    case FRV_FEQD: LiftFcmp(rvi, llvm::CmpInst::FCMP_OEQ, Facet::F64); break;
    case FRV_FLTD: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLT, Facet::F64); break;
    case FRV_FLED: LiftFcmp(rvi, llvm::CmpInst::FCMP_OLE, Facet::F64); break;
    case FRV_FCVTSD: StoreFp(rvi->rd, irb.CreateFPTrunc(LoadFp(rvi->rs1, Facet::F64), irb.getFloatTy())); break;
    case FRV_FCVTDS: StoreFp(rvi->rd, irb.CreateFPExt(LoadFp(rvi->rs1, Facet::F32), irb.getDoubleTy())); break;
    // TODO: F[N]MADD, F[N]MSUB, FCLASS
    }

    SetIP(inst.end());
    return true;
}

} // namespace rellume::rv64
