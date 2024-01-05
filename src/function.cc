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

#include "function.h"

#include "arch.h"
#include "basicblock.h"
#include "callconv.h"
#include "config.h"
#include "function-info.h"
#include "instr.h"
#include "a64/lifter.h"
#include "x86-64/lifter.h"
#include "rv64/lifter.h"
#include "regfile.h"
#include <llvm/ADT/DepthFirstIterator.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/InstIterator.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Transforms/Utils/BasicBlockUtils.h>
#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_map>


namespace rellume {

int Function::AddInst(uint64_t block_addr, uint64_t addr, size_t bufsz,
                      const uint8_t* buf) {
    auto& instr = instrs.emplace_back(DecodedInstr{});
    int ret = instr.inst.DecodeFrom(cfg->arch, buf, bufsz, addr);
    if (ret < 0) { // invalid or unknown instruction
        instrs.erase(instrs.end() - 1);
        return ret;
    }
    instr_map[addr] = instrs.size() - 1;
    instr.new_block = addr == block_addr || instrs.size() == 1 || instrs[instrs.size() - 2].inst.end() != addr;
    return instr.inst.len();
}

class LiftHelper {
    Function* func;
    FunctionInfo fi;
    llvm::DenseMap<uint64_t, std::unique_ptr<ArchBasicBlock>> block_map;

    std::unique_ptr<ArchBasicBlock> exit_block;

    ArchBasicBlock& ResolveAddr(llvm::Value* addr);

public:
    LiftHelper(Function* func) : func(func) {}

    llvm::Function* Lift();
};

ArchBasicBlock& LiftHelper::ResolveAddr(llvm::Value* addr) {
    uint64_t addr_skew = 0;

    // Strip off the base_rip from the expression.
    // Trivial case: no offset has ever been added.
    if (addr == fi.pc_base_value) {
        addr_skew = fi.pc_base_addr;
        addr = llvm::ConstantInt::get(addr->getType(), 0);
    }
    // We can either have an instruction...
    if (auto binop = llvm::dyn_cast<llvm::BinaryOperator>(addr)) {
        if (binop->getOpcode() == llvm::Instruction::Add &&
            binop->getOperand(0) == fi.pc_base_value) {
            addr_skew = fi.pc_base_addr;
            addr = binop->getOperand(1);
        }
    }
    // ... or a constant expression for this.
    if (auto expr = llvm::dyn_cast<llvm::ConstantExpr>(addr)) {
        if (expr->getOpcode() == llvm::Instruction::Add &&
            expr->getOperand(0) == fi.pc_base_value) {
            addr_skew = fi.pc_base_addr;
            addr = expr->getOperand(1);
        }
    }

    if (auto const_addr = llvm::dyn_cast<llvm::ConstantInt>(addr)) {
        auto block_it = block_map.find(addr_skew + const_addr->getZExtValue());
        if (block_it != block_map.end())
            return *(block_it->second);
    }
    return *exit_block;
}

llvm::Function* LiftHelper::Lift() {
    LLConfig* cfg = func->cfg;
    llvm::LLVMContext& ctx = func->mod->getContext();

    if (func->instrs.size() == 0)
        return nullptr;

    using LiftFn = bool(const Instr&, FunctionInfo&, const LLConfig&, ArchBasicBlock&) noexcept;

    LiftFn* lift_fn;
    switch (cfg->arch) {
#ifdef RELLUME_WITH_X86_64
    case Arch::X86_64: lift_fn = x86_64::LiftInstruction; break;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
    case Arch::RV64: lift_fn = rv64::LiftInstruction; break;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
    case Arch::AArch64: lift_fn = aarch64::LiftInstruction; break;
#endif // RELLUME_WITH_AARCH64
    default:
        return nullptr;
    }

    auto fn = llvm::Function::Create(cfg->callconv.FnType(ctx, cfg->sptr_addrspace),
                                     llvm::GlobalValue::ExternalLinkage, "", func->mod);
    fn->setCallingConv(cfg->callconv.FnCallConv());

    // CPU struct pointer parameters has some extra properties.
    unsigned cpu_param_idx = cfg->callconv.CpuStructParamIdx();
    fn->addFnAttr(llvm::Attribute::NullPointerIsValid);
    fn->addParamAttr(cpu_param_idx, llvm::Attribute::NoAlias);
    fn->addParamAttr(cpu_param_idx, llvm::Attribute::NoCapture);
    auto align_attr = llvm::Attribute::get(ctx, llvm::Attribute::Alignment, 16);
    fn->addParamAttr(cpu_param_idx, align_attr);
    fn->addDereferenceableParamAttr(cpu_param_idx, 0x190);

    auto phi_mode =
        cfg->full_facets ? BasicBlock::Phis::ALL : BasicBlock::Phis::NATIVE;
    // Create entry basic block as first block in the function.
    auto entry_block = std::make_unique<ArchBasicBlock>(fn,
                                                        BasicBlock::Phis::NONE,
                                                        cfg->arch);

    fi.fn = fn;
    fi.entry_ip = func->instrs[0].inst.start();
    fi.sptr_raw = &fn->arg_begin()[cpu_param_idx];

    // Initialize the sptr pointers in the function info.
    cfg->callconv.InitSptrs(entry_block->GetInsertBlock(), fi);
    // And initially fill register file.
    cfg->callconv.UnpackParams(entry_block->GetInsertBlock(), fi);

    if (cfg->pc_base_value) {
        fi.pc_base_addr = cfg->pc_base_addr;
        fi.pc_base_value = cfg->pc_base_value;
    } else {
        fi.pc_base_addr = fi.entry_ip;
        if (!cfg->position_independent_code) {
            llvm::Type* i64 = llvm::Type::getInt64Ty(fi.fn->getContext());
            fi.pc_base_value = llvm::ConstantInt::get(i64, fi.pc_base_addr);
        } else {
            RegFile* entry_rf = entry_block->GetInsertBlock()->GetRegFile();
            fi.pc_base_value = entry_rf->GetReg(ArchReg::IP, Facet::I64);
        }
    }

    {
        ArchBasicBlock* cur_ab = nullptr;
        for (size_t i = 0; i < func->instrs.size(); i++) {
            const auto& decinst = func->instrs[i];
            if (decinst.new_block) {
                uint64_t block_addr = decinst.inst.start();
                auto& ab = block_map.getOrInsertDefault(block_addr);
                assert(!ab && "duplicate block address");
                ab = std::make_unique<ArchBasicBlock>(fi.fn, phi_mode, cfg->arch);
                cur_ab = ab.get();
            }

            bool success = lift_fn(decinst.inst, fi, *cfg, *cur_ab);
            if (!success) {
                if (i == 0) { // failure at first instruction, propagate error
                    fn->eraseFromParent();
                    return nullptr;
                }
                // Skip forward to next block.
                while (i < func->instrs.size() - 1 && !func->instrs[i + 1].new_block)
                    i += 1;
            }
        }
    }

    exit_block = std::make_unique<ArchBasicBlock>(fn, phi_mode, cfg->arch);

    // Exit block packs values together and optionally returns something.
    if (cfg->tail_function) {
        CallConv cconv = CallConv::FromFunction(cfg->tail_function, cfg->arch);
        // Force a tail call to the specified function.
        cconv.Call(cfg->tail_function, exit_block->GetInsertBlock(), fi, true);
    } else {
        cfg->callconv.Return(exit_block->GetInsertBlock(), fi);
    }

    entry_block->BranchTo(*block_map[fi.entry_ip]);

    for (auto it = block_map.begin(); it != block_map.end(); ++it) {
        RegFile* regfile = it->second->GetInsertBlock()->GetRegFile();
        if (regfile->GetInsertBlock()->getTerminator())
            continue;
        llvm::Value* next_rip = regfile->GetReg(ArchReg::IP, Facet::I64);
        if (auto select = llvm::dyn_cast<llvm::SelectInst>(next_rip)) {
            it->second->BranchTo(select->getCondition(),
                                 ResolveAddr(select->getTrueValue()),
                                 ResolveAddr(select->getFalseValue()));
        } else {
            it->second->BranchTo(ResolveAddr(next_rip));
        }
    }

    cfg->callconv.OptimizePacks(fi, entry_block->GetInsertBlock());

    // Walk over blocks as long as phi nodes could have been added. We stop when
    // alls phis are filled.
    // TODO: improve walk ordering and efficiency (e.g. by adding predecessors
    // to the set of remaining blocks when something changed)
    bool changed = true;
    while (changed) {
        changed = false;
        for (auto& item : block_map)
            changed |= item.second->FillPhis();
        changed |= exit_block->FillPhis();
    }

    // Remove calls to llvm.ssa_copy, which got inserted to avoid PHI nodes in
    // the register file.
    for (auto it = llvm::inst_begin(fn), e = llvm::inst_end(fn); it != e;) {
        llvm::Instruction* inst = &*it++;
        auto* intr = llvm::dyn_cast<llvm::CallInst>(inst);
        if (!intr || intr->getIntrinsicID() != llvm::Intrinsic::ssa_copy)
          continue;

        inst->replaceAllUsesWith(intr->getOperand(0));
        inst->eraseFromParent();
    }

    // Remove blocks without predecessors. This can happen if constants get
    // folded already during construction, e.g. xor eax,eax;test eax,eax;jz
    llvm::EliminateUnreachableBlocks(*fn);

    if (cfg->verify_ir && llvm::verifyFunction(*(fn), &llvm::errs())) {
        fn->eraseFromParent();
        return nullptr;
    }

    return fn;
}

llvm::Function* Function::Lift() {
    return LiftHelper(this).Lift();
}

}
