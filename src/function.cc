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

class LiftHelper {
    Function* func;
    FunctionInfo fi;
    BasicBlock::Phis phi_mode;
    llvm::DenseMap<uint64_t, std::unique_ptr<ArchBasicBlock>> block_map;

    std::unique_ptr<ArchBasicBlock> exit_block;

    ArchBasicBlock& ResolveAddr(uint64_t addr);
    ArchBasicBlock& ResolveAddr(llvm::Value* addr);

public:
    LiftHelper(Function* func) : func(func) {}

    llvm::Function* Lift();
};

ArchBasicBlock& LiftHelper::ResolveAddr(uint64_t addr) {
    auto block_it = block_map.find(addr);
    if (block_it != block_map.end())
        return *(block_it->second);
    // We haven't created the block yet -- are we going to lift sth into it?
    auto instr_it = func->instr_map.find(addr);
    if (instr_it == func->instr_map.end() || !instr_it->second.decoded)
        return *exit_block;

    // Branches to instructions that we didn't identify as block start indicate
    // a mismatch between decoding and lifting. This can happen for indirect
    // jumps which become constants during lifting. For now, ignore this.
    // TODO: split block if not yet lifted, otw. duplicate?
    if (!func->instrs[instr_it->second.instr_idx].new_block)
        return *exit_block;
    // We will lift something for that address, so create the block.
    auto ab = std::make_unique<ArchBasicBlock>(fi.fn, phi_mode, func->cfg->arch,
                                               instr_it->second.preds);
    return *(block_map[addr] = std::move(ab));
}

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

    if (auto const_addr = llvm::dyn_cast<llvm::ConstantInt>(addr))
        return ResolveAddr(addr_skew + const_addr->getZExtValue());
    return *exit_block;
}

llvm::Function* LiftHelper::Lift() {
    LLConfig* cfg = func->cfg;
    llvm::LLVMContext& ctx = func->mod->getContext();

    if (func->instrs.size() == 0)
        return nullptr;

    uint64_t entry_ip = func->instrs[0].inst.start();
    func->instr_map[entry_ip].preds++;

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

    fi.fn = fn;
    fi.sptr_raw = &fn->arg_begin()[cpu_param_idx];

    phi_mode = cfg->full_facets ? BasicBlock::Phis::ALL : BasicBlock::Phis::NATIVE;
    // Create entry basic block as first block in the function.
    auto entry_block = std::make_unique<ArchBasicBlock>(fn,
                                                        BasicBlock::Phis::NONE,
                                                        cfg->arch, 0);
    // Initialize the sptr pointers in the function info.
    cfg->callconv.InitSptrs(entry_block->GetInsertBlock(), fi);
    // And initially fill register file.
    cfg->callconv.UnpackParams(entry_block->GetInsertBlock(), fi);
    entry_block->BranchTo(ResolveAddr(entry_ip));

    exit_block = std::make_unique<ArchBasicBlock>(fn, phi_mode, cfg->arch, SIZE_MAX);
    exit_block->GetInsertBlock()->InitRegFile(cfg->arch, phi_mode);

    if (cfg->pc_base_value) {
        fi.pc_base_addr = cfg->pc_base_addr;
        fi.pc_base_value = cfg->pc_base_value;
    } else {
        fi.pc_base_addr = entry_ip;
        if (!cfg->position_independent_code) {
            fi.pc_base_value = nullptr;
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
                cur_ab = &ResolveAddr(decinst.inst.start());
                assert(!cur_ab->GetInsertBlock()->GetRegFile());
                cur_ab->GetInsertBlock()->InitRegFile(cfg->arch, phi_mode);
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

            // Finish block by adding branches
            if (i >= func->instrs.size() - 1 || func->instrs[i + 1].new_block) {
                RegFile* regfile = cur_ab->GetInsertBlock()->GetRegFile();
                if (!regfile || regfile->GetInsertBlock()->getTerminator())
                    continue;
                if (decinst.inhibit_branch) {
                    cur_ab->BranchTo(*exit_block);
                    continue;
                }
                llvm::Value* next_rip = regfile->GetReg(ArchReg::IP, Facet::I64);
                if (auto select = llvm::dyn_cast<llvm::SelectInst>(next_rip)) {
                    cur_ab->BranchTo(select->getCondition(),
                                     ResolveAddr(select->getTrueValue()),
                                     ResolveAddr(select->getFalseValue()));
                } else {
                    cur_ab->BranchTo(ResolveAddr(next_rip));
                }
            }
        }
    }

    // Exit block packs values together and optionally returns something.
    if (cfg->tail_function) {
        CallConv cconv = CallConv::FromFunction(cfg->tail_function, cfg->arch);
        // Force a tail call to the specified function.
        cconv.Call(cfg->tail_function, exit_block->GetInsertBlock(), fi, true);
    } else {
        cfg->callconv.Return(exit_block->GetInsertBlock(), fi);
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
