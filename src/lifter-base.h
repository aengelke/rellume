/**
 * This file is part of Rellume.
 *
 * (c) 2016-2020, Alexis Engelke <alexis.engelke@googlemail.com>
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

#ifndef RELLUME_LIFTER_BASE_H
#define RELLUME_LIFTER_BASE_H

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

/**
 * \brief Architecture-independent part of lifter.
 **/
class LifterBase {
protected:
    ArchBasicBlock& ablock;
    RegFile* regfile;
    llvm::IRBuilder<> irb;
    FunctionInfo& fi;
    const LLConfig& cfg;

    LifterBase(FunctionInfo& fi, const LLConfig& cfg, ArchBasicBlock& ab)
            : ablock(ab), regfile(ab.GetRegFile()),
              irb(regfile->GetInsertBlock()), fi(fi), cfg(cfg) {
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

    llvm::Value* GetReg(ArchReg reg, Facet facet) {
        return regfile->GetReg(reg, facet);
    }
    void SetReg(ArchReg reg, llvm::Value* value) {
        regfile->Set(reg, value);
    }
    llvm::Value* GetFlag(ArchReg reg) {
        if (reg == ArchReg::PF) {
            llvm::Value* res = GetReg(reg, Facet::I8);
            res = irb.CreateUnaryIntrinsic(llvm::Intrinsic::ctpop, res);
            return irb.CreateNot(irb.CreateTrunc(res, irb.getInt1Ty()));
        }
        return GetReg(reg, Facet::I1);
    }
    void SetFlagUndef(std::initializer_list<ArchReg> regs) {
        llvm::Value* undef = llvm::UndefValue::get(irb.getInt1Ty());
        for (const auto reg : regs) {
            // TODO: actually use freeze.
            if (reg == ArchReg::PF)
                SetReg(reg, llvm::UndefValue::get(irb.getInt8Ty()));
            else
                SetReg(reg, undef);
        }
    }
    void SetIP(uint64_t inst_addr, bool nofold = false) {
        if (nofold)
            regfile->SetPC(irb.getInt64(inst_addr));
        else
            regfile->SetPC(inst_addr);
    }
    void SetIP(llvm::Value* addr) {
        regfile->SetPC(addr);
    }
    void SetIPCond(llvm::Value* cond, uint64_t addr1, uint64_t addr2) {
        regfile->SetPCCond(cond, addr1, addr2);
    }
    void SetIPCallret(uint64_t retaddr) {
        regfile->SetPCCallret(AddrIPRel(), retaddr);
    }

    void SetInsertBlock(llvm::BasicBlock* block) {
        ablock.GetRegFile()->SetInsertPoint(block);
        irb.SetInsertPoint(block);
    }

    llvm::Value* AddrIPRel(uint64_t off = 0) {
        return regfile->GetPCValue(fi.pc_base_value, fi.pc_base_addr, off);
    }
    llvm::Value* AddrConst(uint64_t addr);

    void CallExternalFunction(llvm::Function* fn);

    void ForceReturn() {
        cfg.callconv.Return(&ablock, fi);
    }
};

} // namespace rellume

#endif
