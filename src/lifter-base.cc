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

#include "lifter-base.h"

#include "arch.h"
#include "basicblock.h"
#include "config.h"
#include "function-info.h"
#include "instr.h"

#include <llvm/IR/Instruction.h>
#include <llvm/Transforms/Utils/Cloning.h>

namespace rellume {

llvm::Value* LifterBase::AddrConst(uint64_t addr) {
    if (addr == 0)
        return llvm::ConstantPointerNull::get(irb.getPtrTy());

    if (cfg.global_base_value) {
        auto offset = irb.getInt64(addr - cfg.global_base_addr);
        return irb.CreateGEP(irb.getInt8Ty(), cfg.global_base_value, offset);
    }

    return irb.CreateIntToPtr(irb.getInt64(addr), irb.getPtrTy());
}

void LifterBase::CallExternalFunction(llvm::Function* fn) {
    CallConv cconv = CallConv::FromFunction(fn, cfg.arch);
    llvm::CallInst* call = cconv.Call(fn, ablock.GetInsertBlock(), fi);
    assert(call && "failed to create call for external function");
    regfile = ablock.GetInsertBlock()->GetRegFile();

    // Directly inline alwaysinline functions
    if (fn->hasFnAttribute(llvm::Attribute::AlwaysInline)) {
        llvm::InlineFunctionInfo ifi;
        llvm::InlineFunction(*call, ifi);
    }
}

} // namespace rellume
