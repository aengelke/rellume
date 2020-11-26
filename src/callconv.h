/**
 * This file is part of Rellume.
 *
 * (c) 2019, Alexis Engelke <alexis.engelke@googlemail.com>
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

#ifndef RELLUME_CALLCONV_H
#define RELLUME_CALLCONV_H

#include "basicblock.h"
#include "regfile.h"

#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <cstddef>
#include <tuple>


namespace rellume {

class FunctionInfo;

class CallConv {
public:
    enum Value {
        INVALID, X86_64_SPTR, X86_64_HHVM, RV64_SPTR,
    };

    static CallConv FromFunction(llvm::Function* fn, Arch arch);

    llvm::FunctionType* FnType(llvm::LLVMContext& ctx,
                               unsigned sptr_addrspace) const;
    llvm::CallingConv::ID FnCallConv() const;
    unsigned CpuStructParamIdx() const;

    void InitSptrs(BasicBlock* bb, FunctionInfo& fi);

    // Pack values from regfile into the CPU struct. The return value for the
    // function is returned (or NULL for void).
    llvm::ReturnInst* Return(BasicBlock* bb, FunctionInfo& fi) const;
    // Unpack values from val (usually the function) into the register file. For
    // SPTR, val can also be the CPU struct pointer directly.
    void UnpackParams(BasicBlock* bb, FunctionInfo& fi) const;

    llvm::CallInst* Call(llvm::Function* fn, BasicBlock* bb, FunctionInfo& fi,
                         bool tail_call = false);

    void OptimizePacks(FunctionInfo& fi, BasicBlock* entry);

    CallConv() = default;
    constexpr CallConv(Value value) : value(value) {}
    operator Value() const { return value; }
    explicit operator bool() = delete;
private:
    Value value;
};

} // namespace

#endif
