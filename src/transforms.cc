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

#include "transforms.h"

#include "facet.h"
#include <llvm/ADT/SmallVector.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Transforms/InstCombine/InstCombine.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Cloning.h>
#include <cassert>
#include <cstdint>


namespace rellume {

namespace {

static llvm::Value* GepHelper(llvm::IRBuilder<>& irb, llvm::Value* base, llvm::ArrayRef<unsigned> idxs) {
    llvm::SmallVector<llvm::Value*, 4> consts;
    for (auto& idx : idxs)
        consts.push_back(irb.getInt32(idx));
    return irb.CreateGEP(base, consts);
}

}

void FastOpt(llvm::Function* llvm_fn) {
    // Run some optimization passes to remove most of the bloat
    llvm::legacy::FunctionPassManager pm(llvm_fn->getParent());
    pm.doInitialization();

    // Aggressive DCE to remove phi cycles, etc.
    pm.add(llvm::createAggressiveDCEPass());
    // Fold some common subexpressions with MemorySSA to remove obsolete stores
    pm.add(llvm::createEarlyCSEPass(true));
    // Combine instructions to simplify code, but avoid expensive transforms
    pm.add(llvm::createInstructionCombiningPass(false));

    pm.run(*llvm_fn);
    pm.doFinalization();
}

llvm::Function* WrapSysVAbi(llvm::Function* orig_fn, llvm::FunctionType* fn_ty,
                            std::size_t stack_size) {
    llvm::LLVMContext& ctx = orig_fn->getContext();
    llvm::Function* new_fn = llvm::Function::Create(fn_ty, llvm::GlobalValue::ExternalLinkage, "glob", orig_fn->getParent());
    llvm::BasicBlock* llvm_bb = llvm::BasicBlock::Create(ctx, "", new_fn);

    llvm::IRBuilder<> irb(llvm_bb);

    llvm::SmallVector<llvm::Type*, 4> cpu_types;
    cpu_types.push_back(irb.getInt64Ty()); // instruction pointer
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt64Ty(), 16));
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt1Ty(), 8));
    cpu_types.push_back(llvm::ArrayType::get(irb.getInt64Ty(), 2));
    cpu_types.push_back(llvm::ArrayType::get(irb.getIntNTy(LL_VECTOR_REGISTER_SIZE), 16));
    llvm::Type* cpu_type = llvm::StructType::get(irb.getContext(), cpu_types);

    llvm::Value* alloca = irb.CreateAlloca(cpu_type, int{0});

    // Set direction flag to zero
    irb.CreateStore(irb.getFalse(), rellume::GepHelper(irb, alloca, {0, 2, 6}));

    unsigned gp_regs[6] = { 7, 6, 2, 1, 8, 9 };
    unsigned gpRegOffset = 0;
    unsigned fpRegOffset = 0;
    for (auto arg = new_fn->arg_begin(); arg != new_fn->arg_end(); ++arg)
    {
        llvm::Type::TypeID type_kind = arg->getType()->getTypeID();

        if (type_kind == llvm::Type::TypeID::IntegerTyID)
        {
            irb.CreateStore(arg, rellume::GepHelper(irb, alloca, {0, 1, gp_regs[gpRegOffset]}));
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::PointerTyID)
        {
            llvm::Value* intval = irb.CreatePtrToInt(arg, irb.getInt64Ty());
            irb.CreateStore(intval, rellume::GepHelper(irb, alloca, {0, 1, gp_regs[gpRegOffset]}));
            gpRegOffset++;
        }
        else if (type_kind == llvm::Type::TypeID::FloatTyID || type_kind == llvm::Type::TypeID::DoubleTyID)
        {
            llvm::Type* int_type = irb.getIntNTy(arg->getType()->getPrimitiveSizeInBits());
            llvm::Type* vec_type = irb.getIntNTy(LL_VECTOR_REGISTER_SIZE);
            llvm::Value* intval = irb.CreateBitCast(arg, int_type);
            llvm::Value* ext = irb.CreateZExt(intval, vec_type);
            irb.CreateStore(ext, rellume::GepHelper(irb, alloca, {0, 4, fpRegOffset}));
            fpRegOffset++;
        }
        else
        {
            assert(false);
        }
    }

    llvm::Value* stack_sz_val = irb.getInt64(stack_size);
    llvm::AllocaInst* stack = irb.CreateAlloca(irb.getInt8Ty(), stack_sz_val);
    stack->setAlignment(16);
    llvm::Value* sp_ptr = irb.CreateGEP(stack, stack_sz_val);
    llvm::Value* sp = irb.CreatePtrToInt(sp_ptr, irb.getInt64Ty());
    irb.CreateStore(sp, rellume::GepHelper(irb, alloca, {0, 1, 4}));

    llvm::Value* call_arg = irb.CreatePointerCast(alloca, irb.getInt8PtrTy());
    llvm::CallInst* call = irb.CreateCall(orig_fn, {call_arg});

    llvm::Type* ret_type = new_fn->getReturnType();
    switch (ret_type->getTypeID())
    {
        llvm::Value* ret;

        case llvm::Type::TypeID::VoidTyID:
            irb.CreateRetVoid();
            break;
        case llvm::Type::TypeID::IntegerTyID:
            ret = irb.CreateLoad(rellume::GepHelper(irb, alloca, {0, 1, 0}));
            ret = irb.CreateTruncOrBitCast(ret, ret_type);
            irb.CreateRet(ret);
            break;
        case llvm::Type::TypeID::PointerTyID:
            ret = irb.CreateLoad(rellume::GepHelper(irb, alloca, {0, 1, 0}));
            ret = irb.CreateIntToPtr(ret, ret_type);
            irb.CreateRet(ret);
            break;
        case llvm::Type::TypeID::FloatTyID:
        case llvm::Type::TypeID::DoubleTyID:
            ret = irb.CreateLoad(rellume::GepHelper(irb, alloca, {0, 4, 0}));
            ret = irb.CreateTrunc(ret, irb.getIntNTy(ret_type->getPrimitiveSizeInBits()));
            ret = irb.CreateBitCast(ret, ret_type);
            irb.CreateRet(ret);
            break;
        default:
            assert(false);
            break;
    }

    llvm::InlineFunctionInfo ifi;
    llvm::InlineFunction(llvm::CallSite(call), ifi);

    if (llvm::verifyFunction(*new_fn, &llvm::errs()))
        return NULL;

    llvm::legacy::FunctionPassManager pm(new_fn->getParent());
    pm.doInitialization();

    // replace CPU struct with scalars
    pm.add(llvm::createSROAPass());
    // instrcombine will get rid of lots of bloat from the CPU struct
    pm.add(llvm::createInstructionCombiningPass(false));
    // Simplify CFG, removes some redundant function exists and empty blocks
    pm.add(llvm::createCFGSimplificationPass());
    // Aggressive DCE to remove phi cycles, etc.
    pm.add(llvm::createAggressiveDCEPass());

    pm.run(*new_fn);
    pm.doFinalization();

    return new_fn;
}

}
