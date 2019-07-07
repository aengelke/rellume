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

#ifndef LL_STATE_H
#define LL_STATE_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm-c/Core.h>
#include <llvm-c/ExecutionEngine.h>

#include <llregfile-internal.h>

#ifdef __cplusplus
extern "C" {
#endif

struct LLConfig {
    size_t stackSize;

    /**
     * \brief Whether overflow intrinsics should be used.
     **/
    bool enableOverflowIntrinsics;
    /**
     * \brief Whether unsafe floating-point optimizations may be applied.
     * Corresponds to -ffast-math.
     **/
    bool enableFastMath;

    /// Whether to use pointer for 64-bit reg-reg compares instead of using
    /// ptrtoint.
    bool prefer_pointer_cmp;

    /**
     * \brief The global offset base
     **/
    uintptr_t globalOffsetBase;
    /**
     * \brief The global variable used to access constant memory regions. Points
     * to globalOffsetBase.
     **/
    LLVMValueRef globalBase;
};

typedef struct LLConfig LLConfig;

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

/**
 * \brief The LLVM state of the back-end.
 **/
class LLStateBase {
protected:
    LLStateBase(llvm::LLVMContext& ctx) : context(llvm::wrap(&ctx)), irb(ctx) {
        builder = llvm::wrap(&irb);
    }

public:
    LLStateBase(LLStateBase&& rhs);
    LLStateBase& operator=(LLStateBase&& rhs);

    LLStateBase(const LLStateBase&) = delete;
    LLStateBase& operator=(const LLStateBase&) = delete;

    LLConfig cfg;

    /// DEPRECATED LLVM context, use irb.getContext()
    LLVMContextRef context;
    /// DEPRECATED LLVM builder, use irb
    LLVMBuilderRef builder;

    /// Current register file
    RegFile* regfile;

    llvm::IRBuilder<> irb;


    llvm::Value* GetReg(LLReg reg, Facet::Value facet) {
        return regfile->GetReg(reg, facet);
    }
    void SetReg(LLReg reg, Facet::Value facet, llvm::Value* value, bool clear=true) {
        regfile->SetReg(reg, facet, value, clear); // clear all other facets
    }
    void SetRegFacet(LLReg reg, Facet::Value facet, llvm::Value* value) {
        regfile->SetReg(reg, facet, value, false);
    }
    llvm::Value* GetFlag(int flag) {
        return regfile->GetFlag(flag);
    }
    void SetFlag(int flag, llvm::Value* value) {
        regfile->SetFlag(flag, value);
    }

    // Operand handling implemented in lloperand.cc
private:
    llvm::Value* OpAddrConst(uint64_t addr);
public:
    llvm::Value* OpAddr(const LLInstrOp& op, llvm::Type* element_type);
    llvm::Value* OpLoad(const LLInstrOp& op, Facet::Value dataType, Alignment alignment = ALIGN_NONE);
    void OpStoreGp(const LLInstrOp& op, llvm::Value* value, Alignment alignment = ALIGN_NONE);
    void OpStoreVec(const LLInstrOp& op, llvm::Value* value, bool avx = false, Alignment alignment = ALIGN_IMP);

    void FlagCalcZ(llvm::Value* value) {
        auto zero = llvm::Constant::getNullValue(value->getType());
        SetFlag(RFLAG_ZF, irb.CreateICmpEQ(value, zero));
    }
    void FlagCalcS(llvm::Value* value) {
        auto zero = llvm::Constant::getNullValue(value->getType());
        SetFlag(RFLAG_SF, irb.CreateICmpSLT(value, zero));
    }
    void FlagCalcP(llvm::Value* value);
    void FlagCalcA(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    // void FlagCalcCAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    // void FlagCalcCSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    // void FlagCalcOAdd(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
    // void FlagCalcOSub(llvm::Value* res, llvm::Value* lhs, llvm::Value* rhs);
};

class LLState : public LLStateBase {
public:
    LLState(llvm::LLVMContext& ctx) : LLStateBase(ctx) {}

    void InstRet(LLInstr&);

    // llinstruction-sse.cc
    void LiftSseMovq(const LLInstr&, Facet::Value type);
};

// Legacy API.

#define OP_SI Facet::I
#define OP_SI32 Facet::I32
#define OP_SI64 Facet::I64
#define OP_VI8 Facet::VI8
#define OP_VI32 Facet::VI32
#define OP_VI64 Facet::VI64
#define OP_SF32 Facet::F32
#define OP_SF64 Facet::F64
#define OP_VF32 Facet::VF32
#define OP_V1F32 Facet::V1F32
#define OP_V2F32 Facet::V2F32
#define OP_V4F32 Facet::V4F32
#define OP_VF64 Facet::VF64
#define OP_V1F64 Facet::V1F64
#define OP_V2F64 Facet::V2F64
#define OperandDataType Facet::Value

enum {
    REG_DEFAULT, REG_ZERO_UPPER_AVX, REG_KEEP_UPPER,
};

#define ll_get_register(reg,facet,state) llvm::wrap((state)->GetReg(reg, facet))
#define ll_set_register(reg,facet,value,clear,state) (state)->SetReg(reg, facet, llvm::unwrap(value), clear)
#define ll_get_flag(reg,state) llvm::wrap((state)->GetFlag(reg))
#define ll_set_flag(reg,value,state) (state)->SetFlag(reg, llvm::unwrap(value))
#define ll_operand_load(facet,align,op,state) llvm::wrap((state)->OpLoad(*(op), facet, align))
#define ll_operand_store(facet,align,op,prh,val,state) do { \
            if ((prh) == REG_DEFAULT) \
                (state)->OpStoreGp(*(op), llvm::unwrap(val), align); \
            else \
                (state)->OpStoreVec(*(op), llvm::unwrap(val), prh == REG_ZERO_UPPER_AVX, align); \
        } while (0)


#ifdef __cplusplus
}
#endif

#endif
