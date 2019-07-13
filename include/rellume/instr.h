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

#ifndef LL_INSTR_H
#define LL_INSTR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum LLInstrType {
    LL_INS_None = 0,
    LL_INS_Invalid = 1,
#define DEF_IT(opc,...) LL_INS_ ## opc,
#include "rellume/opcodes.inc"
#undef DEF_IT
    LL_INS_Max
};


enum {
    LL_RT_None = 0,
    LL_RT_GP8Leg,
    LL_RT_GP8,
    LL_RT_GP16,
    LL_RT_GP32,
    LL_RT_GP64,
    LL_RT_IP,
    LL_RT_X87,
    LL_RT_MMX,
    LL_RT_XMM,
    LL_RT_YMM,
    LL_RT_ZMM,
    LL_RT_SEG,
    LL_RT_Max
};

// Names for register indexes. Warning: indexes for different types overlap!
enum {
    LL_RI_None = 100, // assume no register type has more than 100 regs

    // for LL_RT_GP8Leg (1st 8 from x86, but can address 16 regs in 64bit mode)
    LL_RI_A = 0, LL_RI_C, LL_RI_D, LL_RI_B, LL_RI_SP, LL_RI_BP, LL_RI_SI, LL_RI_DI,
    LL_RI_AH = 4, LL_RI_CH, LL_RI_DH, LL_RI_BH,

    LL_RI_ES = 0, LL_RI_CS, LL_RI_SS, LL_RI_DS, LL_RI_FS, LL_RI_GS,

    LL_RI_GPMax = 16,
    LL_RI_XMMMax = 16,

};

typedef enum LLInstrType LLInstrType;

struct LLReg {
    uint16_t rt;
    uint16_t ri;

#if defined(__cplusplus) && defined(RELLUME_ENABLE_CPP_HEADER)
    LLReg() : rt(LL_RT_None), ri(LL_RI_None) {}
    LLReg(int rt, int ri) : rt(static_cast<uint16_t>(rt)),
            ri(static_cast<uint16_t>(ri)) {}
    static LLReg None() { return LLReg{LL_RT_None, LL_RI_None}; }
    static LLReg Gp(size_t size, uint16_t index, bool legacy = true) {
        switch (size) {
            case 1: return LLReg{legacy ? LL_RT_GP8Leg : LL_RT_GP8, index};
            case 2: return LLReg{LL_RT_GP16, index};
            case 4: return LLReg{LL_RT_GP32, index};
            case 8: return LLReg{LL_RT_GP64, index};
            default: return LLReg{LL_RT_None, LL_RI_None};
        }
    }
    size_t Size() const;
    const char* Name() const;
    bool IsGp() const {
        return rt == LL_RT_GP8Leg || rt == LL_RT_GP8 || rt == LL_RT_GP16 ||
               rt == LL_RT_GP32 || rt == LL_RT_GP64;
    }
    bool IsGpHigh() const {
        return rt == LL_RT_GP8Leg && ri >= 4 && ri < 8;
    }
    bool IsVec() const {
        return rt == LL_RT_XMM || rt == LL_RT_YMM;
    }
#endif
};

typedef struct LLReg LLReg;

enum {
    LL_OP_NONE = 0,
    LL_OP_REG,
    LL_OP_IMM,
    LL_OP_MEM,
};

struct LLInstrOp {
    uint64_t val;
    int type;
    LLReg reg;
    LLReg ireg;
    int scale;
    int seg;
    int size;

#if defined(__cplusplus) && defined(RELLUME_ENABLE_CPP_HEADER)
    static LLInstrOp Reg(const LLReg& reg) {
        int sz = static_cast<int>(reg.Size());
        return LLInstrOp{0, LL_OP_REG, reg, LLReg::None(), 0, LL_RI_None, sz};
    }
#endif
};

typedef struct LLInstrOp LLInstrOp;

struct LLInstr {
    LLInstrType type;
    int operand_count;
    int operand_size;
    int address_size;
    LLInstrOp ops[4];

    uintptr_t addr;
    int len;

#if defined(__cplusplus) && defined(RELLUME_ENABLE_CPP_HEADER)
    static LLInstr Invalid(uintptr_t addr) {
        return LLInstr{LL_INS_Invalid, 0, 0, 0, {}, addr, 1};
    }
    static LLInstr Decode(uint8_t* buf, size_t buf_size, uintptr_t addr);
#endif
};

typedef struct LLInstr LLInstr;

#ifdef __cplusplus
}
#endif

#endif
