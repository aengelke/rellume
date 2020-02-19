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
    LL_RT_GP8High, // AH = 4, CH = 5, DH = 6, BH = 7; others are invalid.
    LL_RT_GP,
    LL_RT_IP,
    LL_RT_X87,
    LL_RT_MMX,
    LL_RT_XMM,
    LL_RT_YMM,
    LL_RT_ZMM,
    LL_RT_SEG,
    LL_RT_BND,
    LL_RT_EFLAGS,
    LL_RT_Max
};

// Names for register indexes. Warning: indexes for different types overlap!
enum {
    LL_RI_None = 100, // assume no register type has more than 100 regs

    LL_RI_A = 0, LL_RI_C, LL_RI_D, LL_RI_B, LL_RI_SP, LL_RI_BP, LL_RI_SI, LL_RI_DI,
    // for LL_RT_GP8Leg
    LL_RI_AH = 4, LL_RI_CH, LL_RI_DH, LL_RI_BH,

    LL_RI_ES = 0, LL_RI_CS, LL_RI_SS, LL_RI_DS, LL_RI_FS, LL_RI_GS,

    LL_RI_GPMax = 16,
    LL_RI_XMMMax = 16,

};

typedef enum LLInstrType LLInstrType;

struct LLReg {
    uint16_t rt;
    uint16_t ri;
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
    unsigned addrsize;
    unsigned size;
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
#endif
};

typedef struct LLInstr LLInstr;

#if defined(__cplusplus) && defined(RELLUME_ENABLE_CPP_HEADER)
namespace rellume {
class Instr : public LLInstr {
public:
    using Type = LLInstrType;
    class Op {
        const Instr& inst;
        unsigned idx;
    public:
        constexpr Op(const Instr& inst, unsigned idx) : inst(inst), idx(idx) {}
        const LLInstrOp& llop() const { return inst.ops[idx]; }
        explicit operator bool() const {
            return idx < 4 && llop().type != LL_OP_NONE;
        }
        unsigned size() const { return llop().size; }
        unsigned bits() const { return size() * 8; }

        bool is_reg() const { return llop().type == LL_OP_REG; }
        const LLReg& reg() const { assert(is_reg()); return llop().reg; }

        bool is_imm() const { return llop().type == LL_OP_IMM; }
        int64_t imm() const { assert(is_imm()); return llop().val; }

        bool is_mem() const { return llop().type == LL_OP_MEM; }
        const LLReg& base() const { assert(is_mem()); return llop().reg; }
        const LLReg& index() const { assert(is_mem()); return llop().ireg; }
        unsigned scale() const { assert(is_mem()); return llop().scale; }
        int64_t off() const { assert(is_mem()); return llop().val; }
        unsigned seg() const { assert(is_mem()); return llop().seg; }
        unsigned addrsz() const { assert(is_mem()); return inst.address_size; }
    };

    Instr() : LLInstr(LLInstr::Invalid(0)) {}
    Instr(const LLInstr& lli) : LLInstr(lli) {}

    unsigned len() const { return LLInstr::len; }
    unsigned start() const { return LLInstr::addr; }
    unsigned end() const { return start() + len(); }
    Type type() const { return LLInstr::type; }
    unsigned opsz() const { return LLInstr::operand_size; }
    unsigned addrsz() const { return LLInstr::address_size; }
    const Op op(unsigned idx) const { return Op{*this, idx}; }

    bool BreaksAlways() const;
    bool BreaksConditionally() const;
    bool HasAbsJumpTarget() const;
    static Instr Decode(uint8_t* buf, size_t buf_size, uintptr_t addr);
};
}
#endif

#ifdef __cplusplus
}
#endif

#endif
