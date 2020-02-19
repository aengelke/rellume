/**
 * This file is part of Rellume.
 *
 * (c) 2020, Alexis Engelke <alexis.engelke@googlemail.com>
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

#ifndef RELLUME_INSTR_H
#define RELLUME_INSTR_H

#include "rellume/instr.h"

#include <fadec.h>

#include <cstdbool>
#include <cstdint>


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
    Instr(const FdInstr& fdi);

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
};

} // namespace

#endif
