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

class Instr : public FdInstr {
public:
    using Type = LLInstrType;
    class Op {
        const Instr* inst;
        unsigned idx;
    public:
        constexpr Op(const Instr* inst, unsigned idx) : inst(inst), idx(idx) {}
        explicit operator bool() const {
            return idx < 4 && FD_OP_TYPE(inst, idx) != FD_OT_NONE;
        }
        unsigned size() const { return FD_OP_SIZE(inst, idx); }
        unsigned bits() const { return size() * 8; }

        bool is_reg() const { return FD_OP_TYPE(inst, idx) == FD_OT_REG; }
        const LLReg reg() const {
            assert(is_reg());
            return MapFdReg(FD_OP_REG(inst, idx), FD_OP_REG_TYPE(inst, idx));
        }

        bool is_imm() const { return FD_OP_TYPE(inst, idx) == FD_OT_IMM; }
        int64_t imm() const { assert(is_imm()); return FD_OP_IMM(inst, idx); }

        bool is_mem() const { return FD_OP_TYPE(inst, idx) == FD_OT_MEM; }
        const LLReg base() const {
            assert(is_mem());
            return MapFdReg(FD_OP_BASE(inst, idx), FD_RT_GPL);
        }
        const LLReg index() const {
            assert(is_mem());
            return MapFdReg(FD_OP_INDEX(inst, idx), FD_RT_GPL);
        }
        unsigned scale() const {
            assert(is_mem());
            if (FD_OP_INDEX(inst, idx) != FD_REG_NONE)
                return 1 << FD_OP_SCALE(inst, idx);
            return 0;
        }
        int64_t off() const { assert(is_mem()); return FD_OP_DISP(inst, idx); }
        unsigned seg() const { assert(is_mem()); return FD_SEGMENT(inst); }
        unsigned addrsz() const { assert(is_mem()); return inst->addrsz(); }
    };

    unsigned len() const { return FD_SIZE(fdi()); }
    unsigned start() const { return FD_ADDRESS(fdi()); }
    unsigned end() const { return start() + len(); }
    Type type() const;
    unsigned addrsz() const { return FD_ADDRSIZE(fdi()); }
    unsigned opsz() const { return FD_OPSIZE(fdi()); }
    const Op op(unsigned idx) const { return Op{this, idx}; }

    bool BreaksAlways() const;
    bool BreaksConditionally() const;
    bool HasAbsJumpTarget() const;

private:
    const FdInstr* fdi() const {return static_cast<const FdInstr*>(this); }
    static LLReg MapFdReg(unsigned idx, unsigned type);
};

} // namespace

#endif
