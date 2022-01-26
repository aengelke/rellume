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

#ifdef RELLUME_WITH_X86_64
#include <fadec.h>
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
#include <frvdec.h>
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
#include <farmdec.h>
#endif // RELLUME_WITH_AARCH64

#include <cstdbool>
#include <cstdint>
#include <optional>

#include "arch.h"

namespace rellume {

class Instr {
    Arch arch;
    unsigned char instlen;
    uint64_t addr;
    union {
#ifdef RELLUME_WITH_X86_64
        FdInstr x86_64;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        FrvInst rv64;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        farmdec::Inst _a64;
#endif // RELLUME_WITH_AARCH64
    };

public:

#ifdef RELLUME_WITH_X86_64
    using Type = FdInstrType;
    struct Reg {
        uint16_t rt;
        uint16_t ri;
        Reg(unsigned rt, unsigned ri)
            : rt(static_cast<uint16_t>(rt)), ri(static_cast<uint16_t>(ri)) {}
        explicit operator bool() const { return ri != FD_REG_NONE; }
    };

    class Op {
        const FdInstr* fdi;
        unsigned idx;

    public:
        constexpr Op(const FdInstr* fdi, unsigned idx) : fdi(fdi), idx(idx) {}
        explicit operator bool() const {
            return idx < 4 && FD_OP_TYPE(fdi, idx) != FD_OT_NONE;
        }
        unsigned size() const { return FD_OP_SIZE(fdi, idx); }
        unsigned bits() const { return size() * 8; }

        bool is_reg() const { return FD_OP_TYPE(fdi, idx) == FD_OT_REG; }
        const Reg reg() const {
            assert(is_reg());
            return Reg(FD_OP_REG_TYPE(fdi, idx), FD_OP_REG(fdi, idx));
        }

        bool is_imm() const { return FD_OP_TYPE(fdi, idx) == FD_OT_IMM; }
        int64_t imm() const { assert(is_imm()); return FD_OP_IMM(fdi, idx); }

        bool is_pcrel() const { return FD_OP_TYPE(fdi, idx) == FD_OT_OFF; }
        int64_t pcrel() const { assert(is_pcrel()); return FD_OP_IMM(fdi, idx); }

        bool is_mem() const { return FD_OP_TYPE(fdi, idx) == FD_OT_MEM; }
        const Reg base() const {
            assert(is_mem());
            return Reg(FD_RT_GPL, FD_OP_BASE(fdi, idx));
        }
        const Reg index() const {
            assert(is_mem());
            return Reg(FD_RT_GPL, FD_OP_INDEX(fdi, idx));
        }
        unsigned scale() const {
            assert(is_mem());
            if (FD_OP_INDEX(fdi, idx) != FD_REG_NONE)
                return 1 << FD_OP_SCALE(fdi, idx);
            return 0;
        }
        int64_t off() const { assert(is_mem()); return FD_OP_DISP(fdi, idx); }
        unsigned seg() const { assert(is_mem()); return FD_SEGMENT(fdi); }
        unsigned addrsz() const { assert(is_mem()); return FD_ADDRSIZE(fdi); }
    };

    Type type() const { return FD_TYPE(&x86_64); }
    unsigned addrsz() const { return FD_ADDRSIZE(&x86_64); }
    unsigned opsz() const { return FD_OPSIZE(&x86_64); }
    const Op op(unsigned idx) const { return Op{&x86_64, idx}; }
    bool has_rep() const { return FD_HAS_REP(&x86_64); }
    bool has_repnz() const { return FD_HAS_REPNZ(&x86_64); }
    bool has_lock() const { return FD_HAS_LOCK(&x86_64); }
#endif // RELLUME_WITH_X86_64


    size_t len() const { return instlen; }
    uintptr_t start() const { return addr; }
    uintptr_t end() const { return start() + len(); }

#ifdef RELLUME_WITH_RV64
    operator const FrvInst*() const {
        assert(arch == Arch::RV64);
        return &rv64;
    }
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
    operator const farmdec::Inst*() const {
        assert(arch == Arch::AArch64);
        return &_a64;
    }
#endif // RELLUME_WITH_AARCH64

    enum class Kind {
        BRANCH,
        COND_BRANCH,
        CALL,
        UNKNOWN,
        OTHER
    };
    Kind Kind() {
        switch (arch) {
#ifdef RELLUME_WITH_X86_64
        case Arch::X86_64:
            switch (type()) {
            default:          return Kind::OTHER;
            case FDI_JO:      return Kind::COND_BRANCH;
            case FDI_JNO:     return Kind::COND_BRANCH;
            case FDI_JC:      return Kind::COND_BRANCH;
            case FDI_JNC:     return Kind::COND_BRANCH;
            case FDI_JZ:      return Kind::COND_BRANCH;
            case FDI_JNZ:     return Kind::COND_BRANCH;
            case FDI_JBE:     return Kind::COND_BRANCH;
            case FDI_JA:      return Kind::COND_BRANCH;
            case FDI_JS:      return Kind::COND_BRANCH;
            case FDI_JNS:     return Kind::COND_BRANCH;
            case FDI_JP:      return Kind::COND_BRANCH;
            case FDI_JNP:     return Kind::COND_BRANCH;
            case FDI_JL:      return Kind::COND_BRANCH;
            case FDI_JGE:     return Kind::COND_BRANCH;
            case FDI_JLE:     return Kind::COND_BRANCH;
            case FDI_JG:      return Kind::COND_BRANCH;
            case FDI_JCXZ:    return Kind::COND_BRANCH;
            case FDI_LOOP:    return Kind::COND_BRANCH;
            case FDI_LOOPZ:   return Kind::COND_BRANCH;
            case FDI_LOOPNZ:  return Kind::COND_BRANCH;
            case FDI_JMP:     return Kind::BRANCH;
            case FDI_CALL:    return Kind::CALL;
            case FDI_RET:     return Kind::UNKNOWN;
            case FDI_SYSCALL: return Kind::UNKNOWN;
            case FDI_INT:     return Kind::UNKNOWN;
            case FDI_INT3:    return Kind::UNKNOWN;
            case FDI_INTO:    return Kind::UNKNOWN;
            case FDI_UD0:     return Kind::UNKNOWN;
            case FDI_UD1:     return Kind::UNKNOWN;
            case FDI_UD2:     return Kind::UNKNOWN;
            case FDI_HLT:     return Kind::UNKNOWN;
            }
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        case Arch::RV64:
            switch (rv64.mnem) {
            default:          return Kind::OTHER;
            case FRV_BEQ:     return Kind::COND_BRANCH;
            case FRV_BNE:     return Kind::COND_BRANCH;
            case FRV_BLT:     return Kind::COND_BRANCH;
            case FRV_BGE:     return Kind::COND_BRANCH;
            case FRV_BLTU:    return Kind::COND_BRANCH;
            case FRV_BGEU:    return Kind::COND_BRANCH;
            case FRV_JAL:     return rv64.rd ? Kind::CALL : Kind::BRANCH;
            case FRV_JALR:    return rv64.rd ? Kind::CALL : Kind::BRANCH;
            case FRV_ECALL:   return Kind::UNKNOWN;
            }
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        case Arch::AArch64:
            switch (_a64.op) {
            default:                 return Kind::OTHER;
            case farmdec::A64_BCOND: return Kind::COND_BRANCH;
            case farmdec::A64_CBZ:   return Kind::COND_BRANCH;
            case farmdec::A64_CBNZ:  return Kind::COND_BRANCH;
            case farmdec::A64_TBZ:   return Kind::COND_BRANCH;
            case farmdec::A64_TBNZ:  return Kind::COND_BRANCH;
            case farmdec::A64_B:     return Kind::BRANCH;
            case farmdec::A64_BL:    return Kind::CALL;
            case farmdec::A64_BR:    return Kind::BRANCH;
            case farmdec::A64_BLR:   return Kind::CALL;
            case farmdec::A64_RET:   return Kind::UNKNOWN;
            case farmdec::A64_SVC:   return Kind::UNKNOWN;
            case farmdec::A64_HVC:   return Kind::UNKNOWN;
            case farmdec::A64_SMC:   return Kind::UNKNOWN;
            case farmdec::A64_BRK:   return Kind::UNKNOWN;
            case farmdec::A64_HLT:   return Kind::UNKNOWN;
            case farmdec::A64_DCPS1: return Kind::UNKNOWN;
            case farmdec::A64_DCPS2: return Kind::UNKNOWN;
            case farmdec::A64_DCPS3: return Kind::UNKNOWN;
            }
#endif // RELLUME_WITH_AARCH64
        default:
            return Kind::UNKNOWN;
        }
    }
    std::optional<uintptr_t> JumpTarget() {
        switch (arch) {
#ifdef RELLUME_WITH_X86_64
        case Arch::X86_64:
            switch (Kind()) {
            case Kind::COND_BRANCH:
            case Kind::BRANCH:
            case Kind::CALL:
                if (op(0).is_pcrel())
                    return end() + op(0).pcrel();
                break;
            default:
                break;
            }
            break;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        case Arch::RV64:
            switch (Kind()) {
            case Kind::COND_BRANCH:
                return start() + rv64.imm;
            case Kind::BRANCH:
            case Kind::CALL:
                if (rv64.mnem == FRV_JAL)
                    return start() + rv64.imm;
            default:
                break;
            }
            break;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        case Arch::AArch64:
            switch (Kind()) {
            case Kind::COND_BRANCH:
                if (_a64.op == farmdec::A64_TBZ || _a64.op == farmdec::A64_TBNZ) {
                    return start() + _a64.tbz.offset;
                }
                /* FALLTHROUGH */
            case Kind::BRANCH:
            case Kind::CALL:
                // ARM PC points at current instruction → start()
                if (_a64.op != farmdec::A64_BR && _a64.op != farmdec::A64_BLR)
                    return start() + _a64.offset;
                break;
            default:
                break;
            }
            break;
#endif // RELLUME_WITH_AARCH64
        default:
            break;
        }
        return std::nullopt;
    }

    /// Fill Instr with the instruction at buf and return number of consumed
    /// bytes (or negative on error). addr is the virtual address of the
    /// instruction.
    int DecodeFrom(Arch arch, const uint8_t* buf, size_t len, uintptr_t addr) {
        this->arch = arch;
        this->addr = addr;
        int res = -1;
        switch (arch) {
#ifdef RELLUME_WITH_X86_64
        case Arch::X86_64:
            res = fd_decode(buf, len, /*mode=*/64, /*addr=*/0, &x86_64);
            break;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        case Arch::RV64:
            res = frv_decode(len, buf, FRV_RV64, &rv64);
            break;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        case Arch::AArch64: {
            if (len < 4) {
                return -1;
            }

            uint32_t binst = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
            fad_decode(&binst, 1, &_a64);
            if (_a64.op == farmdec::A64_ERROR || _a64.op == farmdec::A64_UNKNOWN) {
                return -1;
            }
            res = 4; // all instructions are 32 bits long
            break;
        }
#endif // RELLUME_WITH_AARCH64
        default:
            break;
        }

        if (res >= 0)
            instlen = res;
        return res;
    }
};

} // namespace

#endif
