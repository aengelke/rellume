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

#include "function.h"

#include "arch.h"
#include "basicblock.h"
#include "config.h"
#include "instr.h"
#include <llvm/ADT/SmallVector.h>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace rellume {

namespace {

enum class InstrKind {
    BRANCH,
    COND_BRANCH,
    CALL,
    UNKNOWN,
    OTHER,
};

/// Returns pair of instr kind and absolute jump target (or zero)
std::pair<InstrKind, uint64_t> classifyInstr(Arch arch, const Instr& inst) {
    std::uint64_t branch_target = 0;
    switch (arch) {
#ifdef RELLUME_WITH_X86_64
    case Arch::X86_64:
        switch (inst.type()) {
        default:
            return {InstrKind::OTHER, 0};
        case FDI_JO:
        case FDI_JNO:
        case FDI_JC:
        case FDI_JNC:
        case FDI_JZ:
        case FDI_JNZ:
        case FDI_JBE:
        case FDI_JA:
        case FDI_JS:
        case FDI_JNS:
        case FDI_JP:
        case FDI_JNP:
        case FDI_JL:
        case FDI_JGE:
        case FDI_JLE:
        case FDI_JG:
        case FDI_JCXZ:
        case FDI_LOOP:
        case FDI_LOOPZ:
        case FDI_LOOPNZ:
            return {InstrKind::COND_BRANCH, inst.end() + inst.op(0).pcrel()};
        case FDI_JMP:
            if (inst.op(0).is_pcrel())
                branch_target = inst.end() + inst.op(0).pcrel();
            return {InstrKind::BRANCH, branch_target};
        case FDI_CALL:
            if (inst.op(0).is_pcrel())
                branch_target = inst.end() + inst.op(0).pcrel();
            return {InstrKind::CALL, branch_target};
        case FDI_RET:
        case FDI_SYSCALL:
        case FDI_INT:
        case FDI_INT3:
        case FDI_INTO:
        case FDI_UD0:
        case FDI_UD1:
        case FDI_UD2:
        case FDI_HLT:
            return {InstrKind::UNKNOWN, 0};
        }
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
    case Arch::RV64: {
        const FrvInst* rv64 = inst;
        switch (rv64->mnem) {
        default:
            return {InstrKind::OTHER, 0};
        case FRV_BEQ:
        case FRV_BNE:
        case FRV_BLT:
        case FRV_BGE:
        case FRV_BLTU:
        case FRV_BGEU:
            return {InstrKind::COND_BRANCH, inst.start() + rv64->imm};
        case FRV_JAL:
            branch_target = inst.start() + rv64->imm;
            return {rv64->rd ? InstrKind::CALL : InstrKind::BRANCH, branch_target};
        case FRV_JALR:
            return {rv64->rd ? InstrKind::CALL : InstrKind::BRANCH, 0};
        case FRV_ECALL:
            return {InstrKind::UNKNOWN, 0};
        }
    }
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
    case Arch::AArch64: {
        const farmdec::Inst* a64 = inst;
        switch (a64->op) {
        default:
            return {InstrKind::OTHER, 0};
        case farmdec::A64_BCOND:
        case farmdec::A64_CBZ:
        case farmdec::A64_CBNZ:
            return {InstrKind::COND_BRANCH, inst.start() + a64->offset};
        case farmdec::A64_TBZ:
        case farmdec::A64_TBNZ:
            return {InstrKind::COND_BRANCH, inst.start() + a64->tbz.offset};
        case farmdec::A64_B:
            return {InstrKind::BRANCH, inst.start() + a64->offset};
        case farmdec::A64_BL:
            return {InstrKind::CALL, inst.start() + a64->offset};
        case farmdec::A64_BR:
            return {InstrKind::BRANCH, 0};
        case farmdec::A64_BLR:
            return {InstrKind::CALL, 0};
        case farmdec::A64_RET:
        case farmdec::A64_SVC:
        case farmdec::A64_HVC:
        case farmdec::A64_SMC:
        case farmdec::A64_BRK:
        case farmdec::A64_HLT:
        case farmdec::A64_DCPS1:
        case farmdec::A64_DCPS2:
        case farmdec::A64_DCPS3:
            return {InstrKind::UNKNOWN, 0};
        }
    }
#endif // RELLUME_WITH_AARCH64
    default:
        return {InstrKind::UNKNOWN, 0};
    }
}

} // end anonymous namespace

int Function::Decode(uintptr_t addr, DecodeStop stop, MemReader memacc) {
    uint8_t inst_buf[15];

    llvm::SmallVector<uint64_t> addr_stack;
    addr_stack.push_back(addr);
    while (!addr_stack.empty()) {
        uint64_t start_addr = addr_stack.back();
        addr_stack.pop_back();

        bool new_block = true;
        uint64_t cur_addr = start_addr;
        InstrMapEntry* instr_map_entry = &instr_map[cur_addr];
        while (true) {
            if (instr_map_entry->decoded) {
                instrs[instr_map_entry->instr_idx].new_block = true;
                break;
            }

            size_t count = memacc(cur_addr, inst_buf, sizeof(inst_buf));
            auto& instr = instrs.emplace_back(DecodedInstr{});

            int ret = instr.inst.DecodeFrom(cfg->arch, inst_buf, count, cur_addr);
            if (ret < 0) { // invalid or unknown instruction
                instrs.erase(instrs.end() - 1);
                break;
            }

            instr_map_entry->instr_idx = instrs.size() - 1;
            instr_map_entry->decoded = true;
            instr.new_block = new_block;
            cur_addr += instr.inst.len();
            new_block = false;

            if (stop == DecodeStop::INSTR)
                break;

            auto [kind, jmp_target] = classifyInstr(cfg->arch, instr.inst);

            // For branches, enqueue jump target.
            if (kind == InstrKind::BRANCH || kind == InstrKind::COND_BRANCH) {
                if (jmp_target) {
                    auto& target_entry = instr_map.getOrInsertDefault(jmp_target);
                    target_entry.preds++;
                    if (!target_entry.decoded)
                        addr_stack.push_back(jmp_target);
                    else
                        instrs[target_entry.instr_idx].new_block = true;
                }
            }

            if (kind == InstrKind::CALL)
                instr.inhibit_branch = true;

            // End decoding stream if can't reach next instruction from here.
            if (kind == InstrKind::BRANCH || kind == InstrKind::UNKNOWN ||
                (kind == InstrKind::CALL && !cfg->call_function))
                break;

            // For conditional branches and calls, start a new block.
            // A call still ends a block, because we don't *know* that the
            // execution continues after the call.
            if (kind == InstrKind::COND_BRANCH || kind == InstrKind::CALL) {
                if (stop == DecodeStop::BASICBLOCK)
                    break;
                new_block = true;
            }

            instr_map_entry = &instr_map.getOrInsertDefault(cur_addr);
            instr_map_entry->preds++;
        }

        if (cur_addr != start_addr)
            code_ranges.insert(code_ranges.end() - 1, {start_addr, cur_addr});
        if (stop == DecodeStop::BASICBLOCK)
            break;
    }

    return 0;
}

} // namespace rellume
