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

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace rellume {

static void InstrFlags(Instr::Type ty, bool* breaks, bool* breaks_cond,
                       bool* has_jmp_target) {
    switch (ty) {
    default:          break;
    case FDI_JO:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JNO:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JC:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JNC:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JZ:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JNZ:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JBE:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JA:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JS:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JNS:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JP:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JNP:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JL:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JGE:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JLE:     *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JG:      *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JCXZ:    *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_LOOP:    *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_LOOPZ:   *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_LOOPNZ:  *breaks_cond = true; *has_jmp_target = true; break;
    case FDI_JMP:     *breaks = true;      *has_jmp_target = true; break;
    case FDI_CALL:    *breaks = true;      *has_jmp_target = true; break;
    case FDI_RET:     *breaks = true; break;
    case FDI_SYSCALL: *breaks = true; break;
    case FDI_INT:     *breaks = true; break;
    case FDI_INT3:    *breaks = true; break;
    case FDI_INTO:    *breaks = true; break;
    case FDI_UD0:     *breaks = true; break;
    case FDI_UD1:     *breaks = true; break;
    case FDI_UD2:     *breaks = true; break;
    case FDI_HLT:     *breaks = true; break;
    }
}

int Function::Decode(uintptr_t addr, DecodeStop stop, MemReader memacc) {
    Instr inst;
    uint8_t inst_buf[15];

    std::deque<uintptr_t> addr_queue;
    addr_queue.push_back(addr);

    std::vector<Instr> insts;
    // List of (start_idx,end_idx) (non-inclusive end)
    std::vector<std::pair<size_t, size_t>> blocks;

    // Mapping from address to (block_idx, instr_idx)
    std::unordered_map<uintptr_t, std::pair<size_t, size_t>> addr_map;

    while (!addr_queue.empty()) {
        uintptr_t cur_addr = addr_queue.front();
        addr_queue.pop_front();

        size_t cur_block_start = insts.size();

        auto cur_addr_entry = addr_map.find(cur_addr);
        while (cur_addr_entry == addr_map.end()) {
            size_t inst_buf_sz = memacc(cur_addr, inst_buf, sizeof(inst_buf));
            // Sanity check.
            if (inst_buf_sz == 0 || inst_buf_sz > sizeof(inst_buf))
                break;

            int ret = inst.DecodeFrom(Arch::X86_64, inst_buf, inst_buf_sz, cur_addr);
            if (ret < 0) // invalid or unknown instruction
                break;

            addr_map[cur_addr] = std::make_pair(blocks.size(), insts.size());
            insts.push_back(inst);

            if (stop == DecodeStop::INSTR)
                break;

            bool breaks = false, breaks_cond = false, has_jmp_target = false;
            InstrFlags(inst.type(), &breaks, &breaks_cond, &has_jmp_target);
            if (breaks || breaks_cond) {
                if (stop == DecodeStop::BASICBLOCK)
                    break;

                // If we want explicit call/ret semantics, assume that a call
                // actually returns to the same place.
                if (breaks_cond ||
                    (inst.type() == FDI_CALL && cfg->call_function))
                    addr_queue.push_back(cur_addr + inst.len());
                if (has_jmp_target && inst.type() != FDI_CALL &&
                    inst.op(0).is_pcrel())
                    addr_queue.push_back(inst.end() + inst.op(0).pcrel());
                break;
            }
            cur_addr += inst.len();
            cur_addr_entry = addr_map.find(cur_addr);
        }

        if (insts.size() != cur_block_start)
            blocks.push_back(std::make_pair(cur_block_start, insts.size()));

        if (cur_addr_entry != addr_map.end()) {
            auto& other_blk = blocks[cur_addr_entry->second.first];
            size_t split_idx = cur_addr_entry->second.second;
            if (other_blk.first == split_idx)
                continue;
            size_t end = other_blk.second;
            blocks.push_back(std::make_pair(split_idx, end));
            blocks[cur_addr_entry->second.first].second = split_idx;
            size_t new_block_idx = blocks.size() - 1;
            for (size_t j = split_idx; j < end; j++)
                addr_map[insts[j].start()] = std::make_pair(new_block_idx, j);
        }
    }

    bool first_inst = true;
    for (auto it = blocks.begin(); it != blocks.end(); it++) {
        uint64_t block_addr = insts[it->first].start();
        for (size_t j = it->first; j < it->second; j++) {
            if (!AddInst(block_addr, insts[j])) {
                // If we fail on the first instruction, propagate error.
                if (first_inst)
                    return 1;
                // Otherwise continue with other basic blocks.
                break;
            }
            first_inst = false;
        }
    }

    // If we didn't lift a single instruction, return error code.
    if (first_inst)
        return 1;

    return 0;
}

} // namespace rellume
