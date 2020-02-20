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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <deque>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <rellume/rellume.h>

#include <function.h>
#include <instr.h>

namespace rellume {

int Function::Decode(uintptr_t addr, DecodeStop stop, MemReader memacc)
{
    Instr inst;
    uint8_t inst_buf[15];

    std::deque<uintptr_t> addr_queue;
    addr_queue.push_back(addr);

    std::vector<Instr> insts;
    // List of (start_idx,end_idx) (non-inclusive end)
    std::vector<std::pair<size_t,size_t>> blocks;

    // Mapping from address to (block_idx, instr_idx)
    std::unordered_map<uintptr_t, std::pair<size_t,size_t>> addr_map;

    while (!addr_queue.empty())
    {
        uintptr_t cur_addr = addr_queue.front();
        addr_queue.pop_front();

        size_t cur_block_start = insts.size();

        auto cur_addr_entry = addr_map.find(cur_addr);
        while (cur_addr_entry == addr_map.end())
        {
            size_t inst_buf_sz = memacc(cur_addr, inst_buf, sizeof(inst_buf));
            // Sanity check.
            if (inst_buf_sz == 0 || inst_buf_sz > sizeof(inst_buf))
                break;

            int ret = fd_decode(inst_buf, inst_buf_sz, 64, cur_addr, &inst);
            // If we reach an invalid instruction or an instruction we can't
            // decode, stop.
            if (ret < 0)
                break;

            addr_map[cur_addr] = std::make_pair(blocks.size(), insts.size());
            insts.push_back(inst);

            if (stop == DecodeStop::INSTR)
                break;

            if (inst.BreaksAlways() || inst.BreaksConditionally()) {
                if (stop == DecodeStop::BASICBLOCK)
                    break;

                if (inst.BreaksConditionally())
                    addr_queue.push_back(cur_addr + inst.len());

                if (stop == DecodeStop::SUPERBLOCK)
                    break;

                if (inst.HasAbsJumpTarget() && inst.type() != FDI_CALL)
                    addr_queue.push_back(inst.op(0).imm());
                break;
            }
            cur_addr += inst.len();
            cur_addr_entry = addr_map.find(cur_addr);
        }

        if (insts.size() != cur_block_start)
            blocks.push_back(std::make_pair(cur_block_start, insts.size()));

        if (cur_addr_entry != addr_map.end())
        {
            auto& other_blk = blocks[cur_addr_entry->second.first];
            size_t split_idx = cur_addr_entry->second.second;
            if (other_blk.first == split_idx)
                continue;
            size_t end = other_blk.second;
            blocks.push_back(std::make_pair(split_idx, end));
            blocks[cur_addr_entry->second.first].second = split_idx;
            for (size_t j = split_idx; j < end; j++)
                addr_map[insts[j].start()] = std::make_pair(blocks.size()-1, j);
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

} // namespace
