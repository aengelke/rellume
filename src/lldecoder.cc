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

int Function::Decode(uintptr_t addr, DecodeStop stop, MemReader memacc) {
    uint8_t inst_buf[15];

    llvm::SmallVector<uint64_t> addr_stack;
    addr_stack.push_back(addr);
    while (!addr_stack.empty()) {
        uint64_t start_addr = addr_stack.back();
        addr_stack.pop_back();

        bool new_block = true;
        uint64_t cur_addr = start_addr;
        while (true) {
            auto cur_idx_iter = instr_map.find(cur_addr);
            if (cur_idx_iter != instr_map.end()) {
                instrs[cur_idx_iter->second].new_block = true;
                break;
            }

            size_t count = memacc(cur_addr, inst_buf, sizeof(inst_buf));
            auto& instr = instrs.emplace_back(DecodedInstr{});

            int ret = instr.inst.DecodeFrom(cfg->arch, inst_buf, count, cur_addr);
            if (ret < 0) { // invalid or unknown instruction
                instrs.erase(instrs.end() - 1);
                break;
            }

            instr_map[cur_addr] = instrs.size() - 1;
            instr.new_block = new_block;
            cur_addr += instr.inst.len();
            new_block = false;

            if (stop == DecodeStop::INSTR)
                break;

            switch (instr.inst.Kind()) {
            case Instr::Kind::UNKNOWN:
                goto end_superblock;
            case Instr::Kind::BRANCH:
                if (auto jmp_target = instr.inst.JumpTarget())
                    addr_stack.push_back(jmp_target.value());
                goto end_superblock;
            case Instr::Kind::COND_BRANCH:
                if (auto jmp_target = instr.inst.JumpTarget())
                    addr_stack.push_back(jmp_target.value());
                new_block = true;
                break; // continue at cur_addr in new block
            case Instr::Kind::CALL:
                if (!cfg->call_function)
                    goto end_superblock;
                // A call still ends a block, because we don't *know* that the
                // execution continues after the call.
                new_block = true;
                break; // continue at cur_addr in new block
            default:
                break;
            }

            if (new_block && stop == DecodeStop::BASICBLOCK)
                break;
        };

    end_superblock:
        if (cur_addr != start_addr)
            code_ranges.insert(code_ranges.end() - 1, {start_addr, cur_addr});
        if (stop == DecodeStop::BASICBLOCK)
            break;
    }

    return 0;
}

} // namespace rellume
