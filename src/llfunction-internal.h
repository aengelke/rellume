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

#ifndef LL_FUNCTION_H
#define LL_FUNCTION_H

#include <cstdbool>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Value.h>

#include <llbasicblock-internal.h>
#include <llcommon-internal.h>
#include <llstate-internal.h>


namespace rellume
{

class Function
{
public:
    Function(llvm::Module* mod);
    ~Function();

    Function(Function&& rhs);
    Function& operator=(Function&& rhs);

    Function(const Function&) = delete;
    Function& operator=(const Function&) = delete;

    void EnableOverflowIntrinsics(bool enable);
    void EnableFastMath(bool enable);
    void SetGlobalBase(uintptr_t base, llvm::Value* value);
    BasicBlock* AddBlock(uint64_t address);
    llvm::Function* Lift();

    // Implemented in lldecoder.cc
    int Decode(uintptr_t addr);

private:
    void CreateEntry();
    void CreateExit(llvm::Value* next_rip);

    LLState state;

    llvm::Function* llvm;
    std::vector<BasicBlock*> blocks;
    std::unordered_map<uint64_t,BasicBlock*> block_map;

    BasicBlock* initialBB;
};

}

#endif
