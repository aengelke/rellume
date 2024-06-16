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

#ifndef RELLUME_FUNCTION_INFO_H
#define RELLUME_FUNCTION_INFO_H

#include <cstdbool>
#include <cstdint>
#include <memory>
#include <vector>


namespace llvm {
class Function;
class Instruction;
class Value;
}

namespace rellume {

#ifdef RELLUME_WITH_X86_64
namespace SptrIdx::x86_64 {
    enum {
#define RELLUME_MAPPED_REG(nameu,...) nameu,
#include <rellume/cpustruct-x86_64-private.inc>
#undef RELLUME_MAPPED_REG
    };
}
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
namespace SptrIdx::rv64 {
    enum {
#define RELLUME_MAPPED_REG(nameu,...) nameu,
#include <rellume/cpustruct-rv64-private.inc>
#undef RELLUME_MAPPED_REG
    };
}
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
namespace SptrIdx::aarch64 {
    enum {
#define RELLUME_MAPPED_REG(nameu,...) nameu,
#include <rellume/cpustruct-aarch64-private.inc>
#undef RELLUME_MAPPED_REG
    };
}
#endif // RELLUME_WITH_AARCH64

class ArchBasicBlock;
class RegFile;

/// CallConvPack records which registers were changed in a basic block,
/// and pre-computed LLVM store instructions for them.
///
/// After the function is lifted, its packs are optimised to avoid
/// passing unnecessary store instructions to the LLVM optimiser, which
/// would struggle with the many superfluous stores.
struct CallConvPack {
    std::unique_ptr<RegFile> regfile;
    llvm::Instruction* packBefore;
    ArchBasicBlock* bb;
};

/// FunctionInfo holds the LLVM objects of the lifted function and its
/// environment: sptr is the single argument of the function and stands
/// for "CPU struct pointer". A CPU struct stores a register set. See
/// data/rellume/*cpu.json for the architecture-dependent definitions.
///
/// The CPU struct concept allows passing arguments and returning values
/// without requiring knowledge of the calling convention or the function
/// signature.
struct FunctionInfo {
    /// The function itself
    llvm::Function* fn;
    /// The sptr argument, and its elements
    llvm::Value* sptr_raw;
    std::vector<llvm::Value*> sptr;

    uint64_t pc_base_addr;
    llvm::Value* pc_base_value;

    std::vector<CallConvPack> call_conv_packs;
};


} // namespace

#endif
