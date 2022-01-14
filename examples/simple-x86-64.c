/**
 * This file is part of Rellume.
 *
 * (c) 2022, Alexis Engelke <alexis.engelke@googlemail.com>
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
#include <llvm-c/Core.h>

#include <rellume/rellume.h>


int main(void) {
    // Create LLVM module
    LLVMModuleRef mod = LLVMModuleCreateWithName("lifter");

    static const unsigned char code[] = {
        0x48, 0x89, 0xf8, // mov rax,rdi
        0x48, 0x39, 0xf7, // cmp rdi,rsi
        0x7d, 0x03,       // jge $+3
        0x48, 0x89, 0xf0, // mov rax,rsi
        0xc3,             // ret
    };

    // Create function for lifting
    LLConfig* cfg = ll_config_new();
    ll_config_set_architecture(cfg, "x86-64");
    ll_config_set_call_ret_clobber_flags(cfg, true);
    LLFunc* fn = ll_func_new(mod, cfg);
    // Lift the whole function by following all direct jumps
    ll_func_decode_cfg(fn, (uintptr_t) code, NULL, NULL);
    LLVMValueRef llvm_fn = ll_func_lift(fn);
    LLVMDumpValue(llvm_fn);

    return 0;
}
