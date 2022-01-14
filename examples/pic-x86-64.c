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

    LLVMValueRef pcbase = LLVMAddGlobal(mod, LLVMInt8Type(), "pcbase");
    LLVMValueRef pcbase_val = LLVMConstPtrToInt(pcbase, LLVMInt64Type());

    static const unsigned char code[] = {
        0x48, 0x8B, 0x0D, 0x00, 0x00, 0x34, 0x12, // mov rcx, [rip+0x12340000]
        0x48, 0x8D, 0x05, 0x00, 0x00, 0x34, 0x12, // lea rax, [rip+0x12340000]
        0x7d, 0xf0,                               // jge $-16
        0x0f, 0x02,                                     // ret
        0xc3,                                     // ret
    };

    // Create function for lifting
    LLConfig* cfg = ll_config_new();
    ll_config_set_architecture(cfg, "x86-64");
    ll_config_set_call_ret_clobber_flags(cfg, true);
    ll_config_set_pc_base(cfg, 0, pcbase_val);
    LLFunc* fn = ll_func_new(mod, cfg);
    // Lift the whole function by following all direct jumps
    ll_func_decode_cfg(fn, (uintptr_t) code, NULL, NULL);
    LLVMValueRef llvm_fn = ll_func_lift(fn);
    (void) llvm_fn;
    LLVMDumpModule(mod);

    return 0;
}
