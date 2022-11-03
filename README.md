# Rellume — Lift machine code to LLVM IR

Rellume is a lifter for x86-64/AArch64/RISC-V64 machine code to LLVM IR with focus on the performance of the lifted code. The generated LLVM IR can be compiled and executed again, for example using LLVM's JIT compiler, ideally having the same (or even better) performance as the original code. Special care is taken to model the SIMD instructions and pointers in a way that the optimizer can generate efficient code. The lifter operates on a set of specified instructions (or decodes the control flow automatically) and creates an LLVM-IR function with the same semantics. These functions operate on a generic structure containing the virtual CPU state, but can be wrapped for an arbitrary calling convention.

### Use Cases
- Binary rewriting:
    - Binary Translation: translating machine code to a different architecture while making use of compiler optimizations. This is implemented in [Instrew](https://github.com/aengelke/instrew).
    - Performance improvement: specialization for runtime data, e.g. known parameters or memory locations. This is implemented in the LLVM back-end of [BinOpt](https://github.com/aengelke/binopt).
    - Instrumentation: insert tracing and interception code in hot code paths, where high quality machine code is required.
- Binary analysis: existing tooling for analysis of LLVM IR code can be re-used for binary code.

### Example
See `examples/` for usage examples.

### Publications

- Alexis Engelke. Optimizing Performance Using Dynamic Code Generation. Dissertation. Technical University of Munich, Munich, 2021. ([Thesis](https://mediatum.ub.tum.de/doc/1614897/1614897.pdf))
- Alexis Engelke and Martin Schulz. Instrew: Leveraging LLVM for High Performance Dynamic Binary Instrumentation. VEE'20, March 2020. (Please cite this paper when referring to Rellume.)
- Alexis Engelke and Josef Weidendorfer. Using LLVM for Optimized Light-Weight Binary Re-Writing at Runtime. In Proceedings of the 22nd int. Workshop on High-Level Parallel Programming Models and Supportive Environments (HIPS 2017). Orlando, US, 2017 ([PDF of pre-print version](http://wwwi10.lrr.in.tum.de/~weidendo/pubs/hips17.pdf))

### License

LGPLv2.1+
