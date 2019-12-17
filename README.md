# Rellume — Lifts x86-64 to LLVM IR

Rellume is a lifter for x86-64 machine code to LLVM IR with focus on the performance of the lifted code. The generated LLVM IR can be compiled and executed again, for example using LLVM's JIT compiler, ideally having the same (or even better) performance as the original code. Special care is taken to model the SSE instructions and pointers in a way that the optimizer can generate efficient code. The lifter operates on a set of specified instructions (or decodes the control flow automatically) and creates an LLVM-IR function with the same semantics. These functions operate on a generic structure containing the virtual x86-64 CPU state, but can be wrapped for an arbitrary calling convention.

### Use Cases
- Binary rewriting:
    - Performance improvement: specialization for runtime data, e.g. known parameters or memory locations. This is implemented in the LLVM back-end of [DBrew](https://github.com/caps-tum/dbrew/).
    - Instrumentation: insert tracing and interception code in hot code paths, where high quality machine code is required.
- Binary analysis: existing tooling for analysis of LLVM IR code can be re-used for binary code.

### Example
See [examples/lifter.c](https://github.com/aengelke/rellume/blob/master/examples/lifter.c)

### Publications

- Alexis Engelke and Josef Weidendorfer. Using LLVM for Optimized Light-Weight Binary Re-Writing at Runtime. In Proceedings of the 22nd int. Workshop on High-Level Parallel Programming Models and Supportive Environments (HIPS 2017). Orlando, US, 2017 ([PDF of pre-print version](http://wwwi10.lrr.in.tum.de/~weidendo/pubs/hips17.pdf))

### License

LGPLv2.1+
