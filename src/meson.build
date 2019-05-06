
sources = [
  'llbasicblock.c',
  'lldecoder.cc',
  'llflags.c',
  'llfunc.c',
  'llinstr.c',
  'llinstruction-callret.c',
  'llinstruction-gp.c',
  'llinstruction-sse.c',
  'llinstruction-stack.c',
  'lloperand.c',
  'llregfile.c',
  'llsupport.cc',
]

rellume_inc_priv = include_directories('.')
rellume_flags = ['-DLL_LLVM_MAJOR='+libllvm.version().split('.')[0]]
librellume_lib = static_library('rellume', sources,
                                include_directories: [rellume_inc, rellume_inc_priv],
                                dependencies: [libllvm, fadec],
                                c_args: rellume_flags,
                                cpp_args: rellume_flags)
