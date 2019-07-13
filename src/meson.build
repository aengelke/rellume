
sources = [
  'facet.cc',
  'llbasicblock.cc',
  'lldecoder.cc',
  'llflags.cc',
  'llfunc.cc',
  'llinstr.cc',
  'llinstruction-gp.cc',
  'llinstruction-sse.cc',
  'lloperand.cc',
  'llregfile.cc',
  'rellume.cc',
]

rellume_inc_priv = include_directories('.')
rellume_flags = [
  '-DLL_LLVM_MAJOR='+libllvm.version().split('.')[0],
  '-DRELLUME_ENABLE_CPP_HEADER',
  '-fvisibility=hidden',
  '-fno-exceptions',
  '-fno-unwind-tables',
  '-fno-rtti',
]
librellume_lib = library('rellume', sources,
                         include_directories: [rellume_inc, rellume_inc_priv],
                         dependencies: [libllvm, fadec],
                         c_args: rellume_flags,
                         cpp_args: rellume_flags,
                         cpp_pch: 'pch/llvm.h',
                         install: true)
