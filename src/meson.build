
sources = [
  'basicblock.cc',
  'callconv.cc',
  'facet.cc',
  'function.cc',
  'lldecoder.cc',
  'lifter-base.cc',
  'x86-64/lifter.cc',
  'x86-64/lifter-flags.cc',
  'x86-64/lifter-gp.cc',
  'x86-64/lifter-sse.cc',
  'x86-64/lifter-operand.cc',
  'rv64/lifter.cc',
  'regfile.cc',
  'rellume.cc',
  'transforms.cc',
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
librellume_lib = library('rellume', sources, cpustruct_priv,
                         include_directories: [rellume_inc, rellume_inc_priv],
                         dependencies: [libllvm, fadec, frvdec],
                         c_args: rellume_flags,
                         cpp_args: rellume_flags,
                         cpp_pch: 'pch/llvm.h',
                         install: true)
