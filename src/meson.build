
rellume_sources = files(
  'basicblock.cc',
  'callconv.cc',
  'facet.cc',
  'function.cc',
  'lldecoder.cc',
  'lifter-base.cc',
  'regfile.cc',
  'rellume.cc',
  'transforms.cc',
)

foreach arch : architectures
  subdir('-'.join(arch.split('_')))
endforeach

rellume_inc_priv = include_directories('.')
rellume_flags = [
  '-DLL_LLVM_MAJOR='+libllvm.version().split('.')[0],
  '-DRELLUME_ENABLE_CPP_HEADER',
  '-fvisibility=hidden',
  '-fno-exceptions',
  '-fno-unwind-tables',
  '-fno-rtti',
]
librellume_lib = library('rellume', rellume_sources, cpustruct_priv,
                         include_directories: [rellume_inc, rellume_inc_priv],
                         dependencies: [libllvm] + archdeps,
                         c_args: rellume_flags,
                         cpp_args: rellume_flags,
                         cpp_pch: 'pch/llvm.h',
                         install: true)
