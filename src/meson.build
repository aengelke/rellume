
rellume_sources = files(
  'basicblock.cc',
  'callconv.cc',
  'facet.cc',
  'function.cc',
  'lldecoder.cc',
  'lifter-base.cc',
  'regfile.cc',
  'rellume.cc',
)

foreach arch : architectures
  subdir('-'.join(arch.split('_')))
endforeach

rellume_inc_priv = include_directories('.')
rellume_flags = [
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
                         install: true)
