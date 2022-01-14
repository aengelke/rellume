project('rellume', ['cpp'], meson_version: '>=0.49',
        default_options: [
            'buildtype=debugoptimized',
            'warning_level=3',
            'cpp_std=c++17',
        ])

add_project_arguments(['-Wno-unused-parameter', '-Wshadow=local'], language: 'cpp')

libllvm = dependency('llvm', version: ['>=11', '<14'])
add_project_arguments(['-DLL_LLVM_MAJOR='+libllvm.version().split('.')[0]], language: 'cpp')

architectures = []
archdeps = []
if get_option('with_x86_64')
  archdeps += dependency('fadec', static: true,
                         fallback: ['fadec', 'fadec'],
                         default_options: ['archmode=only64'])
  architectures += 'x86_64'
endif
if get_option('with_rv64')
  archdeps += dependency('frvdec', static: true,
                         fallback: ['frvdec', 'frvdec'])
  architectures += 'rv64'
endif

foreach arch : architectures
  add_project_arguments(['-DRELLUME_WITH_@0@'.format(arch.to_upper())], language: 'cpp')
endforeach

rellume_inc = include_directories('include', 'data')
subdir('data/rellume')
subdir('src')
subdir('include/rellume')
librellume = declare_dependency(include_directories: rellume_inc,
                                link_with: librellume_lib,
                                dependencies: [libllvm],
                                sources: [cpustruct_pub])

subdir('tests')
subdir('examples')

pkg = import('pkgconfig')
pkg.generate(librellume_lib,
             subdirs: ['rellume'],
             version: '0.1',
             name: 'rellume',
             filebase: 'rellume',
             description: 'Lift machine code to LLVM-IR')
