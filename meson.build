project('rellume', ['cpp'],
        default_options: [
            'buildtype=debugoptimized',
            'warning_level=3',
            'cpp_std=c++14',
        ])

add_project_arguments(['-Wno-unused-parameter'], language: 'cpp')

libllvm = dependency('llvm', version: ['>=7', '<9'])
add_project_arguments(['-DLL_LLVM_MAJOR='+libllvm.version().split('.')[0]], language: 'cpp')

fadec_subproject = subproject('fadec', default_options: ['archmode=only64'])
fadec = fadec_subproject.get_variable('fadec')

rellume_inc = include_directories('include')
subdir('src')
subdir('include')
librellume = declare_dependency(include_directories: rellume_inc,
                                link_with: librellume_lib,
                                dependencies: [libllvm])

subdir('tests')
subdir('examples')
