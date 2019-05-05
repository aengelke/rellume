project('libdbrew', ['c', 'cpp'],
        default_options: [
            'buildtype=debugoptimized',
            'warning_level=3',
            'c_std=gnu99',
            'cpp_std=c++11',
        ])

if get_option('warning_level').to_int() >= 3
  add_project_arguments(['-Wmissing-field-initializers',
                         '-Wunused-parameter',
                         '-Wold-style-definition',
                         '-Wmissing-declarations',
                         '-Wmissing-prototypes',
                         '-Wredundant-decls',
                         '-Wmissing-noreturn',
                         '-Wshadow',
                         '-Wpointer-arith',
                         '-Wwrite-strings',
                         '-Winline',
                         '-Wformat-nonliteral',
                         '-Wformat-security',
                         '-Wswitch-default',
                         '-Winit-self',
                         '-Wnested-externs',
                         '-Wstrict-prototypes',
                         '-Wmissing-include-dirs',
                         '-Wundef',
                         '-Wmissing-format-attribute'
                        ], language: 'c')
endif

add_project_arguments(['-Wno-unused-parameter'], language: 'cpp')

libllvm = dependency('llvm', version: ['>=4', '<8'])

fadec_subproject = subproject('fadec', default_options: ['archmode=only64'])
fadec = fadec_subproject.get_variable('libx86decode')

rellume_inc = include_directories('include')
subdir('src')
librellume = declare_dependency(include_directories: rellume_inc,
                                link_with: librellume_lib,
                                dependencies: [libllvm])

subdir('examples')
