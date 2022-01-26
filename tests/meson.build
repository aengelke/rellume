casefiles = {
  'x86_64': [
    'cases_basic.txt',
    'cases_modrm.txt',
    'cases_string.txt',
    'cases_sse.txt',
  ],
  'rv64': [
    'cases_rv64_basic.txt',
  ],
}
llvm_targets = {
  'x86_64': 'X86',
  'rv64': 'RISCV',
}

test_architectures = []
test_assembler_flags = []
cpp = meson.get_compiler('cpp')
foreach arch : architectures
  if not llvm_targets.has_key(arch)
    continue
  endif
  tgt_init = 'LLVMInitialize@0@Target'.format(llvm_targets[arch])
  test_code = '#include<llvm/Support/TargetSelect.h>\nint main(){@0@();}'.format(tgt_init)
  if cpp.compiles(test_code, dependencies: libllvm)
    test_architectures += arch
    test_assembler_flags += '-DTARGET_@0@'.format(arch.to_upper())
  else
    warning('provided LLVM does not support assembling for @0@'.format(arch))
  endif
endforeach

assembler = executable('test_assembler', 'test_assembler.cc',
                       cpp_args: test_assembler_flags,
                       dependencies: [libllvm])
driver = executable('test_driver', 'test_driver.cc', cpustruct_priv, dependencies: [librellume])

python3 = find_program('python3')
foreach arch : test_architectures
  parsed_cases = custom_target('parsed_cases_@0@.txt'.format(arch),
                               command: [python3, files('test_parser.py'), '-o', '@OUTPUT@', '-a', assembler, '-A', arch, '@INPUT@'],
                               input: files(casefiles[arch]),
                               output: 'parsed_cases_@0@.txt'.format(arch))
  test('emulation-@0@'.format(arch), driver,
       args: ['-A', arch, parsed_cases], protocol: 'tap')
endforeach
