casefiles = [
  ['x86_64', [
    'cases_basic.txt',
    'cases_modrm.txt',
    'cases_string.txt',
    'cases_sse.txt',
  ]],
  ['rv64', [
    'cases_rv64_basic.txt',
  ]],
]

assembler = executable('test_assembler', 'test_assembler.cc', dependencies: [libllvm])
driver = executable('test_driver', 'test_driver.cc', cpustruct_priv, dependencies: [librellume])

python3 = find_program('python3')
foreach arch : casefiles
  parsed_cases = custom_target('parsed_cases_@0@.txt'.format(arch[0]),
                               command: [python3, files('test_parser.py'), '-o', '@OUTPUT@', '-a', assembler, '-A', arch[0], '@INPUT@'],
                               input: files(arch[1]),
                               output: 'parsed_cases_@0@.txt'.format(arch[0]))
  test('emulation-@0@'.format(arch[0]), driver,
       args: ['-A', arch[0], parsed_cases], protocol: 'tap')
endforeach
