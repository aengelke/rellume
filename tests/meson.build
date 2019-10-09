
casefiles = [
    'cases_basic.txt',
    'cases_modrm.txt',
    #'cases_string.txt',
    'cases_sse.txt',
]

assembler = executable('test_assembler', 'test_assembler.cc', dependencies: [libllvm])
driver = executable('test_driver', 'test_driver.cc', dependencies: [librellume])

python3 = find_program('python3')
parsed_cases = custom_target('parsed_cases.txt',
                             command: [python3, files('test_parser.py'), '-o', '@OUTPUT@', '-a', assembler, '@INPUT@'],
                             input: files(casefiles),
                             output: 'parsed_cases.txt')

test('emulation', driver, args: [parsed_cases], protocol: 'tap')
