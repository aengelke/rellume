python3 = find_program('python3')
cpustruct_pub = custom_target('cpustruct.inc',
                              command: [python3, files('cpustruct_parser.py'), '-o', '@OUTPUT@', '@INPUT@'],
                              input: files('cpustruct.json'),
                              output: 'cpustruct.inc',
                              install: true,
                              install_dir: get_option('includedir') / 'rellume')
cpustruct_priv = custom_target('cpustruct-private.inc',
                               command: [python3, files('cpustruct_parser.py'), '-p', '-o', '@OUTPUT@', '@INPUT@'],
                               input: files('cpustruct.json'),
                               output: 'cpustruct-private.inc')
