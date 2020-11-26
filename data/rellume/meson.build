python3 = find_program('python3')
cpustruct_in = ['cpustruct-x86_64', 'cpustruct-rv64']
cpustruct_pub = []
cpustruct_priv = []
foreach cpustruct : cpustruct_in
  cpustruct_pub += custom_target('@0@.inc'.format(cpustruct),
                                 command: [python3, files('cpustruct_parser.py'), '-o', '@OUTPUT@', '@INPUT@'],
                                 input: files('@0@.json'.format(cpustruct)),
                                 output: '@0@.inc'.format(cpustruct),
                                 install: true,
                                 install_dir: get_option('includedir') / 'rellume')
  cpustruct_priv += custom_target('@0@-private.inc'.format(cpustruct),
                                  command: [python3, files('cpustruct_parser.py'), '-p', '-o', '@OUTPUT@', '@INPUT@'],
                                  input: files('@0@.json'.format(cpustruct)),
                                  output: '@0@-private.inc'.format(cpustruct))
endforeach
