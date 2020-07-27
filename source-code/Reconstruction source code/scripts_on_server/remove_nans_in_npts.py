import sys

def fn_main(argv):
    if len(argv) < 2:
        print('Usage: %s xxx.npts' % argv[0])
        return
    file_name = argv[1]
    if not file_name.endswith('.npts'):
        print('The name of file should end with .npts')
        return
    output_file_name = file_name[:-5] + '_clean.npts'
    with open(file_name) as rf:
        with open(output_file_name, mode='w') as wf:
            for line in rf.readlines():
                if line.find('nan') < 0:
                    wf.write(line)
                pass
    pass

if __name__ == '__main__':
    fn_main(sys.argv)
