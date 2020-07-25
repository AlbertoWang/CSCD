import sys
# .ntps to .obj
def ntps2obj(ntps_file, obj_file):
    with open(ntps_file) as f_in:
        f_out = open(obj_file, 'w')
        for line in f_in:
            xyz_list = line.split(' ')
            f_out.write('v '+line)

if len(sys.argv) < 3:
    print("npts to obj:\nformat_scripts xxx.npts xxx.obj")
    exit
ntps2obj(sys.argv[1], sys.argv[2])