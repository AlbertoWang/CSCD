# .ntps to .obj
def ntps2obj(ntps_file, obj_file):
    with open(ntps_file) as f_in:
        f_out = open(obj_file, 'w')
        for line in f_in:
            xyz_list = line.split(' ')
            f_out.write('v '+line)

ntps2obj('/Users/alberto/Downloads/GraduationDesign/Data/art-scan-13-uniform.npts', '/Users/alberto/Downloads/GraduationDesign-master/src/13_uniform.obj')