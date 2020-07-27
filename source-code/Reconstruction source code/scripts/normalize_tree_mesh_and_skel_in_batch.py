import sys
import os
import argparse
import subprocess


def normalize_in_batch(ply_dir, obj_dir, new_ply_dir, new_obj_dir):
    for _, _, files in os.walk(ply_dir):
        for file_name in files:
            if not file_name.endswith('.ply'): continue
            ply_file_path = ply_dir + '/' + file_name 
            obj_file_path = obj_dir + '/' + file_name[:-4] + '.obj' 
            new_ply_file_path = new_ply_dir + '/' + file_name 
            new_obj_file_path = new_obj_dir + '/' + file_name[:-4] + '.obj' 
            subprocess.call(('./normalize_mesh_and_skeleton.exe %s %s %s %s' % (ply_file_path, obj_file_path, new_ply_file_path, new_obj_file_path)).split())
            print('Successfully normalize %s,  ...' % (file_name))
        pass 
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ply_dir', type=str, default='mesh', help='the directory of mesh in ply')
    parser.add_argument('--obj_dir', type=str, default='mesh_skel', help='the directory of mesh skeleton in npts')
    parser.add_argument('--new_ply_dir', type=str, default='mesh_1', help='the directory of mesh in ply')
    parser.add_argument('--new_obj_dir', type=str, default='mesh_skel_1', help='the directory of mesh skeleton in npts')
    argument_vals = parser.parse_args()
    normalize_in_batch(argument_vals.ply_dir, argument_vals.obj_dir, argument_vals.new_ply_dir, argument_vals.new_obj_dir)
    #fn_main(sys.argv)
