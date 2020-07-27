import sys
import os
import argparse
import subprocess


def ply_to_npts_in_batch(ply_dir, npts_dir):
    for _, _, files in os.walk(ply_dir):
        for file_name in files:
            if not file_name.endswith('.ply'): continue
            ply_file_path = ply_dir + '/' + file_name 
            npts_file_path = npts_dir + '/' + file_name[:-4] + '.npts'
            subprocess.call(('./ply_2_npts.exe %s %s' % (ply_file_path, npts_file_path)).split())
            print('Successfully convert %s to %s ...' % (ply_file_path, npts_file_path))
        pass 
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ply_dir', type=str, default='scan_ply', help='the directory of point clouds in ply')
    parser.add_argument('--npts_dir', type=str, default='scan_npts', help='the directory of point clouds in npts')
    argument_vals = parser.parse_args()
    ply_to_npts_in_batch(argument_vals.ply_dir, argument_vals.npts_dir)
    #fn_main(sys.argv)
