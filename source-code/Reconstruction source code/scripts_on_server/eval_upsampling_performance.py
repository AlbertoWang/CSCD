import sys
import os
import os.path
import argparse
import subprocess
import configparser


def run_eval_in_batch(mpu_dir, mesh_ply_dir, upsample_result_dir, result_dir):
    for _, _, files in os.walk(upsample_result_dir):
        for file_name in files:
            if not file_name.endswith('.ply'): continue
            first_dot_pos = file_name.find('.')
            # if file_name.find(keyword) < 0: continue
            ply_pc_path = upsample_result_dir + '/' + file_name
            ply_mesh_path = mesh_ply_dir + '/' + file_name[:first_dot_pos] + '.ply'
            mpu_mesh_path = mpu_dir + '/' + file_name[:first_dot_pos] + '.mpu'
            result_path = result_dir + '/' + file_name[:first_dot_pos]  
            expected_dist_path = result_dir + '/' + file_name[:first_dot_pos] + '.dist' 
            subprocess.call(('./bin/run_evaluation %s %s %s %s %d' % (ply_mesh_path, mpu_mesh_path, ply_pc_path, result_path, 0)).split())
            # subprocess.call(('./bin/read_dist %s' % (expected_dist_path)).split())

            #subprocess.call(('./implicit_uniform %s %d' % (mpu_file_path, sample_cnt)).split())
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully run evaluation on %s ...' % ply_pc_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    #parser.add_argument('--ply_dir', type=str, default='mesh', help='the directory of generated meshes(generated by generated_tree_models.py)')
    parser.add_argument('--mpu_dir', type=str, default='mpu', help='the directory of generated mpu from perfect mesh(generated by mesh_to_implicit)')
    parser.add_argument('--mesh_ply_dir', type=str, default='mesh', help='the directory of perfect mesh(generated by tree_generate)')
    parser.add_argument('--scan_upsample_dir', type=str, default='scan_ply_upsample', help='the directory of upsampling result(ply format)')
    parser.add_argument('--eval_result_dir', type=str, default='result_eval_upsample', help='the directory of result of evaluation (ply format)')
    parser.add_argument('--keyword', type=str, default='small', help='the directory of generated scans with noise(ply format)')
    # parser.add_argument('--config_dir', type=str, default='config', help='the directory of configs(used to config how scanner works)')
    #parser.add_argument('--translation_x', type=float, default=0.0, help='the x-coord of the translation')
    argument_vals = parser.parse_args()
    run_eval_in_batch(argument_vals.mpu_dir, argument_vals.mesh_ply_dir, argument_vals.scan_upsample_dir, argument_vals.eval_result_dir)
    #sample_mesh(argument_vals.mpu_dir, argument_vals.npts_dir, argument_vals.pts_num)
    pass

