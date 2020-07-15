import sys
import os
import os.path
import argparse
import subprocess
import configparser

def add_occlusion_in_batch(npts_dir, occlusion_range):
    pass

def add_noise_in_batch(npts_dir, noise_npts_dir, noise_ratio, noise_range, keyword, keyword2):
    for _, _, files in os.walk(npts_dir):
        for file_name in files:
            if not file_name.endswith('.npts'): continue
            if file_name.find(keyword) < 0: continue
            if file_name.find(keyword2) < 0: continue
            npts_file_path = npts_dir + '/' + file_name
            noise_npts_file_path = noise_npts_dir + '/' + file_name[:-5] + ('_noise_%d_%d.npts' % (int(noise_ratio * 100), int(noise_range * 100)))
            if os.path.isfile(noise_npts_file_path): continue
            subprocess.call(('./generate_gaussian_noise %s %s %f %f' % (npts_file_path, noise_npts_file_path, noise_ratio, noise_range)).split())
            #npts_to_ply(npts_file_path, ply_file_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully add noise for %s  ...' % (npts_file_path ))
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    pass
    pass

def npts_to_ply(file_name, output_file_name):
    if not file_name.endswith('.npts'):
        print('The name of file should end with .npts')
        return
    pts_lines = []
    with open(output_file_name, mode='w') as wf:
        with open(file_name) as rf:
            lines = rf.readlines()
            for line in lines:
                if len(line) > 1 and line.find('nan') < 0:
                    pts_lines.append(line)
                    pass
        wf.write('ply\n')
        wf.write('format ascii 1.0\n')
        wf.write('element vertex %d\n' % len(pts_lines))
        wf.write('property float x\n')
        wf.write('property float y\n')
        wf.write('property float z\n')
        wf.write('property float nx\n')
        wf.write('property float ny\n')
        wf.write('property float nz\n')
        wf.write('end_header\n')
        for line in pts_lines:
            wf.write(line)
    pass

def npts_to_ply_in_batch(npts_dir, ply_dir, keyword):
    for _, _, files in os.walk(npts_dir):
        for file_name in files:
            if not file_name.endswith('.npts'): continue
            if file_name.find(keyword) < 0: continue
            npts_file_path = npts_dir + '/' + file_name
            ply_file_path = ply_dir + '/' + file_name[:-5] + '.ply'
            npts_to_ply(npts_file_path, ply_file_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully convert %s to %s ...' % (npts_file_path, ply_file_path))
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    pass

class RunError(Exception):
    def __init__(self, msg):
        self.value = msg
    def __str__(self):
        return 'Error: ' + self.value

def runcommand(args):
    try:
        print(os.getcwd())
        print(args)
        subprocess.check_call(args)
    except OSError as e:
        raise RunError(args[0] + ': Fork falied with error \'' + e.strerror + '\'')
    except subprocess.CalledProcessError as e:
        raise RunError(args[0] + ': Execution failed with returncode = ' + repr(e.returncode))
		

def parse_config_and_run(config, mpu_file_path, npts_file_path):
        if config.has_section("uniform"):
            args = []
            #args.append('bin/bash')
			#args.append('-c')
            args.append("./uniform_sampler" )
            args.append(mpu_file_path)
            args.append(npts_file_path)

            # required

            args.append(config.get("uniform", "camera_res_x"))
            args.append(config.get("uniform", "camera_res_y"))
            args.append(config.get("uniform", "scan_res"))

            # optional
            if config.has_option("uniform", "min_range"):
                args.append("min_range")
                args.append(config.get("uniform", "min_range"))

            if config.has_option("uniform", "max_range"):
                args.append("max_range")
                args.append(config.get("uniform", "max_range"))

            if config.has_option("uniform", "num_stripes"):
                args.append("num_stripes")
                args.append(config.get("uniform", "num_stripes"))

            if config.has_option("uniform", "laser_fov"):
                args.append("laser_fov")
                args.append(config.get("uniform", "laser_fov"))

            if config.has_option("uniform", "peak_threshold"):
                args.append("peak_threshold")
                args.append(config.get("uniform", "peak_threshold"))

            if config.has_option("uniform", "std_threshold"):
                args.append("std_threshold")
                args.append(config.get("uniform", "std_threshold"))

            if config.has_option("uniform", "additive_noise"):
                args.append("additive_noise")
                args.append(config.get("uniform", "additive_noise"))

            if config.has_option("uniform", "laser_smoother"):
                args.append("laser_smoother")
                args.append(config.get("uniform", "laser_smoother"))

            if config.has_option("uniform", "registration_error"):
                args.append("registration_error")
                args.append(config.get("uniform", "registration_error"))

            if config.has_option("uniform", "normal_type"):
                args.append("normal_type")
                args.append(config.get("uniform", "normal_type"))

            if config.has_option("uniform", "pca_knn"):
                args.append("pca_knn")
                args.append(config.get("uniform", "pca_knn"))

            if config.has_option("uniform", "random_sample_rotation"):
                args.append("random_sample_rotation")
                args.append(config.get("uniform", "random_sample_rotation"))

            runcommand(args)


def scan_with_multi_configs(mpu_file_name, mpu_dir, scan_dir, config_dir):
    for _, _, files in os.walk(config_dir):
        for file_name in files:
            if not file_name.endswith('.conf'): continue
            config_file_path = config_dir + '/' + file_name
            config = configparser.ConfigParser()
            config.read(config_file_path)
            # ======
            mpu_file_path = mpu_dir + '/' + mpu_file_name
            npts_file_path = scan_dir + '/' + mpu_file_name[:-4] + '_' + file_name[:-5] + '.npts'
            if os.path.isfile(npts_file_path): continue
            try:
                parse_config_and_run(config, mpu_file_path, npts_file_path)
            except RunError as re:
                print("Catch an exception...")
                pass
            #subprocess.call(('./implicit_uniform %s %d' % (mpu_file_path, sample_cnt)).split())
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully sample with config %s ...' % config_file_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    pass
    #try:
    #    mpu_file_path = config.get("sampler", "mpu_file_path")
    #    npts_file_path = config.get("sampler", "npts_file_path")
    #    pathdir = config.get("sampler", "pathdir")
    #except Exception:
    #    raise RunError('Input and Output files must be specified')

	
    #if not os.access(mpu_file_path, os.R_OK):
    #    raise RunError('Permission denied: ' + mpu_file_path)

    #os.chdir(pathdir)


def sample_sparse(mpu_dir, scan_dir, config_dir, keyword):
    for _, _, files in os.walk(mpu_dir):
        for file_name in files:
            if not file_name.endswith('.mpu'): continue
            if file_name.find(keyword) < 0: continue
            mpu_file_path = mpu_dir + '/' + file_name
            scan_with_multi_configs(file_name, mpu_dir, scan_dir, config_dir)
            #subprocess.call(('./implicit_uniform %s %d' % (mpu_file_path, sample_cnt)).split())
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully sample scans on %s ...' % mpu_file_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    for _, _, files in os.walk(scan_dir):
        for file_name in files:
            if not file_name.endswith('.png'): continue
            png_file_path = scan_dir + '/' + file_name
            os.remove(png_file_path)
        pass 
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    #parser.add_argument('--ply_dir', type=str, default='mesh', help='the directory of generated meshes(generated by generated_tree_models.py)')
    parser.add_argument('--mpu_dir', type=str, default='mpu', help='the directory of generated mpu(generated by mesh_to_implicit)')
    parser.add_argument('--scan_npts_dir', type=str, default='scan_npts', help='the directory of generated scans(generated by uniform_sampler)')
    parser.add_argument('--scan_ply_dir', type=str, default='scan_ply', help='the directory of generated scans(ply format)')
    parser.add_argument('--scan_npts_noise_dir', type=str, default='scan_npts_noise', help='the directory of generated scans with noise(npts format)')
    parser.add_argument('--scan_ply_noise_dir', type=str, default='scan_ply_noise', help='the directory of generated scans with noise(ply format)')
    parser.add_argument('--keyword', type=str, default='small', help='the directory of generated scans with noise(ply format)')
    parser.add_argument('--config_dir', type=str, default='config', help='the directory of configs(used to config how scanner works)')
    #parser.add_argument('--ply_pc_dir', type=str, default='gt_ply_pc', help='the directory of the output point cloud')
    #parser.add_argument('--pts_num', type=int, default=10000, help='the number of points for each point cloud')
    #parser.add_argument('--translation_x', type=float, default=0.0, help='the x-coord of the translation')
    argument_vals = parser.parse_args()
    sample_sparse(argument_vals.mpu_dir, argument_vals.scan_npts_dir, argument_vals.config_dir, argument_vals.keyword)
    npts_to_ply_in_batch(argument_vals.scan_npts_dir, argument_vals.scan_ply_dir, argument_vals.keyword)
    add_noise_in_batch(argument_vals.scan_npts_dir, argument_vals.scan_npts_noise_dir, 0.3, 0.2, "res_100", argument_vals.keyword)
    add_noise_in_batch(argument_vals.scan_npts_dir, argument_vals.scan_npts_noise_dir, 0.3, 0.3, "res_100", argument_vals.keyword)
    add_noise_in_batch(argument_vals.scan_npts_dir, argument_vals.scan_npts_noise_dir, 0.3, 0.5, "res_100", argument_vals.keyword)
    npts_to_ply_in_batch(argument_vals.scan_npts_noise_dir, argument_vals.scan_ply_noise_dir, argument_vals.keyword)
    #sample_mesh(argument_vals.mpu_dir, argument_vals.npts_dir, argument_vals.pts_num)
    pass
