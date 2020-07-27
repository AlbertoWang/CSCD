import sys
import os
import os.path
import argparse
import subprocess
import configparser
import random
import h5py
import numpy as np

def check_file_integrity(model_list, patch_pc_dir, patch_skel_dir, patch_num):
    selected_models = []
    pc_patches = []
    skel_patches = []
    with open(model_list) as m:
        for line in m.readlines():
            if line[-1] == '\n': line = line[:-1]
            if line.endswith('.obj'):
                selected_models.append(line)
            else:
                print("Warning: ignore line: " + line)
            pass 
        pass
    for selected_model in selected_models:
        selected_model_file_name = selected_model[:-4]
        for patch_index in range(patch_num):
            pc_patch_i_file_path = ("%s/%s_pc_%d.npts" % (patch_pc_dir, selected_model_file_name, patch_index)) 
            skel_patch_i_file_path = ("%s/%s_%d.npts" % (patch_skel_dir, selected_model_file_name, patch_index)) 
            if not os.path.isfile(pc_patch_i_file_path):
                print("Error: fail to find file: " + pc_patch_i_file_path)
                continue
            if not os.path.isfile(skel_patch_i_file_path):
                print("Error: fail to find file: " + skel_patch_i_file_path)
                continue
            pc_patches.append(pc_patch_i_file_path)        
            skel_patches.append(skel_patch_i_file_path)
            pass
        pass
         
    return pc_patches, skel_patches 
    pass

def make_dataset(patches_pc, patches_skel, dataset_file_path, samples_num_pc, samples_num_skel):
    if patches_pc is None or patches_skel is None:
        print("Error: patches list is none")
        return

    if len(patches_pc) != len(patches_skel):
        print("Error: invalid patches size, pc: %d, skel: %d" % (len(patches_pc), len(patches_skel)))
        return
        pass  
    # prepare data
    data_skel = np.zeros(shape=(len(patches_skel), samples_num_skel, 3), dtype=float) # 1024
    data_pc = np.zeros(shape=(len(patches_pc), samples_num_pc, 3), dtype=float) # 256

    # fill the array
    print("Start to fill the data array...")
    for patches_index in range(len(patches_pc)):
        patches_pc_i_file_name = patches_pc[patches_index]
        patches_skel_i_file_name = patches_skel[patches_index]
        with open(patches_pc_i_file_name) as f:
            line_cnt = 0
            for line in f.readlines():
                nums = line.split()
                if len(nums) < 3: 
                    print("Verbose: omit line: " + line)
                    continue
                data_pc[patches_index][line_cnt][0] = float(nums[0])
                data_pc[patches_index][line_cnt][1] = float(nums[1])
                data_pc[patches_index][line_cnt][2] = float(nums[2])
                line_cnt += 1
                pass 
            pass 
            if line_cnt < samples_num_pc:
                print("Severe WARNing: num of points in patch %d is less than expected" % patch_index)
                print("We will randomly select points from this patch to fill the array")
                for add_point_index in range(samples_num_pc - line_cnt):
                    random_point_index = randint(0, line_cnt - 1)
                    data_pc[patches_index][line_cnt + add_point_index] = data_pc[patches_index][random_point_index]
                    pass
                pass
        with open(patches_skel_i_file_name) as f:
            line_cnt = 0
            for line in f.readlines():
                nums = line.split()
                if len(nums) < 3: 
                    print("Verbose: omit line: " + line)
                    continue
                data_skel[patches_index][line_cnt][0] = float(nums[0])
                data_skel[patches_index][line_cnt][1] = float(nums[1])
                data_skel[patches_index][line_cnt][2] = float(nums[2])
                line_cnt += 1
                pass 
            pass 
            if line_cnt < samples_num_skel:
                print("Severe WARNing: num of points in patch %d is less than expected" % patch_index)
                print("We will randomly select points from this patch to fill the array")
                for add_point_index in range(samples_num_skel - line_cnt):
                    random_point_index = randint(0, line_cnt - 1)
                    data_skel[patches_index][line_cnt + add_point_index] = data_skel[patches_index][random_point_index]
                    pass
                pass
        pass 

    print("The shape of skeleton dataset:")
    print(data_skel.shape)
    print("The shape of point cloud dataset:")
    print(data_pc.shape)
    # two dataset: "poisson_1024", "poisson_256" 
    # each dataset is like a numpy array
    f = h5py.File(dataset_file_path, "w")
    dataset_1024 = f.create_dataset("poisson_gt", data_skel.shape, dtype="float32", data=data_skel)
    dataset_256 = f.create_dataset("poisson_pc", data_pc.shape, dtype="float32", data=data_pc)
    print("Writing data set to " + dataset_file_path)
    pass

def generate_scan_patch(selected_models, scan_dir, seed_dir, patch_dir, sample_num):
    if selected_models is None:
        print("Error: selected models is None,")
        return
    for _, _, files in os.walk(scan_dir):
        for file_name in files:
            if not file_name.endswith('.npts'): continue
            scan_file_path = scan_dir + '/' + file_name
            seed_file_path = seed_dir + '/' + file_name[:-5] + '.npts'
            output_patch_file_path_stencil = patch_dir + '/' + file_name[:-5] 
            skel_file_path = file_name[:-5] + '.obj'
            if skel_file_path not in selected_models: continue
            if not os.path.isfile(seed_file_path): 
                print('Warning: cannot find seed file: ' + seed_file_path)
                continue
            subprocess.call(('./read_seeds_and_sample_pc %s %s %s %d' % (scan_file_path, seed_file_path, output_patch_file_path_stencil, sample_num)).split())
            #npts_to_ply(npts_file_path, ply_file_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully convert %s to %s, %s ...' % (obj_file_path, output_seed_file_path_stencil, output_patch_file_path_stencil))
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    pass


def generate_seeds_and_skel_patch(model_list_file, skel_dir, seed_dir, patch_dir, patch_num, sample_num):
    if not os.path.isfile(model_list_file):
        print("Error: cannot file model list file: " + model_list_file)
        return
    selected_models = []
    with open(model_list_file) as m:
        for line in m.readlines():
            if line[-1] == '\n': line = line[:-1]
            if line.endswith('.obj'):
                selected_models.append(line)
            else:
                print("Warning: ignore line: " + line)
            pass 
        pass
    for _, _, files in os.walk(skel_dir):
        for file_name in files:
            if not file_name.endswith('.obj'): continue
            if file_name not in selected_models: continue
            obj_file_path = skel_dir + '/' + file_name
            output_seed_file_path_stencil = seed_dir + '/' + file_name[:-4] + '.npts'
            output_patch_file_path_stencil = patch_dir + '/' + file_name[:-4] 
            subprocess.call(('./select_seeds_and_sample_gt %s %s %s %d %d' % (obj_file_path, output_seed_file_path_stencil, output_patch_file_path_stencil, patch_num, sample_num)).split())
            #npts_to_ply(npts_file_path, ply_file_path)
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('Successfully convert %s to %s, %s ...' % (obj_file_path, output_seed_file_path_stencil, output_patch_file_path_stencil))
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        pass 
    return selected_models



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_list', type=str, default='models_selected.list', help='the list of selected models')
    parser.add_argument('--dataset_name', type=str, default='train.h5', help='the name of generated h5 dataset')
    parser.add_argument('--pc_patch_dir', type=str, default='patch_pc', help='the directory of generated patches(generated by read_seed_and_sample_pc)')
    parser.add_argument('--skel_patch_dir', type=str, default='patch_skel', help='the directory of raw skeleton(generated by select_seeds_and_sample_gt)')
    parser.add_argument('--patch_num', type=int, default=100, help='the num of patches for each model')
    parser.add_argument('--samples_per_pc_patch', type=int, default=256, help='the num of sample point for each point cloud patch')
    parser.add_argument('--samples_per_skel_patch', type=int, default=1024, help='the num of sample point for each skeleton patch')
    #parser.add_argument('--seed_dir', type=str, default='seed', help='the directory of seeds file(generated by select_seeds_and_sample_gt)')
    argument_vals = parser.parse_args()
    patches_pc, patches_skel = check_file_integrity(argument_vals.model_list, argument_vals.pc_patch_dir, argument_vals.skel_patch_dir, argument_vals.patch_num)
    make_dataset(patches_pc, patches_skel, argument_vals.dataset_name, argument_vals.samples_per_pc_patch, argument_vals.samples_per_skel_patch)
    pass
