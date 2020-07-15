import subprocess

def run_recon():
    print('Be sure that xxx.exe exists in ./ ')
    skel_dir = 'D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result'
    # 1, 12, 13, 14, 15, 16
    scan_indices = [16]
    compared_methods = [
        'rosa', 
        'l1', 
        'l1mst', 
        'mdcs', 
        'ours'
        ]

    for scan_index in scan_indices:
        # scan_gt = skel_dir + '\\art-scan-' + str(scan_index) + '-gt.obj'
        scan_file_name = skel_dir + '\\art-scan-' + str(scan_index) + '-uniform.npts'
        for compared_method in compared_methods:
            skel_file_name = skel_dir + '\\art-scan-' + str(scan_index) + '-' + compared_method + '.obj'
            temp_ri_file_name = skel_dir + '\\art-scan-' + str(scan_index) + '-' + compared_method + '.ri'
            tree_ply_file_name = skel_dir + '\\art-mesh-' + str(scan_index) + '-' + compared_method + '.ply'
            subprocess.call(('./radius_info_generation.exe %s %s %s' % (skel_file_name, scan_file_name, temp_ri_file_name)).split())
            subprocess.call(('./skeleton_based_tree_geometry_generation.exe %s %s %s' % (skel_file_name, temp_ri_file_name, tree_ply_file_name)).split())
            print('\n\n')
            pass
        pass
    pass


if __name__ == '__main__':
    run_recon()
    pass
