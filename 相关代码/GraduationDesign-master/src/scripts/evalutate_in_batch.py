import subprocess

def run_eval_mesh():
    print('Be sure that evaluate_mesh_result.exe exists in ../ ')
    skel_dir = 'D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result'
    raw_pc_indices = [1, 12, 13, 14, 15, 16]
    compared_methods = ['rosa', 'l1', 'l1mst', 'mdcs', 'ours']

    for raw_pc_index in raw_pc_indices:
        mesh_gt = skel_dir + '\\art-mesh-' + str(raw_pc_index) + '-gt.ply'
        for compared_method in compared_methods:
            mesh_file_name = skel_dir + '\\art-mesh-' + str(raw_pc_index) + '-' + compared_method + '.ply'
            subprocess.call(('../evaluate_mesh_result.exe %s %s' % (mesh_file_name, mesh_gt)).split())
            print('\n\n')
            pass
        pass
    pass

def run_eval_raw():
    print('Be sure that evaluate_result.exe exists in ../ ')
    skel_dir = 'D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result'
    raw_pc_indices = [4, 10, 17]
    compared_methods = ['rosa', 'l1', 'l1mst', 'mdcs', 'ours']

    for raw_pc_index in raw_pc_indices:
        raw_scan_gt = skel_dir + '\\raw-scan-' + str(raw_pc_index) + '-gt.obj'
        for compared_method in compared_methods:
            skel_file_name = skel_dir + '\\raw-scan-' + str(raw_pc_index) + '-' + compared_method + '.obj'
            subprocess.call(('../evaluate_result.exe %s %s' % (skel_file_name, raw_scan_gt)).split())
            print('\n\n')
            pass
        pass
    # raw_scan_4_ours = skel_dir + '\\raw-scan-4.obj'
    # raw_scan_4_l1 = skel_dir + '\\raw-scan-4-l1.obj'
    # raw_scan_4_l1mst = skel_dir + '\\raw-scan-4-l1mst.obj'
    # #-----------
    # raw_scan_10_gt = skel_dir + '\\raw-scan-10-gt.obj'
    # raw_scan_10_ours = skel_dir + '\\raw-scan-10.obj'
    # raw_scan_10_l1 = skel_dir + '\\raw-scan-10-l1.obj'
    # raw_scan_10_l1mst = skel_dir + '\\raw-scan-10-l1mst.obj'
    # #-----------
    # raw_scan_17_gt = skel_dir + '\\raw-scan-17-gt.obj'
    # raw_scan_17_ours = skel_dir + '\\raw-scan-17.obj'
    # raw_scan_17_l1 = skel_dir + '\\raw-scan-17-l1.obj'
    # raw_scan_17_l1mst = skel_dir + '\\raw-scan-17-l1mst.obj'
    # #============
    # # raw_scan_4_gt = skel_dir + '\\raw-scan-4-gt.obj'
    # # raw_scan_10_gt = skel_dir + '\\raw-scan-10-gt.obj'
    # # raw_scan_17_gt = skel_dir + '\\raw-scan-17-gt.obj'

    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_4_ours, raw_scan_4_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_4_l1, raw_scan_4_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_4_l1mst, raw_scan_4_gt)).split())

    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_10_ours, raw_scan_10_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_10_l1, raw_scan_10_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_10_l1mst, raw_scan_10_gt)).split())

    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_17_ours, raw_scan_17_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_17_l1, raw_scan_17_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_17_l1mst, raw_scan_17_gt)).split())
    pass

def run_eval_mve():
    print('Be sure that evaluate_result.exe exists in ../ ')
    skel_dir = 'D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result'
    raw_pc_indices = [2, 5, 8]
    compared_methods = ['rosa', 'l1', 'l1mst', 'mdcs', 'ours']

    for raw_pc_index in raw_pc_indices:
        raw_scan_gt = skel_dir + '\\mve-scan-' + str(raw_pc_index) + '-gt.obj'
        for compared_method in compared_methods:
            skel_file_name = skel_dir + '\\mve-scan-' + str(raw_pc_index) + '-' + compared_method + '.obj'
            subprocess.call(('../evaluate_result.exe %s %s' % (skel_file_name, raw_scan_gt)).split())
            print('\n\n')
            pass
        pass
    # mve_scan_2_gt = skel_dir + '\\mve-scan-2-gt.obj'
    # mve_scan_2_ours = skel_dir + '\\mve-scan-2.obj'
    # mve_scan_2_l1 = skel_dir + '\\mve-scan-2-l1.obj'
    # mve_scan_2_l1mst = skel_dir + '\\mve-scan-2-l1mst.obj'
    # #-----------
    # mve_scan_5_gt = skel_dir + '\\mve-scan-5-gt.obj'
    # mve_scan_5_ours = skel_dir + '\\mve-scan-5.obj'
    # mve_scan_5_l1 = skel_dir + '\\mve-scan-5-l1.obj'
    # mve_scan_5_l1mst = skel_dir + '\\mve-scan-5-l1mst.obj'
    # #-----------
    # mve_scan_8_gt = skel_dir + '\\mve-scan-8-gt.obj'
    # mve_scan_8_ours = skel_dir + '\\mve-scan-8.obj'
    # mve_scan_8_l1 = skel_dir + '\\mve-scan-8-l1.obj'
    # mve_scan_8_l1mst = skel_dir + '\\mve-scan-8-l1mst.obj'
    # #============
    # # raw_scan_4_gt = skel_dir + '\\mve-scan-4-gt.obj'
    # # raw_scan_10_gt = skel_dir + '\\mve-scan-10-gt.obj'
    # # raw_scan_17_gt = skel_dir + '\\mve-scan-17-gt.obj'

    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_2_ours, mve_scan_2_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_2_l1, mve_scan_2_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_2_l1mst, mve_scan_2_gt)).split())

    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_5_ours, mve_scan_5_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_5_l1, mve_scan_5_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_5_l1mst, mve_scan_5_gt)).split())

    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_8_ours, mve_scan_8_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_8_l1, mve_scan_8_gt)).split())
    # subprocess.call(('../evaluate_result.exe %s %s' % (mve_scan_8_l1mst, mve_scan_8_gt)).split())
    
    pass

if __name__ == '__main__':
    # run_eval_mve()
    run_eval_raw()
    #run_eval_mesh()
    pass