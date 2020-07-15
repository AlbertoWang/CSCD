import subprocess


def run_eval_raw():
    print('Be sure that evaluate_result.exe exists in ../ ')
    skel_dir = 'D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result'
    raw_pc_indices = [17]
    missing_percentages = ['05', '10', '15', '20', '25']

    for raw_pc_index in raw_pc_indices:
        raw_scan_gt = skel_dir + '\\raw-scan-' + str(raw_pc_index) + '-gt.obj'
        for missing_percentage in missing_percentages:
            raw_scan_skel_without_repair = skel_dir + '\\missing-raw-scan-' + str(raw_pc_index) + '-wor-' + missing_percentage +  '.obj'
            raw_scan_skel_with_repair = skel_dir + '\\missing-raw-scan-' + str(raw_pc_index) + '-rep-' + missing_percentage + '.obj'
            subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_skel_without_repair, raw_scan_gt)).split())
            print('\n\n')
            subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_skel_with_repair, raw_scan_gt)).split())
            print('\n\n')
            pass
        pass
    pass

def run_eval_mve():
    print('Be sure that evaluate_result.exe exists in ../ ')
    skel_dir = 'D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result'
    raw_pc_indices = [8]
    missing_percentages = ['05', '10', '15', '20', '25']

    for raw_pc_index in raw_pc_indices:
        raw_scan_gt = skel_dir + '\\mve-scan-' + str(raw_pc_index) + '-gt.obj'
        for missing_percentage in missing_percentages:
            raw_scan_skel_without_repair = skel_dir + '\\missing-mve-scan-' + str(raw_pc_index) + '-wor-' + missing_percentage +  '.obj'
            raw_scan_skel_with_repair = skel_dir + '\\missing-mve-scan-' + str(raw_pc_index) + '-rep-' + missing_percentage + '.obj'
            subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_skel_without_repair, raw_scan_gt)).split())
            print('\n\n')
            subprocess.call(('../evaluate_result.exe %s %s' % (raw_scan_skel_with_repair, raw_scan_gt)).split())
            print('\n\n')
            pass
        pass
    pass

if __name__ == '__main__':
    #run_eval_mve()
    run_eval_raw()
    pass