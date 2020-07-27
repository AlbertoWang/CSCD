#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <string>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "Eigen/Dense"
#include "polyscope/polyscope.h"
#include "polyscope/gl/ground_plane.h"
#include "polyscope/point_cloud.h"
#include "../io_xyz.hpp"
#include "../normalize_xyz.hpp"
#include "../string_utils.hpp"
#include "./eigen_utils.hpp"
#include "draw_models_impl.h"

using namespace std;
using Eigen::MatrixXf;
using Eigen::MatrixXf;

#define CONST_E		2.7182818284590452354	/* e */



using Graph = std::unordered_map<int, std::unordered_set<int> >;

struct BoundingBox {
    Point3d center;
    Vector3d extent;

};


namespace zjl {
    bool initializing = true;
    std::mutex render_mutex; 
    std::condition_variable cv;
};

ShowSkeletonConfig get_skel_config() {
    ShowSkeletonConfig skel_config;
    skel_config.show_node = false;
    skel_config.edge_radius = 0.00276f;
    skel_config.node_radius = 0.00345;
    skel_config.node_color_r = 49.0f / 255.0;
    skel_config.node_color_g = 217.0f / 255.0;
    skel_config.node_color_b = 27.0f / 255.0;
    return skel_config;
}

ShowPointCloudConfig get_pc_config() {
    ShowPointCloudConfig pc_config;
    pc_config.point_radius = 0.00191f;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    // 233, 233, 101
    return pc_config;
}

ShowMeshConfig get_mesh_config() {
    ShowMeshConfig config;
    config.mesh_color_r = 0.9f;
    config.mesh_color_g = 0.9f;
    config.mesh_color_b = 0.9f;
    return config;
}

// void render_point_cloud(const MatrixXf& point_cloud, ShowPointCloudConfig config) {
//     const char* data_key = "point cloud";
//     polyscope::init();
//     polyscope::registerPointCloud(data_key, point_cloud);
//     polyscope::PointCloud* pc = polyscope::getPointCloud(data_key);
//     pc->pointRadius = config.point_radius;
//     polyscope::gl::groundPlaneEnabled = config.enable_ground;
//     // ===
//     zjl::initializing = false;
//     zjl::cv.notify_all();
//     // ===
//     polyscope::show();
//     polyscope::shutdown();    
// }

void draw_six_scans() {
    std::array<float, 16> raw_4_camera_matrix = {
        0.1040, -0.1046, 0.9891, 0.0000, -0.0000, 0.9944, 0.1052, 0.0000, -0.9945, -0.0110, 0.1034, 0.0000, 0.0417, -0.1151, -1.8000, 1.0000 
    }; // raw-scan-4
    std::array<float, 16> raw_10_camera_matrix = {
        0.5141, -0.0313, -0.8572, 0.0000, 0.0000, 0.9993, -0.0365, 0.0000, 0.8577, 0.0188, 0.5138, 0.0000, 0.1875, 0.0904, -1.7310, 1.0000 
    }; 
    std::array<float, 16> raw_17_camera_matrix = {
        0.5141, -0.0313, -0.8572, 0.0000, 0.0000, 0.9993, -0.0365, 0.0000, 0.8577, 0.0188, 0.5138, 0.0000, 0.1875, 0.0904, -1.7310, 1.0000 
    }; 
    std::array<float, 16> mve_2_camera_matrix = {
        0.9929, -0.0392, -0.1117, 0.0000, 0.0000, 0.9434, -0.3316, 0.0000, 0.1183, 0.3293, 0.9368, 0.0000, 0.0418, 0.0298, -1.9110, 1.0000 
    }; 
    std::array<float, 16> mve_5_camera_matrix = {
        -0.4840, -0.0998, 0.8694, 0.0000, -0.0000, 0.9934, 0.1140, 0.0000, -0.8750, 0.0551, -0.4808, 0.0000, 0.0116, -0.0676, -1.8432, 1.0000 
    }; 
    std::array<float, 16> mve_8_camera_matrix = {
        -0.9999, 0.0035, 0.0112, 0.0000, 0.0000, 0.9532, -0.3022, 0.0000, -0.0117, -0.3022, -0.9532, 0.0000, 0.0650, 0.0293, -0.8384, 1.0000 
    }; 

    auto mve_scan_2_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mve_scan_2.npts"; 
    auto mve_scan_5_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mve_scan_5.npts"; 
    auto mve_scan_8_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mve_scan_8.npts"; 
    auto raw_scan_4_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\raw_scan_4.npts"; 
    auto raw_scan_10_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\raw_scan_10.npts"; 
    auto raw_scan_17_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\raw_scan_17.npts"; 
    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.00126f;
    
    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(mve_8_camera_matrix); 
    zjl::prepare_point_cloud_npts(mve_scan_8_path, pc_config, "point cloud");
    zjl::draw();
    zjl::deinit();

}

void draw_result_raw_scan_4() {
    std::array<float, 16> camera_matrix = {
        0.1040, -0.1046, 0.9891, 0.0000, -0.0000, 0.9944, 0.1052, 0.0000, -0.9945, -0.0110, 0.1034, 0.0000, 0.0417, -0.1151, -1.8000, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\raw_scan_4.npts"; 
    auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton-ours");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();
}

void draw_result_raw_scan_10() {
    std::array<float, 16> camera_matrix = {
        0.5141, -0.0313, -0.8572, 0.0000, 0.0000, 0.9993, -0.0365, 0.0000, 0.8577, 0.0188, 0.5138, 0.0000, 0.1875, 0.0904, -1.7310, 1.0000 
    }; // raw-scan-10
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\raw_scan_10.npts"; 
    auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-10-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-10-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-10-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-10-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-10-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-10-gt.obj"; 
    // skel_path = gt_skel_path;

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton-ours");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_mdcs, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();
}

void draw_result_mve_scan_2() {
    std::array<float, 16> camera_matrix = {
        0.9929, -0.0392, -0.1117, 0.0000, 0.0000, 0.9434, -0.3316, 0.0000, 0.1183, 0.3293, 0.9368, 0.0000, 0.0418, 0.0298, -1.9110, 1.0000 
    }; // mve-scan-8
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mve_scan_2.npts"; 
    auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-2-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-2-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-2-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-2-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-2-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-2-gt.obj"; 
    // skel_path = gt_skel_path;

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    pc_config.point_radius = 0.0007f;

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_mdcs, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();
}

void draw_result_mve_scan_5() {
    std::array<float, 16> camera_matrix = {
        -0.4840, -0.0998, 0.8694, 0.0000, -0.0000, 0.9934, 0.1140, 0.0000, -0.8750, 0.0551, -0.4808, 0.0000, 0.0116, -0.0676, -1.8432, 1.0000 
    }; // mve-scan-8
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mve_scan_5.npts"; 
    auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-5-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-5-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-5-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-5-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-5-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-5-gt.obj"; 
    // skel_path = gt_skel_path;

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    pc_config.point_radius = 0.00092f;

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_mdcs, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();
}

void draw_result_mve_scan_8() {
    std::array<float, 16> camera_matrix = {
        -0.9999, 0.0035, 0.0112, 0.0000, 0.0000, 0.9532, -0.3022, 0.0000, -0.0117, -0.3022, -0.9532, 0.0000, 0.0650, 0.0293, -0.8384, 1.0000 
    }; // mve-scan-8
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mve_scan_8.npts"; 
    auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-8-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-8-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-8-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-8-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-8-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\mve-scan-8-gt.obj"; 
    // skel_path = gt_skel_path;

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    pc_config.point_radius = 0.00092f;

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton-ours");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_mdcs, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();
}

void draw_result_raw_scan_17() {
    std::array<float, 16> camera_matrix = {
        0.5141, -0.0313, -0.8572, 0.0000, 0.0000, 0.9993, -0.0365, 0.0000, 0.8577, 0.0188, 0.5138, 0.0000, 0.1875, 0.0904, -1.7310, 1.0000 
    }; // raw-scan-10
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\raw_scan_17.npts"; 
    auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-gt.obj"; 
    // skel_path = gt_skel_path;

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_mdcs, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();
}

void draw_art_scan_1() {
    std::array<float, 16> camera_matrix = {
        0.5141, -0.0313, -0.8572, 0.0000, 0.0000, 0.9993, -0.0365, 0.0000, 0.8577, 0.0188, 0.5138, 0.0000, 0.1875, 0.0904, -1.7310, 1.0000 
    }; // raw-scan-10
    auto mesh_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mesh_1\\tree_small_1.ply"; 
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\scan_npts_noise_1\\tree_small_1_res_250.npts"; 
    // auto skel_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17.obj"; 
    // auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-l1.obj"; 
    // auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-l1mst.obj"; 
    // auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-17-rosa.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\mesh_skel_1\\tree_small_1.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();

    zjl::init();
    zjl::disable_ground();
    // zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    // zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    // zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    // zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    //zjl::prepare_mesh_ply(mesh_path, "mesh");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    // zjl::set_camera_from_16_floats(camera_matrix);
    zjl::draw();
    zjl::deinit();

}

void draw_nomralization() {
    auto pc_1 = "D:\\User\\zhouh\\Documents\\CQU_SKELETON\\Data\\2017.10LVP\\tree9.txt"; 
    std::vector<Point3d> points;
    zjl::IO::read_point_cloud_from_npts_without_normals(pc_1, points);
    auto pp = zjl::Normalizer::normalize<Point3d>(points);
    auto pp2 = zjl::Normalizer::normalize<Point3d>(points);

    ShowPointCloudConfig config;
    config.point_radius = 0.00025;

    zjl::init();
    zjl::prepare_xyz_axis(8.0f);
    zjl::prepare_box(pp.first, pp.second, "bb-raw");
    zjl::prepare_box(pp2.first, pp2.second, "bb-n");
    zjl::prepare_point_cloud_npts(pc_1, config, "raw");
    zjl::prepare_point_cloud_npts_fromdata(points, config, "np");
    zjl::draw();
    zjl::deinit();
}

void draw_global_branch_contraction() {
    auto sample_init = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-init.npts";
    auto sample_iter_4 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-iter-4.npts";
    auto sample_iter_8 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-iter-8.npts";
    auto sample_iter_16 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-iter-16.npts";
    auto sample_iter_24 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-iter-24.npts";
    // std::string branch_3_iter_0 = "D:\\z\\branch-3-iter-0.npts";
    // std::string branch_3_iter_1 = "D:\\z\\branch-3-iter-2.npts";
    // std::string branch_3_iter_2 = "D:\\z\\branch-3-iter-4.npts";

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.00217;
    ShowPointCloudConfig skel_config;
    skel_config.point_color_r = 0.1f;
    skel_config.point_color_g = 0.3f;
    skel_config.point_color_b = 0.9f;

    zjl::init();
    zjl::disable_ground();
    //zjl::prepare_point_cloud_npts(sample_init, pc_config, "b1-pc");
    zjl::prepare_point_cloud_npts(sample_init, skel_config, "b1-pc");
    zjl::prepare_point_cloud_npts(sample_iter_4, skel_config, "b1-i0");
    zjl::prepare_point_cloud_npts(sample_iter_8, skel_config, "b1-i1");
    zjl::prepare_point_cloud_npts(sample_iter_16, skel_config, "b1-i2");
    zjl::prepare_point_cloud_npts(sample_iter_24, skel_config, "b1-i3");
    zjl::draw();
    zjl::deinit();

}

void draw_single_branch_contraction_3() {
    auto branch_3_pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\branch_segment_3.npts";
    std::string branch_3_iter_0 = "D:\\z\\branch-3-iter-0.npts";
    std::string branch_3_iter_1 = "D:\\z\\branch-3-iter-2.npts";
    std::string branch_3_iter_2 = "D:\\z\\branch-3-iter-4.npts";
    std::string branch_3_iter_3 = "D:\\z\\branch-3-iter-6.npts";

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    ShowPointCloudConfig skel_config;
    skel_config.point_color_r = 0.1f;
    skel_config.point_color_g = 0.3f;
    skel_config.point_color_b = 0.9f;

    zjl::init();
    zjl::prepare_point_cloud_npts(branch_3_pc, pc_config, "b1-pc");
    zjl::prepare_point_cloud_npts(branch_3_iter_0, skel_config, "b1-i0");
    zjl::prepare_point_cloud_npts(branch_3_iter_1, skel_config, "b1-i1");
    zjl::prepare_point_cloud_npts(branch_3_iter_2, skel_config, "b1-i2");
    zjl::prepare_point_cloud_npts(branch_3_iter_3, skel_config, "b1-i3");
    zjl::draw();
    zjl::deinit();
}

void draw_recenter_skel_comparison() {
    auto pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample_contraction.npts";
    auto skel_1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter-raw-skel.obj";
    auto skel_2 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter-new-skel.obj";
    // std::vector<Point3d> old_center = {{-0.0432999, 0.07437, -0.02917}};
    // std::vector<Point3d> new_center = {{+0.0332999, 0.07437, -0.02917}};

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0012;
    // 0.0012
    ShowSkeletonConfig skeleton_config;
    // For global view:
    // skeleton_config.edge_radius = 0.00400f;
    // skeleton_config.node_radius = 0.006f;
    // For highlight view:
    skeleton_config.edge_radius = 0.00126;
    skeleton_config.node_radius = 0.00345f;
    skeleton_config.show_node = true;

    ShowMeshConfig mesh_config;

    zjl::init();
    zjl::disable_ground();
    // zjl::set_camera_from_16_floats(camera_params);
    zjl::prepare_point_cloud_npts(pc, pc_config, "b1-pc");
    zjl::prepare_skeleton_obj(skel_1, skeleton_config, "skel1");
    zjl::prepare_skeleton_obj(skel_2, skeleton_config, "skel2");
    // zjl::prepare_point_cloud_npts(branch_1_iter_0, skel_config, "b1-i0");
    zjl::draw();
    zjl::deinit();

}

void draw_anistropic_illustration() {
    //auto pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\anistropic_uniform.npts";
    auto pc2 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\anistropic_non_uniform.npts";
    auto pc2_in_circle = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\anistropic_non_uniform_in_circle.npts";
    auto pc2_out_of_circle = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\anistropic_non_uniform_outof_circle.npts";
    std::vector<Point3d> center = {{0.4869, 1.0047, 0.0f}};

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.008;
    // 0.00064 -> highlight-mode

    ShowPointCloudConfig pc_config_2;
    pc_config_2.point_color_r = 0.1;
    pc_config_2.point_color_g = 0.9;
    pc_config_2.point_color_b = 0.3;
    pc_config_2.point_radius = 0.008;

    ShowPointCloudConfig center_config;
    center_config.point_color_r = 0.9;
    center_config.point_color_g = 0.1;
    center_config.point_color_b = 0.0;
    center_config.point_radius = 0.008;

    zjl::init();
    zjl::disable_ground();
    //zjl::prepare_point_cloud_npts(pc2, pc_config_2, "pc");
    zjl::prepare_point_cloud_npts(pc2_in_circle, pc_config_2, "pc-inc");
    zjl::prepare_point_cloud_npts(pc2_out_of_circle, pc_config, "pc-oc");
    zjl::prepare_point_cloud_npts_fromdata(center, center_config, "center");
    zjl::draw();
    zjl::deinit();

}

void draw_furthest_sample_illustration2() {
    // std::array<float, 16> camera_params = {
    //     -0.9862, 0.0089, 0.1653, 0.0000, 0.0000, 0.9985, -0.0539, 0.0000, -0.1656, -0.0531, -0.9848, 0.0000, -0.2701, 0.2161, -0.1669, 1.0000 
    // };
    auto pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\furthest_sample_dbscan_branch.npts";
    auto pc2 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\furthest_sample_dbscan_center.npts";

    auto dbscan1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\dbscan_1.npts";
    auto dbscan2 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\dbscan_2.npts";
    auto dbscan3 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\dbscan_3.npts";
    auto dbscan4 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\dbscan_4.npts";
    auto dbscan_skel = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\dbscan_connect.obj";
    //auto skel_1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\furthest_sample_skel.obj";

    std::vector<Point3d> bridge_points = {
        {-0.422357, -0.5877, -0.14088},
        {-0.288279, -0.46818, -0.09784}
    };

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 0.1;
    pc_config.point_color_g = 0.3;
    pc_config.point_color_b = 0.9;
    pc_config.point_radius = 0.002;
    // 0.00064 -> highlight-mode

    ShowPointCloudConfig pc_config_2;
    pc_config_2.point_color_r = 0.1;
    pc_config_2.point_color_g = 0.9;
    pc_config_2.point_color_b = 0.3;
    pc_config_2.point_radius = 0.002;

    ShowSkeletonConfig skel_config;
    skel_config.show_node = true;
    skel_config.node_radius = 0.0027;
    skel_config.edge_radius = 0.0011;

    ShowPointCloudConfig bridge_config_2;
    bridge_config_2.point_color_r = 0.1;
    bridge_config_2.point_color_g = 0.9;
    bridge_config_2.point_color_b = 0.3;
    bridge_config_2.point_radius = 0.0035;

    zjl::init();
    zjl::disable_ground();
    //zjl::set_camera_from_16_floats(camera_params);

    zjl::prepare_point_cloud_npts(pc, pc_config, "b1-pc");
    zjl::prepare_point_cloud_npts(pc2, pc_config_2, "b2-pc");

    // zjl::prepare_point_cloud_npts(dbscan1, pc_config, "dbscan1");
    // zjl::prepare_point_cloud_npts(dbscan2, pc_config, "dbscan2");
    // zjl::prepare_point_cloud_npts(dbscan3, pc_config, "dbscan3");
    // zjl::prepare_point_cloud_npts(dbscan4, pc_config, "dbscan4");
    zjl::prepare_skeleton_obj(dbscan_skel, skel_config, "dbscan-skel");
    zjl::prepare_point_cloud_npts_fromdata(bridge_points, bridge_config_2, "bridge-pts");

    zjl::draw();
    zjl::deinit();
}

void draw_furthest_sample_illustration1() {
    std::array<float, 16> camera_params = {
        -0.9862, 0.0089, 0.1653, 0.0000, 0.0000, 0.9985, -0.0539, 0.0000, -0.1656, -0.0531, -0.9848, 0.0000, -0.2701, 0.2161, -0.1669, 1.0000 
    };
    auto pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\furthest_sample_pc.npts";
    auto skel_1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\furthest_sample_skel.obj";

    std::vector<Point3d> skel_points_1 = {{-0.256368, -0.238813, -0.09218}};
    std::vector<Edge> skel_edges_1;

    std::vector<Point3d> skel_points_2 = {
        {-0.256368, -0.238813, -0.09218}, // middle point
        {-0.253836, -0.20428, -0.101147}, // upper point
        {-0.261216, -0.272386, -0.087416}  // bottom point
    };
    std::vector<Edge> skel_edges_2 = {{0, 1}, {0, 2}};

    std::vector<Point3d> skel_points_3 = {
        {-0.254500, -0.17507, -0.11301}, // upper point
        {-0.253836, -0.20428, -0.101147}, // upper point
        {-0.256368, -0.238813, -0.09218}, // middle point
        {-0.261216, -0.272386, -0.087416},  // bottom point
        {-0.268279, -0.307891, -0.084908}  // bottom point
    };
    std::vector<Edge> skel_edges_3 = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}};

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 0.1;
    pc_config.point_color_g = 0.3;
    pc_config.point_color_b = 0.9;
    pc_config.point_radius = 0.00035;
    // 0.00064 -> highlight-mode

    ShowPointCloudConfig pc_config_2;
    pc_config_2.point_color_r = 0.1;
    pc_config_2.point_color_g = 0.9;
    pc_config_2.point_color_b = 0.3;
    pc_config_2.point_radius = 0.0012;

    ShowSkeletonConfig skel_config;
    skel_config.show_node = true;
    skel_config.node_radius = 0.0027;
    skel_config.edge_radius = 0.0014;

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_params);
    zjl::prepare_point_cloud_npts(pc, pc_config, "b1-pc");
    //zjl::prepare_point_cloud_npts(skel, pc_config_2, "b2-pc");
    zjl::prepare_skeleton_obj(skel_1, skel_config, "skel");
    zjl::prepare_skeleton_from_data(skel_points_1, skel_edges_1, skel_config, "1-point");
    zjl::prepare_skeleton_from_data(skel_points_2, skel_edges_2, skel_config, "3-point");
    zjl::prepare_skeleton_from_data(skel_points_3, skel_edges_3, skel_config, "5-point");
    zjl::draw();
    zjl::deinit();
}

void draw_mls_comparison() {
    auto pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-iter-24.npts";
    auto pc2 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\mls-result-new2.npts";

    ShowPointCloudConfig pc_config;
    // pc_config.point_color_r = 233.0f / 255.0f;
    // pc_config.point_color_g = 233.0f / 255.0f;
    // pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_color_r = 0.1;
    pc_config.point_color_g = 0.3;
    pc_config.point_color_b = 0.9;
    pc_config.point_radius = 0.00246;
    // 0.00064 -> highlight-mode

    ShowPointCloudConfig pc_config_2;
    pc_config_2.point_color_r = 0.1;
    pc_config_2.point_color_g = 0.9;
    pc_config_2.point_color_b = 0.3;
    pc_config_2.point_radius = 0.0012;

    zjl::init();
    zjl::disable_ground();
    zjl::prepare_point_cloud_npts(pc, pc_config, "b1-pc");
    zjl::prepare_point_cloud_npts(pc2, pc_config_2, "b2-pc");
    zjl::draw();
    zjl::deinit();

}

void draw_recenter_illustration() {
    // -0.6374 0.6710 -0.3787 0.0000 -0.0000 0.4915 0.8709 0.0000 0.7705 0.5551 -0.3133 0.0000 0.1631 -0.1060 -1.0416 1.0000 
    std::array<float, 16> camera_params = {
        -0.6374, 0.6710, -0.3787, 0.0000, -0.0000, 0.4915, 0.8709, 0.0000, 0.7705, 0.5551, -0.3133, 0.0000, 0.1631, -0.1060, -1.0416, 1.0000 
    };

    auto branch_1_pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter_branch_points.npts";
    auto branch_1_pc_in = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter_points_in_cylinder.npts";
    auto branch_1_pc_out = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter_points_outof_cylinder.npts";
    auto branch_1_skel = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter_branch_skel.obj";
    auto cylinder = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\recenter_cylinder.ply";
    std::vector<Point3d> old_center = {{-0.0432999, 0.07437, -0.02917}};
    std::vector<Point3d> new_center = {{+0.0332999, 0.07437, -0.02917}};

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    ShowPointCloudConfig pc2_config;
    pc2_config.point_color_r = 0.1f;
    pc2_config.point_color_g = 0.3f;
    pc2_config.point_color_b = 0.9f;
    ShowPointCloudConfig skeleton_center_config;
    skeleton_center_config.point_color_r = 1.0f;
    skeleton_center_config.point_color_g = 0.0f;
    skeleton_center_config.point_color_b = 0.0f;
    skeleton_center_config.point_radius = 0.010f;
    ShowPointCloudConfig new_skeleton_center_config;
    new_skeleton_center_config.point_color_r = 0.0f;
    new_skeleton_center_config.point_color_g = 1.0f;
    new_skeleton_center_config.point_color_b = 0.0f;
    new_skeleton_center_config.point_radius = 0.010f;
    ShowSkeletonConfig skeleton_config;
    skeleton_config.edge_radius = 0.0075f;
    skeleton_config.node_radius = 0.014f;
    skeleton_config.show_node = true;

    ShowMeshConfig mesh_config;

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_params);
    zjl::prepare_point_cloud_npts(branch_1_pc_in, pc2_config, "b1-pc");
    zjl::prepare_point_cloud_npts(branch_1_pc_out, pc_config, "b2-pc");
    zjl::prepare_mesh_ply(cylinder, mesh_config, "cylinder");
    zjl::prepare_skeleton_obj(branch_1_skel, skeleton_config, "skel");
    zjl::prepare_point_cloud_npts_fromdata(old_center, skeleton_center_config, "old_center");
    zjl::prepare_point_cloud_npts_fromdata(new_center, new_skeleton_center_config, "new_center");
    // zjl::prepare_point_cloud_npts(branch_1_iter_0, skel_config, "b1-i0");
    zjl::draw();
    zjl::deinit();

}

void draw_single_branch_contraction_1() {
    auto branch_1_pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\branch_segment_1.npts";
    std::string branch_1_iter_0 = "D:\\z\\branch-1-iter-0.npts";
    std::string branch_1_iter_1 = "D:\\z\\branch-1-iter-2.npts";
    std::string branch_1_iter_2 = "D:\\z\\branch-1-iter-4.npts";
    std::string branch_1_iter_3 = "D:\\z\\branch-1-iter-6.npts";

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    ShowPointCloudConfig skel_config;
    skel_config.point_color_r = 0.1f;
    skel_config.point_color_g = 0.3f;
    skel_config.point_color_b = 0.9f;

    zjl::init();
    zjl::prepare_point_cloud_npts(branch_1_pc, pc_config, "b1-pc");
    zjl::prepare_point_cloud_npts(branch_1_iter_0, skel_config, "b1-i0");
    zjl::prepare_point_cloud_npts(branch_1_iter_1, skel_config, "b1-i1");
    zjl::prepare_point_cloud_npts(branch_1_iter_2, skel_config, "b1-i2");
    zjl::prepare_point_cloud_npts(branch_1_iter_3, skel_config, "b1-i3");
    zjl::draw();
    zjl::deinit();
}

void draw_wlop_comparison() {
    auto wlop_0 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\wlop_data.npts";
    auto wlop_3 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\wlop_data_iter_3.npts";
    auto wlop_6 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\wlop_data_iter_6.npts";
    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0071;
    ShowPointCloudConfig r1_config;
    r1_config.point_color_r = 0.1f;
    r1_config.point_color_g = 0.3f;
    r1_config.point_color_b = 0.9f;
    // ShowPointCloudConfig r2_config;
    // r2_config.point_color_r = 0.3f;
    // r2_config.point_color_g = 0.9f;
    // r2_config.point_color_b = 0.1f;
    zjl::init();
    zjl::prepare_point_cloud_npts(wlop_0, pc_config, "pc");
    zjl::prepare_point_cloud_npts(wlop_3, pc_config, "pc1");
    zjl::prepare_point_cloud_npts(wlop_6, pc_config, "pc2");
    zjl::draw();
    zjl::deinit();
}

void draw_anistropic_comparison() {
    auto branch_pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\Branch_4_Anis_Non_Uniform.npts";
    auto result_with_anis = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\With_Anis.npts";
    auto result_without_anis = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\Without_Anis.npts";
    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    ShowPointCloudConfig r1_config;
    r1_config.point_color_r = 0.1f;
    r1_config.point_color_g = 0.3f;
    r1_config.point_color_b = 0.9f;
    ShowPointCloudConfig r2_config;
    r2_config.point_color_r = 0.3f;
    r2_config.point_color_g = 0.9f;
    r2_config.point_color_b = 0.1f;
    zjl::init();
    zjl::prepare_point_cloud_npts(branch_pc, pc_config, "pc");
    zjl::prepare_point_cloud_npts(result_with_anis, r1_config, "w-anis");
    zjl::prepare_point_cloud_npts(result_without_anis, r2_config, "w/o-anis");
    zjl::draw();
    zjl::deinit();
}

void draw_skeleton_repair() {
    std::array<float, 16> camera_params = {
        -1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, -1.0000, 0.0000, -0.2286, 0.0815, -2.1010, 1.0000 
    };
    auto pc_missing = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\skel_missing_2_pc.npts";
    auto skel_missing = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\skel_missing_2.obj";
    auto skel_missing_repaired = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\skel_missing_2_repair.obj";
    // std::vector<Point3d> old_center = {{-0.0432999, 0.07437, -0.02917}};
    // std::vector<Point3d> new_center = {{+0.0332999, 0.07437, -0.02917}};
    std::vector<Point3d> ngps = {
        {-0.0072, -0.4235, -0.0111},   
        {-0.0466, -0.2815, -0.0993},   
        {-0.2693, -0.3078, -0.0849},   
    };

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0025;

    ShowPointCloudConfig ngp_config;
    ngp_config.point_color_r = 0.1f;
    ngp_config.point_color_g = 0.9f;
    ngp_config.point_color_b = 0.3f;
    ngp_config.point_radius = 0.0026;

    ShowSkeletonConfig skeleton_config;
    skeleton_config.node_radius = 0.00565f;
    skeleton_config.edge_radius = 0.00276;
    skeleton_config.show_node = true;

    ShowSkeletonConfig skeleton_config_2;
    skeleton_config_2.node_radius = 0.00665f;
    skeleton_config_2.edge_radius = 0.00266;
    skeleton_config_2.edge_color_r = 0.1f;
    skeleton_config_2.edge_color_g = 0.9f;
    skeleton_config_2.edge_color_b = 0.3f;
    skeleton_config_2.show_node = true;

    //ShowMeshConfig mesh_config;

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_params);
    zjl::prepare_point_cloud_npts(pc_missing, pc_config, "pc");
    zjl::prepare_point_cloud_npts_fromdata(ngps, ngp_config, "ngp");
    zjl::prepare_skeleton_obj(skel_missing, skeleton_config, "skel");
    zjl::prepare_skeleton_obj(skel_missing_repaired, skeleton_config_2, "skel2");
    ///zjl::prepare_point_cloud_npts_fromdata(old_center, skeleton_center_config, "old_center");
    // zjl::prepare_point_cloud_npts(branch_1_iter_0, skel_config, "b1-i0");
    zjl::draw();
    zjl::deinit();

}

void draw_cardinal_illustration() {
    std::array<float, 16> camera_params = {
        -1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, -1.0000, 0.0000, 0.1687, 0.1659, -1.0495, 1.0000 
    };
    auto non_uniform_skel = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cardinal_non_uniform_skel.obj";
    auto non_uniform_skel_no_middle = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cardinal_non_uniform_skel_no_middle_edge.obj";
    auto uniform_skel = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cardinal_uniform_skel.obj";
    auto interpolated_points_n_5_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cardinal_n_0.5.npts";
    auto interpolated_points_zero_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cardinal_zero.npts";
    auto interpolated_points_p_5_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cardinal_p_0.5.npts";
    
    // std::vector<Point3d> old_center = {{-0.0432999, 0.07437, -0.02917}};
    // std::vector<Point3d> new_center = {{+0.0332999, 0.07437, -0.02917}};
    std::vector<Point3d> control_points = {
        {0.20297, -1.0727, -0.0},   
        {-0.08907, -0.53758, -0.0},   
        {0.11387, 0.27606, -0.0},   
        {0.58418, 0.72512, -0.0}   
    };

    ShowPointCloudConfig pc_config;
    // pc_config.point_color_r = 233.0f / 255.0f;
    // pc_config.point_color_g = 233.0f / 255.0f;
    // pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_color_r = 0.1f;
    pc_config.point_color_g = 0.9f;
    pc_config.point_color_b = 0.3f;
    pc_config.point_radius = 0.0025;

    ShowPointCloudConfig ngp_config;
    ngp_config.point_color_r = 0.9f;
    ngp_config.point_color_g = 0.9f;
    ngp_config.point_color_b = 0.9f;
    ngp_config.point_radius = 0.0026;

    ShowSkeletonConfig skeleton_config;
    skeleton_config.node_radius = 0.01006;
    skeleton_config.edge_radius = 0.00468;
    skeleton_config.show_node = true;

    ShowSkeletonConfig skeleton_config_2;
    skeleton_config_2.node_radius = 0.01006;
    skeleton_config_2.edge_radius = 0.00468;
    skeleton_config_2.edge_color_r = 0.1f;
    skeleton_config_2.edge_color_g = 0.9f;
    skeleton_config_2.edge_color_b = 0.3f;
    skeleton_config_2.show_node = true;

    ShowSkeletonConfig inter_skel_config;
    inter_skel_config.edge_radius = 0.00266;
    inter_skel_config.node_radius = 0.00266;
    inter_skel_config.edge_color_r = 0.1f;
    inter_skel_config.edge_color_g = 0.9f;
    inter_skel_config.edge_color_b = 0.3f;
    inter_skel_config.show_node = false;

    //ShowMeshConfig mesh_config;

    std::vector<Point3d> interpolated_points_n_5;
    zjl::IO::read_point_cloud_from_npts_without_normals(interpolated_points_n_5_path, interpolated_points_n_5);
    std::vector<Edge> node_indices;
    for (int i = 0; i + 1 < interpolated_points_n_5.size(); i ++) {
        node_indices.push_back({i, i + 1});
    }

    std::vector<Point3d> interpolated_points_p_5;
    zjl::IO::read_point_cloud_from_npts_without_normals(interpolated_points_p_5_path, interpolated_points_p_5);
    std::vector<Edge> edges_p_5;
    for (int i = 0; i + 1 < interpolated_points_p_5.size(); i ++) {
        node_indices.push_back({i, i + 1});
    }

    std::vector<Point3d> interpolated_points_zero;
    zjl::IO::read_point_cloud_from_npts_without_normals(interpolated_points_zero_path, interpolated_points_zero);
    std::vector<Edge> edges_zero;
    for (int i = 0; i + 1 < interpolated_points_zero.size(); i ++) {
        node_indices.push_back({i, i + 1});
    }

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_params);
    //zjl::prepare_point_cloud_npts(interpolated_points_n_5, pc_config, "pc -0.5");
    zjl::prepare_point_cloud_npts_fromdata(control_points, ngp_config, "ngp");
    zjl::prepare_skeleton_obj(non_uniform_skel, skeleton_config, "skel");
    zjl::prepare_skeleton_obj(non_uniform_skel_no_middle, skeleton_config, "skel-nm");
    zjl::prepare_skeleton_obj(uniform_skel, skeleton_config_2, "skel2");
    zjl::prepare_skeleton_from_data(interpolated_points_n_5, node_indices, inter_skel_config, "inter-config - 0.5");
    zjl::prepare_skeleton_from_data(interpolated_points_p_5, edges_p_5, inter_skel_config, "inter-config + 0.5");
    zjl::prepare_skeleton_from_data(interpolated_points_zero, edges_zero, inter_skel_config, "inter-config - 0.0");
    ///zjl::prepare_point_cloud_npts_fromdata(old_center, skeleton_center_config, "old_center");
    // zjl::prepare_point_cloud_npts(branch_1_iter_0, skel_config, "b1-i0");
    zjl::draw();
    zjl::deinit();
}

void draw_octree_illustration() {
    //0.8026 -0.0642 0.5930 0.0000 0.0000 0.9942 0.1076 0.0000 -0.5965 -0.0863 0.7980 0.0000 0.0727 -0.2783 -2.4473 1.0000 
    std::array<float, 16> camera_params = {
        0.8026, -0.0642, 0.5930, 0.0000, 0.0000, 0.9942, 0.1076, 0.0000, -0.5965, -0.0863, 0.7980, 0.0000, 0.0727, -0.2783, -2.4473, 1.0000 
    };

    auto point_cloud_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-init.npts";
    auto cell_points_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\octree_cell_subset.npts";
    auto octree_wireframe_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\octree_wireframe.obj";
    auto cell_wireframe_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\octree_cell_wireframe.obj";
    auto cell_division_wireframe_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\octree_cell_subdivision_wireframe.obj";

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0019;

    ShowSkeletonConfig skeleton_config;
    skeleton_config.node_radius = 0.00424;
    skeleton_config.edge_radius = 0.00146;
    skeleton_config.show_node = true;

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_params);
    //zjl::prepare_point_cloud_npts_fromdata(control_points, ngp_config, "ngp");
    zjl::prepare_point_cloud_npts(point_cloud_path, pc_config, "pc");
    zjl::prepare_point_cloud_npts(cell_points_path, pc_config, "pc-ss");
    zjl::prepare_skeleton_obj(octree_wireframe_path, skeleton_config, "octree");
    zjl::prepare_skeleton_obj(cell_wireframe_path, skeleton_config, "octree-cell");
    zjl::prepare_skeleton_obj(cell_division_wireframe_path, skeleton_config, "octree-d-cell");
    //zjl::prepare_skeleton_obj(non_uniform_skel_no_middle, skeleton_config, "skel-nm");
    zjl::draw();
    zjl::deinit();

}

void draw_result_art_scan_16() {
    std::array<float, 16> camera_matrix = {
        -0.8591, 0.1086, 0.5001, 0.0000, -0.0000, 0.9773, -0.2122, 0.0000, -0.5117, -0.1823, -0.8395, 0.0000, 0.1095, 0.0171, -1.8529, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\art_scan_16.npts"; 
    auto mesh_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-16-gt.ply"; 
    auto mesh_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-16-rosa.ply"; 
    auto mesh_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-16-l1.ply"; 
    auto mesh_path_l1mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-16-l1mst.ply"; 
    auto mesh_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-16-mdcs.ply"; 
    auto mesh_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-16-ours.ply"; 
    // ===
    auto skel_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-4-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-rosa.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    auto mesh_config = get_mesh_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_matrix);
    // zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    // zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    // zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    // zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    // zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    //zjl::prepare_mesh_ply(mesh_path_rosa, mesh_config, "mesh-rosa");
    //zjl::prepare_mesh_ply(mesh_path_l1, mesh_config, "mesh-l1");
    //zjl::prepare_mesh_ply(mesh_path_l1mst, mesh_config, "mesh-l1mst");
    //zjl::prepare_mesh_ply(mesh_path_mdcs, mesh_config, "mesh-mdcs");
    //zjl::prepare_mesh_ply(mesh_path_ours, mesh_config, "mesh-ours");
    //zjl::prepare_mesh_ply(mesh_path_gt, mesh_config, "mesh-gt");
    zjl::draw();
    zjl::deinit();
}

void draw_result_art_scan_15() {
    std::array<float, 16> camera_matrix = {
        -0.8282, 0.1460, 0.5410, 0.0000, 0.0000, 0.9655, -0.2605, 0.0000, -0.5603, -0.2158, -0.7996, 0.0000, 0.1075, 0.0151, -1.8541, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\art_scan_15.npts"; 
    auto mesh_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-15-gt.ply"; 
    auto mesh_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-15-rosa.ply"; 
    auto mesh_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-15-l1.ply"; 
    auto mesh_path_l1mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-15-l1mst.ply"; 
    auto mesh_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-15-mdcs.ply"; 
    auto mesh_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-15-ours.ply"; 
    // ===
    auto skel_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-4-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-rosa.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    auto mesh_config = get_mesh_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_matrix);
    // zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    // zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    // zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    // zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    // zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    //zjl::prepare_mesh_ply(mesh_path_rosa, mesh_config, "mesh-rosa");
    //zjl::prepare_mesh_ply(mesh_path_l1, mesh_config, "mesh-l1");
    //zjl::prepare_mesh_ply(mesh_path_l1mst, mesh_config, "mesh-l1mst");
    //zjl::prepare_mesh_ply(mesh_path_mdcs, mesh_config, "mesh-mdcs");
    //zjl::prepare_mesh_ply(mesh_path_ours, mesh_config, "mesh-ours");
    //zjl::prepare_mesh_ply(mesh_path_gt, mesh_config, "mesh-gt");
    zjl::draw();
    zjl::deinit();
}

void draw_result_art_scan_14() {
    std::array<float, 16> camera_matrix = {
        0.8442, -0.1267, -0.5208, 0.0000, 0.0000, 0.9717, -0.2364, 0.0000, 0.5359, 0.1996, 0.8203, 0.0000, 0.0902, -0.0556, -1.8124, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\art_scan_14.npts"; 
    auto mesh_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-14-gt.ply"; 
    auto mesh_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-14-rosa.ply"; 
    auto mesh_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-14-l1.ply"; 
    auto mesh_path_l1mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-14-l1mst.ply"; 
    auto mesh_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-14-mdcs.ply"; 
    auto mesh_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-14-ours.ply"; 
    // ===
    auto skel_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-4-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-rosa.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    auto mesh_config = get_mesh_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_matrix);
    // zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    // zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    // zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    // zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    // zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    //zjl::prepare_mesh_ply(mesh_path_rosa, mesh_config, "mesh-rosa");
    //zjl::prepare_mesh_ply(mesh_path_l1, mesh_config, "mesh-l1");
    //zjl::prepare_mesh_ply(mesh_path_l1mst, mesh_config, "mesh-l1mst");
    //zjl::prepare_mesh_ply(mesh_path_mdcs, mesh_config, "mesh-mdcs");
    //zjl::prepare_mesh_ply(mesh_path_ours, mesh_config, "mesh-ours");
    //zjl::prepare_mesh_ply(mesh_path_gt, mesh_config, "mesh-gt");
    zjl::draw();
    zjl::deinit();
}

void draw_result_art_scan_13() {
    std::array<float, 16> camera_matrix = {
        -0.2114, 0.2034, 0.9560, 0.0000, -0.0000, 0.9781, -0.2081, 0.0000, -0.9773, -0.0440, -0.2068, 0.0000, 0.0673, -0.1191, -2.4981, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\art_scan_13.npts"; 
    auto mesh_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-13-gt.ply"; 
    auto mesh_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-13-rosa.ply"; 
    auto mesh_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-13-l1.ply"; 
    auto mesh_path_l1mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-13-l1mst.ply"; 
    auto mesh_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-13-mdcs.ply"; 
    auto mesh_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-13-ours.ply"; 
    // ===
    auto skel_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-4-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-rosa.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    auto mesh_config = get_mesh_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_matrix);
    // zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    // zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    // zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    // zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    // zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    //zjl::prepare_mesh_ply(mesh_path_rosa, mesh_config, "mesh-rosa");
    //zjl::prepare_mesh_ply(mesh_path_l1, mesh_config, "mesh-l1");
    //zjl::prepare_mesh_ply(mesh_path_l1mst, mesh_config, "mesh-l1mst");
    //zjl::prepare_mesh_ply(mesh_path_mdcs, mesh_config, "mesh-mdcs");
    //zjl::prepare_mesh_ply(mesh_path_ours, mesh_config, "mesh-ours");
    //zjl::prepare_mesh_ply(mesh_path_gt, mesh_config, "mesh-gt");
    zjl::draw();
    zjl::deinit();
}

void draw_result_art_scan_12() {
    std::array<float, 16> camera_matrix = {
        0.0132, -0.0728, -0.9973, 0.0000, 0.0000, 0.9973, -0.0728, 0.0000, 0.9998, 0.0010, 0.0132, 0.0000, 0.0258, -0.1137, -2.4579, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\art_scan_12.npts"; 
    auto mesh_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-12-gt.ply"; 
    auto mesh_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-12-rosa.ply"; 
    auto mesh_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-12-l1.ply"; 
    auto mesh_path_l1mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-12-l1mst.ply"; 
    auto mesh_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-12-mdcs.ply"; 
    auto mesh_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-12-ours.ply"; 
    // ===
    auto skel_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-4-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-rosa.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\raw-scan-4-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    auto mesh_config = get_mesh_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_matrix);
    // zjl::prepare_skeleton_obj(skel_path, skel_config, "skeleton");
    // zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    // zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    // zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    // zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");
    //zjl::prepare_mesh_ply(mesh_path_rosa, mesh_config, "mesh-rosa");
    //zjl::prepare_mesh_ply(mesh_path_l1, mesh_config, "mesh-l1");
    //zjl::prepare_mesh_ply(mesh_path_l1mst, mesh_config, "mesh-l1mst");
    //zjl::prepare_mesh_ply(mesh_path_mdcs, mesh_config, "mesh-mdcs");
    //zjl::prepare_mesh_ply(mesh_path_ours, mesh_config, "mesh-ours");
    //zjl::prepare_mesh_ply(mesh_path_gt, mesh_config, "mesh-gt");
    zjl::draw();
    zjl::deinit();
}

void draw_result_art_scan_1() {
    std::array<float, 16> camera_matrix = {
        -0.8712, 0.0011, 0.4908, 0.0000, -0.0000, 0.9999, -0.0024, 0.0000, -0.4908, -0.0021, -0.8713, 0.0000, 0.0258, -0.1137, -2.2245, 1.0000 
    }; // raw-scan-4
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\art_scan_1.npts"; 
    auto mesh_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-1-gt.ply"; 
    auto mesh_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-1-rosa.ply"; 
    auto mesh_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-1-l1.ply"; 
    auto mesh_path_l1mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-1-l1mst.ply"; 
    auto mesh_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-1-mdcs.ply"; 
    auto mesh_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-mesh-1-ours.ply"; 
    // ===
    auto skel_path_ours = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-1-ours.obj"; 
    auto skel_path_l1 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-1-l1.obj"; 
    auto skel_path_l1_mst = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-1-l1mst.obj"; 
    auto skel_path_rosa = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-1-rosa.obj"; 
    auto skel_path_mdcs = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-1-mdcs.obj"; 
    auto skel_path_gt = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\art-scan-1-gt.obj"; 

    auto skel_config = get_skel_config();
    auto pc_config = get_pc_config();
    auto mesh_config = get_mesh_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_matrix);
    zjl::prepare_skeleton_obj(skel_path_ours, skel_config, "skeleton-ours");
    zjl::prepare_skeleton_obj(skel_path_l1, skel_config, "skeleton-l1");
    zjl::prepare_skeleton_obj(skel_path_l1_mst, skel_config, "skeleton-l1-mst");
    zjl::prepare_skeleton_obj(skel_path_rosa, skel_config, "skeleton-rosa");
    zjl::prepare_skeleton_obj(skel_path_mdcs, skel_config, "skeleton-mdcs");
    zjl::prepare_skeleton_obj(skel_path_gt, skel_config, "skeleton-gt");
    zjl::prepare_point_cloud_npts(pc_path, pc_config, "point cloud");

    //zjl::prepare_mesh_ply(mesh_path_ours, mesh_config, "mesh-ours");
    //zjl::prepare_mesh_ply(mesh_path_rosa, mesh_config, "mesh-rosa");
    //zjl::prepare_mesh_ply(mesh_path_l1, mesh_config, "mesh-l1");
    //zjl::prepare_mesh_ply(mesh_path_l1mst, mesh_config, "mesh-l1mst");
    //zjl::prepare_mesh_ply(mesh_path_mdcs, mesh_config, "mesh-mdcs");
    //zjl::prepare_mesh_ply(mesh_path_gt, mesh_config, "mesh-gt");
    zjl::draw();
    zjl::deinit();
}

void draw_knn_distance_taper_off_illustration() {
    std::array<float, 16> camera_params = {
        0.8026, -0.0642, 0.5930, 0.0000, 0.0000, 0.9942, 0.1076, 0.0000, -0.5965, -0.0863, 0.7980, 0.0000, 0.0727, -0.2783, -2.4473, 1.0000 
    };

    auto cylinder_0_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cylinder_whole.npts";
    auto cylinder_0n_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cylinder_whole_neighbors.npts";
    auto cylinder_1_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cylinder_1.npts";
    auto cylinder_2_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cylinder_2.npts";
    auto cylinder_3_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cylinder_3.npts";
    auto cylinder_4_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\cylinder_4.npts";

    std::vector<Point3d> c0_neighbors_center = {
        {0, -1, 1.35922}
    }; 

    std::vector<Point3d> c1_neighbors = {
        {0.2503, 0.7703, -1},
        {-0.2503, 0.7703, -1},
        {0.4761, 0.6553, -1},
        {-0.4761, 0.6553, -1},
    }; 

    std::vector<Point3d> c2_neighbors = {
        {0.2225, 0.6847, -1},
        {-0.2225, 0.6847, -1},
        {0.4232, 0.5825, -1},
        {-0.4232, 0.5825, -1},
    }; 

    std::vector<Point3d> c4_neighbors = {
        {0.1391, 0.4280, -1},
        {-0.1391, 0.4280, -1},
        {0.2645, 0.3641, -1},
        {-0.2645, 0.3641, -1},
    }; 

    std::vector<Point3d> c1_neighbors_center = {
        {0, 0.81, -1} 
    }; 

    std::vector<Point3d> c2_neighbors_center = {
        {0, 0.72, -1} 
    }; 
    
    std::vector<Point3d> c4_neighbors_center = {
        {0, 0.45, -1} 
    }; 

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0143;

    ShowPointCloudConfig pc_config_neighbors;
    pc_config_neighbors.point_color_r = 0.1f;
    pc_config_neighbors.point_color_g = 0.9f;
    pc_config_neighbors.point_color_b = 0.3f;
    pc_config_neighbors.point_radius = 0.0153;

    ShowPointCloudConfig pc_config_neighbors_c;
    pc_config_neighbors_c.point_color_r = 0.99f;
    pc_config_neighbors_c.point_color_g = 0.0f;
    pc_config_neighbors_c.point_color_b = 0.0f;
    pc_config_neighbors_c.point_radius = 0.0155;

    ShowSkeletonConfig skeleton_config;
    skeleton_config.node_radius = 0.00424;
    skeleton_config.edge_radius = 0.00146;
    skeleton_config.show_node = true;

    zjl::init();
    zjl::disable_ground();
    // whole cylinder
    // zjl::prepare_point_cloud_npts(cylinder_0_path, pc_config, "c0");
    // zjl::prepare_point_cloud_npts(cylinder_0n_path, pc_config_neighbors, "c0n");
    // zjl::prepare_point_cloud_npts_fromdata(c0_neighbors_center, pc_config_neighbors_c, "p0c");
    zjl::prepare_point_cloud_npts(cylinder_1_path, pc_config, "c1");
    zjl::prepare_point_cloud_npts(cylinder_2_path, pc_config, "c2");
    //zjl::prepare_point_cloud_npts(cylinder_3_path, pc_config, "c3");
    zjl::prepare_point_cloud_npts(cylinder_4_path, pc_config, "c4");
    zjl::prepare_point_cloud_npts_fromdata(c1_neighbors, pc_config_neighbors, "c1n");
    zjl::prepare_point_cloud_npts_fromdata(c2_neighbors, pc_config_neighbors, "c2n");
    zjl::prepare_point_cloud_npts_fromdata(c4_neighbors, pc_config_neighbors, "c4n");
    zjl::prepare_point_cloud_npts_fromdata(c1_neighbors_center, pc_config_neighbors_c, "c1nc");
    zjl::prepare_point_cloud_npts_fromdata(c2_neighbors_center, pc_config_neighbors_c, "c2nc");
    zjl::prepare_point_cloud_npts_fromdata(c4_neighbors_center, pc_config_neighbors_c, "c4nc");
    //zjl::prepare_point_cloud_npts(cell_points_path, pc_config, "pc-ss");
    zjl::draw();
    zjl::deinit();

}

void draw_sigma_illustration() {
    auto pc_sigma = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sigma_illustration.npts";
    std::vector<Point3d> center_1 = {{0.40, 0.00, 0.0}};
    std::vector<Point3d> center_2 = {{0.75, 0.00, 0.0}};
    std::vector<Point3d> center_3 = {{1.06, 0.00, 0.0}};
    std::vector<Point3d> center_4 = {{1.63, 0.00, 0.0}};
    // std::vector<Point3d> new_center = {{+0.0332999, 0.07437, -0.02917}};

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0025;

    ShowPointCloudConfig center_config;
    center_config.point_color_r = 0.99f;
    center_config.point_color_g = 0.0f;
    center_config.point_color_b = 0.0f;
    center_config.point_radius = 0.01;

    zjl::init();
    zjl::disable_ground();
    //zjl::set_camera_from_16_floats(camera_params);
    zjl::prepare_point_cloud_npts(pc_sigma, pc_config, "pc");
    zjl::prepare_point_cloud_npts_fromdata(center_1, center_config, "c1");
    //zjl::prepare_point_cloud_npts_fromdata(center_2, center_config, "c2");
    zjl::prepare_point_cloud_npts_fromdata(center_3, center_config, "c3");
    zjl::prepare_point_cloud_npts_fromdata(center_4, center_config, "c4");
    // zjl::prepare_point_cloud_npts(branch_1_iter_0, skel_config, "b1-i0");
    zjl::draw();
    zjl::deinit();

}

void draw_missing_data_and_skeleton() {
    std::array<float, 16> camera_params = {
        -0.4840, -0.0998, 0.8694, 0.0000, -0.0000, 0.9934, 0.1140, 0.0000, -0.8750, 0.0551, -0.4808, 0.0000, 0.0116, -0.0676, -1.8432, 1.0000 
    }; // mve-scan-5
    auto missing_05 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\missing_mve_scan_5_05.npts";
    auto missing_10 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\missing_mve_scan_5_10.npts";
    auto missing_15 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\missing_mve_scan_5_15.npts";
    auto missing_20 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\missing_mve_scan_5_20.npts";
    auto missing_25 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\missing_mve_scan_5_25.npts";
    auto skel_05 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-rep-05.obj";
    auto skel_10 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-rep-10.obj";
    auto skel_15 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-rep-15.obj";
    auto skel_20 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-rep-20.obj";
    auto skel_25 = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-rep-25.obj";
    // std::vector<Point3d> new_center = {{+0.0332999, 0.07437, -0.02917}};
    auto skel_05_wor = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-wor-05.obj";
    auto skel_10_wor = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-wor-10.obj";
    auto skel_15_wor = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-wor-15.obj";
    auto skel_20_wor = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-wor-20.obj";
    auto skel_25_wor = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Result\\missing-mve-scan-5-wor-25.obj";

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0025;

    ShowPointCloudConfig center_config;
    center_config.point_color_r = 0.99f;
    center_config.point_color_g = 0.0f;
    center_config.point_color_b = 0.0f;
    center_config.point_radius = 0.01;

    auto skel_config = get_skel_config();

    zjl::init();
    zjl::disable_ground();
    zjl::set_camera_from_16_floats(camera_params);
    // zjl::prepare_point_cloud_npts(missing_05, pc_config, "pc05");
    // zjl::prepare_point_cloud_npts(missing_10, pc_config, "pc10");
    // zjl::prepare_point_cloud_npts(missing_15, pc_config, "pc15");
    // zjl::prepare_point_cloud_npts(missing_20, pc_config, "pc20");
    // zjl::prepare_point_cloud_npts(missing_25, pc_config, "pc25");
    // zjl::prepare_skeleton_obj(skel_05, skel_config, "skel05");
    // zjl::prepare_skeleton_obj(skel_10, skel_config, "skel10");
    // zjl::prepare_skeleton_obj(skel_15, skel_config, "skel15");
    // zjl::prepare_skeleton_obj(skel_20, skel_config, "skel20");
    // zjl::prepare_skeleton_obj(skel_25, skel_config, "skel25");
    zjl::prepare_skeleton_obj(skel_05_wor, skel_config, "skel05-wor");
    zjl::prepare_skeleton_obj(skel_10_wor, skel_config, "skel10-wor");
    zjl::prepare_skeleton_obj(skel_15_wor, skel_config, "skel15-wor");
    zjl::prepare_skeleton_obj(skel_20_wor, skel_config, "skel20-wor");
    zjl::prepare_skeleton_obj(skel_25_wor, skel_config, "skel25-wor");
    zjl::draw();
    zjl::deinit();

}

void draw_overview() {
    std::array<float, 16> camera_params = {
        -0.4840, -0.0998, 0.8694, 0.0000, -0.0000, 0.9934, 0.1140, 0.0000, -0.8750, 0.0551, -0.4808, 0.0000, 0.0116, -0.0676, -1.8432, 1.0000 
    }; // mve-scan-5
    auto missing_pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\overview_raw_pc_missing.npts";
    auto missing_pc_wlop = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\overview_raw_pc_wlop_2.npts";
    auto skeletal_pc = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\overview_skeletal_pc_missing.npts";
    auto raw_skel = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\overview_skel_2_missing.obj";
    auto raw_skel_repair = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\overview_skel_2_repair.obj";
    auto raw_skel_repair_recenter = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\overview_skel_2_repair_recenter.obj";

    ShowPointCloudConfig pc_config;
    pc_config.point_color_r = 233.0f / 255.0f;
    pc_config.point_color_g = 233.0f / 255.0f;
    pc_config.point_color_b = 101.0f / 255.0f;
    pc_config.point_radius = 0.0038;

    ShowPointCloudConfig skeletal_pc_config;
    skeletal_pc_config.point_color_r = 0.1f;
    skeletal_pc_config.point_color_g = 0.3f;
    skeletal_pc_config.point_color_b = 0.9f;
    skeletal_pc_config.point_radius = 0.0048;

    auto skel_config = get_skel_config();
    skel_config.node_color_r = 0.99;
    skel_config.node_color_g = 0.00;
    skel_config.node_color_b = 0.00;
    skel_config.node_radius = 0.009;
    skel_config.edge_radius = 0.006;
    skel_config.show_node = true;

    zjl::init();
    zjl::disable_ground();
    //zjl::set_camera_from_16_floats(camera_params);
    zjl::prepare_point_cloud_npts(missing_pc, pc_config, "missing_pc");
    zjl::prepare_point_cloud_npts(missing_pc_wlop, pc_config, "missing_pc_w");
    zjl::prepare_point_cloud_npts(skeletal_pc, skeletal_pc_config, "skeletal_pc");
    zjl::prepare_skeleton_obj(raw_skel, skel_config, "raw-skel");
    zjl::prepare_skeleton_obj(raw_skel_repair, skel_config, "raw-skel-repair");
    zjl::prepare_skeleton_obj(raw_skel_repair_recenter, skel_config, "raw-skel-rr");
    // zjl::prepare_skeleton_obj(skel_25, skel_config, "skel25");
    zjl::draw();
    zjl::deinit();

}

void draw_in_code() {
    //draw_result_art_scan_1();
    //draw_result_art_scan_12();
    //draw_result_art_scan_13();
    //draw_result_art_scan_14();
    //draw_result_art_scan_15();
    //draw_result_art_scan_16();

    //draw_result_raw_scan_4();
    //draw_result_raw_scan_10();
    //draw_result_raw_scan_17();
    //draw_result_mve_scan_2();
    //draw_result_mve_scan_5();
    //draw_result_mve_scan_8();

    //draw_knn_distance_taper_off_illustration();
    //draw_sigma_illustration();
    //draw_missing_data_and_skeleton();
    draw_overview();

    // draw_single_branch_contraction_1();
    // draw_single_branch_contraction_3();
    // draw_six_scans();
    //draw_recenter_illustration();
    //draw_recenter_skel_comparison();
    //draw_mls_comparison();
    //draw_anistropic_illustration();

    //draw_furthest_sample_illustration1();
    //draw_furthest_sample_illustration2();
    //draw_skeleton_repair();
    //draw_cardinal_illustration();
    //draw_octree_illustration();

    //draw_anistropic_comparison();
    //draw_art_scan_1();
    //draw_wlop_comparison();
    //draw_global_branch_contraction();
}

void draw_by_user(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage:" << argv[0] << " *.npts(input, point cloud file) --pointRadius 0.005 --cameraFile xxx.matrix --disableGround --screenshotAndExit" << endl;
        return;
    }
    string points_file_path(argv[1]);

    ShowPointCloudConfig point_cloud_config;
    ShowMeshConfig mesh_config;
    bool auto_exit = false;

    for (int i = 0; i < argc; i ++) {
        if (strcmp(argv[i], "--pointRadius") == 0) {
            if (i + 1 < argc) {
                float radius = atof(argv[i + 1]);
                if (radius > 0.0) {
                    point_cloud_config.point_radius = radius;

                }
            }
        } else if (strcmp(argv[i], "--cameraFile") == 0) {
            if (i + 1 < argc) {
                ifstream ifs(argv[i + 1]); 
                if (ifs.is_open()) {
                    float vals[16];
                    for (int i = 0; i < 16; i ++) ifs >> vals[i];
                    glm::mat4x4 view_mat = glm::make_mat4(vals);
                    ifs.close();
                    polyscope::view::setScreenshotMatrix = true;
                    polyscope::view::viewMat4Screenshot = view_mat;
                } else {
                    printf("Error: Invalid camera matrix\n");
                }
            }
        } else if (strcmp(argv[i], "--disableGround") == 0) {
            point_cloud_config.enable_ground = false;
        } else if (strcmp(argv[i], "--screenshotAndExit") == 0) {
            auto_exit = true;
        }

    }

    // prepare data container
    // MatrixXf raw_point_cloud;
    // std::vector<Point3d> points;

    // zjl::IO::read_point_cloud_from_npts_without_normals(points_file_path, points);
    // zjl::Util::xyz_points_to_eigen_matrix(points, raw_point_cloud);
    // read_points_from_npts(points_file_path, raw_point_cloud);

    std::thread t([&](){
        zjl::init();
        if (zjl::Util::string_end_with(points_file_path, ".npts")) {
            zjl::prepare_point_cloud_npts(points_file_path, point_cloud_config);
        } else if (zjl::Util::string_end_with(points_file_path, ".ply")) {
            zjl::prepare_mesh_ply(points_file_path, mesh_config, "mesh");
        } else if (zjl::Util::string_end_with(points_file_path, ".obj")) {
            zjl::prepare_skeleton_obj(points_file_path, ShowSkeletonConfig(), "skel");
        }
        polyscope::gl::groundPlaneEnabled = point_cloud_config.enable_ground;
        // ===
        zjl::initializing = false;
        zjl::cv.notify_all();
        // ===
        zjl::draw();
        zjl::deinit();
        //render_point_cloud(raw_point_cloud, point_cloud_config);
    });
    //render_point_cloud(raw_point_cloud, point_cloud_config);

    if (auto_exit) {
        std::unique_lock<std::mutex> mutex_lock(zjl::render_mutex);
        while (zjl::initializing) {
            zjl::cv.wait(mutex_lock);
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1s);    
        polyscope::state::message = polyscope::Message::Screenshot;
        // TODO: replace it with condition variable
        std::this_thread::sleep_for(1s);    
    } else {
        t.join();
    }
}

int main(int argc, char** argv) {
    //  using namespace std;
    //  cout << "Hello world!" << endl;
    // ========
    draw_in_code();
    // draw_by_user(argc, argv);

    return 0;
}
