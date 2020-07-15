#pragma once
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <string>
#include <limits>
#include <array>
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
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "../io_xyz.hpp"
#include "../point_xyz.hpp"
#include "./eigen_utils.hpp"


struct ShowSkeletonConfig {
    bool show_node;
    float edge_radius;
    float node_radius;
    float node_color_r;
    float node_color_g;
    float node_color_b;
    float edge_color_r;
    float edge_color_g;
    float edge_color_b;

    ShowSkeletonConfig() :
        show_node(true),
        edge_radius(1.0f) ,
        node_radius(0.005),
        node_color_r(1.0f),
        node_color_g(0.0f),
        node_color_b(0.0f),
        edge_color_r(0.1f),
        edge_color_g(0.3f),
        edge_color_b(0.9f)
    {}
};

struct ShowPointCloudConfig {
    float point_radius;
    bool enable_ground;
    float point_color_r;
    float point_color_g;
    float point_color_b;

    ShowPointCloudConfig():
        point_radius(0.005f),
        enable_ground(true),
        point_color_r(0.7f),
        point_color_g(0.7f),
        point_color_b(0.7f)
    {}
};

struct ShowMeshConfig {
    float mesh_color_r; 
    float mesh_color_g; 
    float mesh_color_b; 

    ShowMeshConfig():
        mesh_color_r(0.3),
        mesh_color_g(0.9),
        mesh_color_b(0.1)
    {}
};

namespace zjl {

    void init();

    void prepare_xyz_axis(float bound = 100.0f);

    void prepare_box(const Point3d& center, const Point3d& diagonal_vector, const std::string& name="bb");

    void prepare_point_cloud_npts(const std::string& filepath, ShowPointCloudConfig config, const std::string& name="pc");

    void prepare_mesh_ply(const std::string& filepath, ShowMeshConfig config,  const std::string& name="mesh");

    void prepare_skeleton_obj(const std::string& filepath, ShowSkeletonConfig config, const std::string& name="skel");

    void prepare_skeleton_from_data(const std::vector<Point3d>& points, const std::vector<Edge>& edges, ShowSkeletonConfig config, const std::string& name); 

    void prepare_point_cloud_npts_fromdata(const std::vector<Point3d>& points, ShowPointCloudConfig config, const std::string& name="pc");

    void prepare_mesh_ply_from_data(const std::vector<Point3d>& vertices, const std::vector<Triangle>& faces, ShowMeshConfig config, const std::string& name="mesh");

    void set_camera_from_16_floats(std::array<float, 16> camera_mat);

    void disable_ground();
    // void send_message(polyscope::Message message);

    void draw();

    void deinit();
};