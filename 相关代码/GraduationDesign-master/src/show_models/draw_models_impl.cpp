#include "./draw_models_impl.h"

using namespace std;
using namespace Eigen;

namespace zjl {
void init() {
    polyscope::init();
}

void disable_ground() {
    polyscope::gl::groundPlaneEnabled = false;
}

void prepare_box(const Point3d& center, const Point3d& diagonal_vector, const std::string& name) {
    MatrixXf points(8, 3); 
    MatrixXi edges(12, 2);
    points(0, 0) = center.x - diagonal_vector.x;
    points(0, 1) = center.y - diagonal_vector.y;
    points(0, 2) = center.z - diagonal_vector.z;

    points(1, 0) = center.x - diagonal_vector.x;
    points(1, 1) = center.y - diagonal_vector.y;
    points(1, 2) = center.z + diagonal_vector.z;

    points(2, 0) = center.x - diagonal_vector.x;
    points(2, 1) = center.y + diagonal_vector.y;
    points(2, 2) = center.z - diagonal_vector.z;

    points(3, 0) = center.x - diagonal_vector.x;
    points(3, 1) = center.y + diagonal_vector.y;
    points(3, 2) = center.z + diagonal_vector.z;

    points(4, 0) = center.x + diagonal_vector.x;
    points(4, 1) = center.y - diagonal_vector.y;
    points(4, 2) = center.z - diagonal_vector.z;

    points(5, 0) = center.x + diagonal_vector.x;
    points(5, 1) = center.y - diagonal_vector.y;
    points(5, 2) = center.z + diagonal_vector.z;

    points(6, 0) = center.x + diagonal_vector.x;
    points(6, 1) = center.y + diagonal_vector.y;
    points(6, 2) = center.z - diagonal_vector.z;
    
    points(7, 0) = center.x + diagonal_vector.x;
    points(7, 1) = center.y + diagonal_vector.y;
    points(7, 2) = center.z + diagonal_vector.z;

    edges(0, 0) = 0; edges(0, 1) = 1;
    edges(1, 0) = 1; edges(1, 1) = 3;
    edges(2, 0) = 3; edges(2, 1) = 2;
    edges(3, 0) = 2; edges(3, 1) = 0;

    edges(4, 0) = 4; edges(4, 1) = 5;
    edges(5, 0) = 5; edges(5, 1) = 7;
    edges(6, 0) = 7; edges(6, 1) = 6;
    edges(7, 0) = 6; edges(7, 1) = 4;

    edges(8, 0) = 0; edges(8, 1) = 4;
    edges(9, 0) = 1; edges(9, 1) = 5;
    edges(10, 0) = 2; edges(10, 1) = 6;
    edges(11, 0) = 3; edges(11, 1) = 7;

    polyscope::registerCurveNetwork(name, points, edges);
}

void prepare_xyz_axis(float inf) {
    //float inf = 10000.0f;
    std::vector<std::array<float, 3>> points_x = {{-inf, 0.0f, 0.0f}, {inf, 0.0f, 0.0f}};
    std::vector<std::array<float, 3>> points_y = {{0.0f, -inf, 0.0f}, {0.0f, inf, 0.0f}};
    std::vector<std::array<float, 3>> points_z = {{0.0f, 0.0f, -inf}, {0.0f, 0.0f, inf}};
    auto x_axis = polyscope::registerCurveNetworkLine("x-axis", points_x);
    auto y_axis = polyscope::registerCurveNetworkLine("y-axis", points_y);
    auto z_axis = polyscope::registerCurveNetworkLine("z-axis", points_z);
    x_axis->baseColor = {151.0/255.0, 55.0/255.0, 68.0/255.0};
    y_axis->baseColor = {98.0/255.0, 136.0/255.0, 31.0/255.0};
    z_axis->baseColor = {40.0/255.0, 142.0/255.0, 249.0/255.0};
}

void prepare_point_cloud_npts_fromdata(const std::vector<Point3d>& points, ShowPointCloudConfig config, const std::string& name) {
    static int cnt = 0;
    MatrixXf raw_point_cloud;
    zjl::Util::xyz_points_to_eigen_matrix(points, raw_point_cloud);
    auto pc = polyscope::registerPointCloud(name, raw_point_cloud);
    pc->pointRadius = config.point_radius;
    pc->pointColor = { config.point_color_r, config.point_color_g, config.point_color_b};
}

void prepare_mesh_ply_from_data(const std::vector<Point3d>& vertices, const std::vector<Triangle>& faces, ShowMeshConfig config, const std::string& name) {
    MatrixXf raw_point_cloud;
    MatrixXi faces_mat;
    zjl::Util::xyz_points_to_eigen_matrix(vertices, raw_point_cloud);
    zjl::Util::ijk_faces_to_eigen_matrix(faces, faces_mat);
    // ??? VVVV
    auto mesh = polyscope::registerSurfaceMesh(name, raw_point_cloud, faces_mat);
    mesh->surfaceColor = {config.mesh_color_r, config.mesh_color_g, config.mesh_color_b};
    // //pc->pointRadius = point_radius;
}

void prepare_mesh_ply(const std::string& filepath, ShowMeshConfig config, const std::string& name) {
    //std::cout << "To prepare a mesh: " << name << std::endl;
    std::vector<Point3d> vertices;
    std::vector<Triangle> faces;
    zjl::IO::read_trimesh_from_ply(filepath, vertices, faces);

    prepare_mesh_ply_from_data(vertices, faces, config, name);
}

void prepare_point_cloud_npts(const std::string& filepath, ShowPointCloudConfig config, const std::string&name) {
    //std::cout << "To prepare a point cloud: " << name << std::endl;
    std::vector<Point3d> points;
    zjl::IO::read_point_cloud_from_npts_without_normals(filepath, points);
    return prepare_point_cloud_npts_fromdata(points, config, name);
}

void prepare_skeleton_from_data(const std::vector<Point3d>& points, const std::vector<Edge>& edges, ShowSkeletonConfig config, const std::string& name) {
    MatrixXf points_mat;
    MatrixXi edges_mat;
    zjl::Util::xyz_points_to_eigen_matrix(points, points_mat);
    zjl::Util::ij_edge_to_eigen_matrix(edges, edges_mat);
    auto graph = polyscope::registerCurveNetwork(name, points_mat, edges_mat);
    if (config.show_node) {
        auto pc = polyscope::registerPointCloud(name + "-points", points_mat);
        pc->pointRadius = config.node_radius;
        pc->pointColor = {config.node_color_r, config.node_color_g, config.node_color_b};
    }
    graph->radius = config.edge_radius;
    graph->baseColor = {config.edge_color_r, config.edge_color_g, config.edge_color_b}; 
}

void prepare_skeleton_obj(const std::string& filepath, ShowSkeletonConfig config, const std::string& name) {
    std::vector<Point3d> points;
    std::vector<Edge> edges;
    zjl::IO::read_graph_from_obj(filepath, points, edges);

    prepare_skeleton_from_data(points, edges, config, name);

    // MatrixXf points_mat;
    // MatrixXi edges_mat;
    // zjl::Util::xyz_points_to_eigen_matrix(points, points_mat);
    // zjl::Util::ij_edge_to_eigen_matrix(edges, edges_mat);
    // auto graph = polyscope::registerCurveNetwork(name, points_mat, edges_mat);
    // if (config.show_node) {
    //     auto pc = polyscope::registerPointCloud(name + "-points", points_mat);
    //     pc->pointRadius = config.node_radius;
    //     pc->pointColor = {config.node_color_r, config.node_color_g, config.node_color_b};
    // }
    // graph->radius = config.edge_radius;
    // graph->baseColor = {config.edge_color_r, config.edge_color_g, config.edge_color_b}; 

}

void set_camera_from_16_floats(std::array<float, 16> camera_mat) {
    glm::mat4x4 view_mat = glm::make_mat4(camera_mat.data());
    polyscope::state::message = polyscope::Message::MoveCamera;
    polyscope::view::setScreenshotMatrix = true;
    polyscope::view::viewMat4Screenshot = view_mat;
}

void draw() {
    polyscope::show();
}

void deinit() {

}

};
