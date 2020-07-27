#include <vector>
#include <algorithm>
#include <iostream>
#include "./point_xyz.hpp"
#include "./io_xyz.hpp"
#include "./octree_xyz.hpp"

using namespace std;

void generate_octree_data() {
    auto pc_path = "D:\\User\\zhouh\\Documents\\CQU_PC_2\\Data\\ForFigures\\sample-init.npts";
    std::vector<Point3d> points;
    zjl::IO::read_point_cloud_from_npts_without_normals(pc_path, points);
    zjl::OctreeXYZ<Point3d> octree(points);
    octree.build_index();
    const auto cells_map = octree.get_nodes();

    std::vector<Point3d> octree_cell_vertices;
    std::vector<Edge> octree_cell_edges;

    int cell_count = 0;
    for (const auto p : cells_map) {
        const auto cell_pointer = p.second; 
        Point3d p0 = {
            cell_pointer->center.x - cell_pointer->x_extent,
            cell_pointer->center.y - cell_pointer->y_extent,
            cell_pointer->center.z - cell_pointer->z_extent
        };
        Point3d p1 = {
            cell_pointer->center.x - cell_pointer->x_extent,
            cell_pointer->center.y - cell_pointer->y_extent,
            cell_pointer->center.z + cell_pointer->z_extent
        };
        Point3d p2 = {
            cell_pointer->center.x - cell_pointer->x_extent,
            cell_pointer->center.y + cell_pointer->y_extent,
            cell_pointer->center.z - cell_pointer->z_extent
        };
        Point3d p3 = {
            cell_pointer->center.x - cell_pointer->x_extent,
            cell_pointer->center.y + cell_pointer->y_extent,
            cell_pointer->center.z + cell_pointer->z_extent
        };
        Point3d p4 = {
            cell_pointer->center.x + cell_pointer->x_extent,
            cell_pointer->center.y - cell_pointer->y_extent,
            cell_pointer->center.z - cell_pointer->z_extent
        };
        Point3d p5 = {
            cell_pointer->center.x + cell_pointer->x_extent,
            cell_pointer->center.y - cell_pointer->y_extent,
            cell_pointer->center.z + cell_pointer->z_extent
        };
        Point3d p6 = {
            cell_pointer->center.x + cell_pointer->x_extent,
            cell_pointer->center.y + cell_pointer->y_extent,
            cell_pointer->center.z - cell_pointer->z_extent
        };
        Point3d p7 = {
            cell_pointer->center.x + cell_pointer->x_extent,
            cell_pointer->center.y + cell_pointer->y_extent,
            cell_pointer->center.z + cell_pointer->z_extent
        };
        octree_cell_vertices.push_back(p0);
        octree_cell_vertices.push_back(p1);
        octree_cell_vertices.push_back(p2);
        octree_cell_vertices.push_back(p3);
        octree_cell_vertices.push_back(p4);
        octree_cell_vertices.push_back(p5);
        octree_cell_vertices.push_back(p6);
        octree_cell_vertices.push_back(p7);

        octree_cell_edges.push_back({cell_count * 8 + 0, cell_count * 8 + 1});
        octree_cell_edges.push_back({cell_count * 8 + 1, cell_count * 8 + 3});
        octree_cell_edges.push_back({cell_count * 8 + 3, cell_count * 8 + 2});
        octree_cell_edges.push_back({cell_count * 8 + 2, cell_count * 8 + 0});

        octree_cell_edges.push_back({cell_count * 8 + 4, cell_count * 8 + 5});
        octree_cell_edges.push_back({cell_count * 8 + 5, cell_count * 8 + 7});
        octree_cell_edges.push_back({cell_count * 8 + 7, cell_count * 8 + 6});
        octree_cell_edges.push_back({cell_count * 8 + 6, cell_count * 8 + 4});

        octree_cell_edges.push_back({cell_count * 8 + 0, cell_count * 8 + 4});
        octree_cell_edges.push_back({cell_count * 8 + 1, cell_count * 8 + 5});
        octree_cell_edges.push_back({cell_count * 8 + 2, cell_count * 8 + 6});
        octree_cell_edges.push_back({cell_count * 8 + 3, cell_count * 8 + 7});
        cell_count += 1;

        std::cout << p.first << std::endl;
    }
    zjl::IO::write_graph_to_obj("D:\\o.obj", octree_cell_vertices, octree_cell_edges);

    const auto one_cell_ptr = cells_map.at(9952);
    std::vector<Point3d> subset_points;
    std::vector<Point3d> cell_vertices;
    std::vector<Edge> cell_edges;
    for (const auto point : points) {
        if (one_cell_ptr->contains(point)) {
            subset_points.push_back(point);
        }
    }
    // ===
        Point3d p0 = {
            one_cell_ptr->center.x - one_cell_ptr->x_extent,
            one_cell_ptr->center.y - one_cell_ptr->y_extent,
            one_cell_ptr->center.z - one_cell_ptr->z_extent
        };
        Point3d p1 = {
            one_cell_ptr->center.x - one_cell_ptr->x_extent,
            one_cell_ptr->center.y - one_cell_ptr->y_extent,
            one_cell_ptr->center.z + one_cell_ptr->z_extent
        };
        Point3d p2 = {
            one_cell_ptr->center.x - one_cell_ptr->x_extent,
            one_cell_ptr->center.y + one_cell_ptr->y_extent,
            one_cell_ptr->center.z - one_cell_ptr->z_extent
        };
        Point3d p3 = {
            one_cell_ptr->center.x - one_cell_ptr->x_extent,
            one_cell_ptr->center.y + one_cell_ptr->y_extent,
            one_cell_ptr->center.z + one_cell_ptr->z_extent
        };
        Point3d p4 = {
            one_cell_ptr->center.x + one_cell_ptr->x_extent,
            one_cell_ptr->center.y - one_cell_ptr->y_extent,
            one_cell_ptr->center.z - one_cell_ptr->z_extent
        };
        Point3d p5 = {
            one_cell_ptr->center.x + one_cell_ptr->x_extent,
            one_cell_ptr->center.y - one_cell_ptr->y_extent,
            one_cell_ptr->center.z + one_cell_ptr->z_extent
        };
        Point3d p6 = {
            one_cell_ptr->center.x + one_cell_ptr->x_extent,
            one_cell_ptr->center.y + one_cell_ptr->y_extent,
            one_cell_ptr->center.z - one_cell_ptr->z_extent
        };
        Point3d p7 = {
            one_cell_ptr->center.x + one_cell_ptr->x_extent,
            one_cell_ptr->center.y + one_cell_ptr->y_extent,
            one_cell_ptr->center.z + one_cell_ptr->z_extent
        };
        cell_vertices.push_back(p0);
        cell_vertices.push_back(p1);
        cell_vertices.push_back(p2);
        cell_vertices.push_back(p3);
        cell_vertices.push_back(p4);
        cell_vertices.push_back(p5);
        cell_vertices.push_back(p6);
        cell_vertices.push_back(p7);

        cell_edges.push_back({ 0,  1});
        cell_edges.push_back({ 1,  3});
        cell_edges.push_back({ 3,  2});
        cell_edges.push_back({ 2,  0});

        cell_edges.push_back({ 4,  5});
        cell_edges.push_back({ 5,  7});
        cell_edges.push_back({ 7,  6});
        cell_edges.push_back({ 6,  4});

        cell_edges.push_back({ 0,  4});
        cell_edges.push_back({ 1,  5});
        cell_edges.push_back({ 2,  6});
        cell_edges.push_back({ 3,  7});
    // ===
    // zjl::IO::write_point_cloud_to_npts_without_normals("D:\\cell_subset.npts", subset_points);
    // zjl::IO::write_graph_to_obj("D:\\one_cell.obj", cell_vertices, cell_edges);
    std::cout << "=== " << subset_points.size() << std::endl;
}

int main(int argc, char** argv) {
    generate_octree_data();
    return 0;
}