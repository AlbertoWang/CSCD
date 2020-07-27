#pragma once
#include <iostream>
#include <cmath>
#include <string>
#include <cstring>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <random>
#include <functional>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "octree_xyz.hpp"

namespace zjl {

namespace zjl_int {
#define CONST_E		2.7182818284590452354	/* e */

template<typename P>
struct BoundingBox {
    P center;
    P extent;

    static BoundingBox<P> of(const std::vector<P>& raw_points) {
        if (raw_points.empty()) {
            cerr << "Error: no points" << endl;
            exit(-1);
        }
        P extent(0, 0, 0);
        P center(0, 0, 0);
        const double LD = std::numeric_limits<double>::lowest();
        const double MD = std::numeric_limits<double>::max();
        P max_point(LD, LD, LD);
        P min_point(MD, MD, MD);
        for (const auto& kv : raw_points) {
            max_point.x = std::max(kv.x, max_point.x);
            max_point.y = std::max(kv.y, max_point.y);
            max_point.z = std::max(kv.z, max_point.z);
            // ==
            min_point.x = std::min(kv.x, min_point.x);
            min_point.y = std::min(kv.y, min_point.y);
            min_point.z = std::min(kv.z, min_point.z);
        }
        center = (max_point + min_point) * 0.5;
        extent = (max_point - min_point) * 0.5;
        return {center, extent};
    }

};

};

using namespace zjl_int;
template<typename P>
class Wlop {

    using idx = int;

private:

    /* the underlying surface */
    const std::vector<P>& surf_points;

    std::vector<P>* samples;

    std::vector<std::vector<idx>> sample_surf_neighbors;

    std::vector<std::vector<idx>> sample_sample_neighbors;

    std::vector<std::vector<idx>> surf_surf_neighbors;

    std::vector<double> sample_densities;

    std::vector<double> surf_densities;

    OctreeXYZ<P>* const surf_octree;

    double neighborhood_radius;

    double repulsion_mu;

private:

    inline double gaussian(double distance) {
        return pow(CONST_E, - 4.0 * distance * distance / neighborhood_radius / neighborhood_radius);
    }

    void compute_average_term(std::vector<P>& average_vectors, std::vector<double>& average_weight_sums) {
        average_vectors.clear();
        average_weight_sums.clear();
        for (size_t i = 0; i < samples->size(); i ++) {
            const P& sample = (*samples)[i];
            double average_weight_sum = 0.0;
            P delta(0, 0, 0);
            for (idx surf_neighbor_index : sample_surf_neighbors[i]) {
                double average_weight = 0.0;
                const P& neighboring_point = surf_points[surf_neighbor_index];
                const double distance = std::max((sample - neighboring_point).norm(), neighborhood_radius * 0.01);
                // ....
                average_weight = gaussian(distance) / distance;
                average_weight *= surf_densities[surf_neighbor_index];
                const P delta_j = neighboring_point * average_weight;
                delta = delta + delta_j;
                average_weight_sum += average_weight;
            }
            average_weight_sums.push_back(average_weight_sum);
            average_vectors.push_back(delta);
        }
    }

    void compute_repulsion_term(std::vector<P>& repulsion_vectors, std::vector<double>& repulsion_weight_sums) {
        repulsion_vectors.clear();
        repulsion_weight_sums.clear();

        for (size_t i = 0; i < samples->size(); i ++) {
            const P& sample = (*samples)[i];
            P delta(0.0, 0.0, 0.0);
            double repulsion_weight_sum = 0.0;
            for (const idx neighbor_index : sample_sample_neighbors[i]) {
                if (i == neighbor_index) continue;
                const P& neighboring_point = (*samples)[neighbor_index];
                const double distance = std::max((sample - neighboring_point).norm(), neighborhood_radius * 0.01); 
                double repulsion_weight = gaussian(distance) / distance;
                repulsion_weight *= sample_densities[neighbor_index];

                P repulsion_vector_comp = (sample - neighboring_point) * repulsion_weight;
                repulsion_weight_sum += repulsion_weight;
                delta = delta + repulsion_vector_comp;
            }
            repulsion_weight_sums.push_back(repulsion_weight_sum);
            repulsion_vectors.push_back(delta);
        }
    }

    void compute_density(const std::vector<P>& points, std::vector<double>& densities, const std::vector<std::vector<idx>>& neighboring_indices, bool is_surf) {
        for (size_t i = 0; i < points.size(); i ++) {
            double density = 1.0;
            const P& point = points[i];
            const std::vector<idx>& neighbors = neighboring_indices[i];

            for (const idx neighboring_index : neighbors) {
                const P& neighboring_point = points[neighboring_index];
                double distance = (point - neighboring_point).norm();
                double density_gaussian_comp = gaussian(distance);
                density += density_gaussian_comp;
            }
            if (is_surf) {
                density = 1.0 / density;
            } else {
                density = sqrt(density);
            }
            densities[i] = density;
        }
    }

public:
    Wlop(const std::vector<P>& ps):
    surf_points(ps),
    samples(nullptr),
    surf_octree(new OctreeXYZ<P>(ps)),
    repulsion_mu(0.5)
    {
        surf_octree->build_index();
        for (size_t i = 0; i < surf_points.size(); i ++) surf_densities.push_back(1.0);
        auto bbox = zjl_int::BoundingBox<P>::of(surf_points);
        neighborhood_radius = 4.0 * sqrt(bbox.extent.norm() * 2.0 / surf_points.size()) * 2.0;
    }

    ~Wlop() {
        delete(surf_octree);
    }

    Wlop(const Wlop& w) = delete;
    Wlop(Wlop&& w) = delete;
    Wlop& operator=(const Wlop&) = delete;
    Wlop& operator=(Wlop&&) = delete;

    void iterate(int iter_idx) {
        OctreeXYZ<P> sample_octree(*samples);
        sample_octree.build_index();

        // compute neighbors
        sample_surf_neighbors.clear();
        sample_sample_neighbors.clear();
        surf_surf_neighbors.clear();

        for (size_t i = 0; i < samples->size(); i ++) {
            std::vector<idx> neighborhood_temp_1;
            std::vector<idx> neighborhood_temp_2;
            sample_octree.search_neighbors_within_distance((*samples)[i], neighborhood_radius, neighborhood_temp_1);
            sample_sample_neighbors.push_back(neighborhood_temp_1);
            // ===
            surf_octree->search_neighbors_within_distance((*samples)[i], neighborhood_radius, neighborhood_temp_2);
            sample_surf_neighbors.push_back(neighborhood_temp_2);
        }
        const double magic_ratio = 0.95;
        if (iter_idx == 0) {
            for (size_t i = 0; i < surf_points.size(); i ++) {
                std::vector<idx> neighbors_temp;
                surf_octree->search_neighbors_within_distance(surf_points[i], neighborhood_radius, neighbors_temp);
                surf_surf_neighbors.push_back(neighbors_temp);
            }
            compute_density(surf_points, surf_densities, surf_surf_neighbors, true);
        }     
        compute_density(*samples, sample_densities, sample_sample_neighbors, false);
        std::vector<double> average_weight_sums;
        std::vector<double> repulsion_weight_sums;
        std::vector<P> average_vectors;
        std::vector<P> repulsion_vectors;
        compute_average_term(average_vectors, average_weight_sums);
        compute_repulsion_term(repulsion_vectors, repulsion_weight_sums);

        for (size_t i = 0; i < samples->size(); i ++) {
            // P& sample = *samples[i];
            P sample_position(0.0, 0.0, 0.0);
            double average_weight_sum = average_weight_sums[i];
            double repulsion_weight_sum = repulsion_weight_sums[i];
            const P& average_vector = average_vectors[i];
            const P& repulsion_vector = repulsion_vectors[i];
            if (average_weight_sum > 1e-6) {
                sample_position = sample_position + average_vector * (1.0 / average_weight_sum);
            }
            //std::cout << "Debug: average_weight_sum: " << average_weight_sum << endl;
            //std::cout << "Debug: average_vector: " << sample_position.x << " " << sample_position.y << " " << sample_position.z << endl;
            if (repulsion_weight_sum > 1e-6 && repulsion_mu >= 0) {
                sample_position = sample_position + repulsion_vector * (repulsion_mu / repulsion_weight_sum);
            }
            //std::cout << "Debug: repulsion_vector: " << (repulsion_vector).x << " " << (repulsion_vector ).y << " " << (repulsion_vector ).z << endl;
            (*samples)[i] = sample_position;
        }
    }

    void resample(std::vector<P>& samples, const int iter_num, const double neighborhood_size_ratio = 1.0) {
        if (iter_num <= 0) {
            std::cerr << "Warning: iter_num: " << iter_num << std::endl;
        }
        if (surf_points.empty()) {
            std::cerr << "Warning: points is empty " << std::endl;
        }
        const double raw_size = neighborhood_radius;
        neighborhood_radius *= neighborhood_size_ratio;
        this->samples = &samples;
        // before run ===============
        sample_densities.clear();
        for (size_t i = 0; i < samples.size(); i ++) {
            sample_densities.push_back(1.0);
        }
        for (int i = 0; i < iter_num; i ++) {
            iterate(i);
        }
        // after run ===============
        this->samples = nullptr;
        this->neighborhood_radius = raw_size;
    }

};

};
