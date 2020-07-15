#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <string>
#include "Eigen/Dense"
#include "point_xyz.hpp"
#include "io_xyz.hpp"
#include "geometry_generator_xyz.hpp"

using namespace std;

struct Skeleton {
    std::vector<Point3d> skeleton_points;
    std::vector<Edge> skeleton_edges;
};

struct TriMesh {
    std::vector<Point3d> vertices;
    std::vector<Triangle> faces;
};

struct SkeletonNodeInfo {

    /* the index of the parent node in the skeleton graph, the index of the root node is that of itself*/
    int parent_node_index;

    /* the indices of the children nodes */
    std::vector<int> children;

    /* normalized to a range of 0 to 1 */
    float height; 

    /* indicates the real average distance from each points to the skeleton, 0 - 2 */
    float average_distance; 

    /* the radius of the branch in the generated mesh */
    float branch_radius; 

    /* if the degree of this node is larger than 2, false by default */
    bool is_branching; 

    /* if the degree of this node is 1, false by default */
    bool is_end;

    bool is_root; 

    Point3d tangent_vector;

    /* help determine the */
    //Eigen::RowVector3f mAuxXVector;

    SkeletonNodeInfo():
        parent_node_index(-1), 
        height(0.0), // redundant info
        average_distance(0.0), 
        branch_radius(1.0), 
        is_branching(false), // redundant info
        // is_end(false),  // redundant info
        is_root(false),
        tangent_vector({0.0f, 1.0f, 0.0f})
        //mAuxXVector(0.0f, 0.0f, 0.0f)
    {}
};

void read_radius_info_from_ri(const std::string& filepath, std::unordered_map<int, double>& radius_info) {
    radius_info.clear();
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "Error: fail to open " << filepath << endl;
        return;
    }
    string line;
    int node_id;
    double radius;
    while (std::getline(ifs, line)) {
        istringstream iss(line);
        if (!(iss >> node_id >> radius)) {
            cerr << "Error: failed to read int and double from line: " << line << endl;
            continue;
        }
        radius_info[node_id] = radius;
    }
    std::cout << "Succeed in reading " << radius_info.size() << " id-radius pairs from " << filepath << std::endl;
}

int analyze_skeleton(const Skeleton& skeleton, unordered_map<int, SkeletonNodeInfo>& skeletonInfo, const std::unordered_map<int, double>& radius_info) {

    skeletonInfo.clear();


    //std::cout << "Aynalyzing skeleton... Start to find root and check the validity of skeleton.";
    // find root point
    int rootNodeIndex = 0;
    for (int rowIndex = 0; rowIndex < skeleton.skeleton_points.size(); rowIndex++) {
        if (skeleton.skeleton_points.at(rowIndex).y < skeleton.skeleton_points.at(rootNodeIndex).y) {
            rootNodeIndex = rowIndex;
        }
    }
    skeletonInfo[rootNodeIndex].is_root = true;
    skeletonInfo[rootNodeIndex].height = 0.0f;
    skeletonInfo[rootNodeIndex].parent_node_index = rootNodeIndex;

    // build a simple data structure that represents for a undirected graph
    unordered_map<int, vector<int> > edgeMap;
    for (int rowIndex = 0; rowIndex < skeleton.skeleton_edges.size(); rowIndex++) {
        int va = skeleton.skeleton_edges.at(rowIndex).vi;
        int vb = skeleton.skeleton_edges.at(rowIndex).vj;
        if (find(edgeMap[va].begin(), edgeMap[va].end(), vb) == edgeMap[va].end()) {
            edgeMap[va].push_back(vb);
        }
        if (find(edgeMap[vb].begin(), edgeMap[vb].end(), va) == edgeMap[vb].end()) {
            edgeMap[vb].push_back(va);
        }
    }

    // check if contains cycle
    bool containsCycle = false;
    vector<bool> visitMap(skeleton.skeleton_points.size());
    vector<pair<int, int>> visitQueue;;; visitQueue.push_back({rootNodeIndex, -1});
    for (size_t i = 0; i < visitMap.size(); i++) visitMap[i] = false;
    for (size_t i = 0; i < visitQueue.size(); i++) {
        int currNodeIndex = visitQueue[i].first;
        if (visitMap[currNodeIndex]) { 
            containsCycle = true; 
            std::cout << "Cycle is located at (" << skeleton.skeleton_points.at(currNodeIndex).x
                << ", " << skeleton.skeleton_points.at(currNodeIndex).y
                << ", " << skeleton.skeleton_points.at(currNodeIndex).z << ")";
            break; 
        }
        visitMap[currNodeIndex] = true;
        for (int adj : edgeMap[currNodeIndex]) {
            if (adj == visitQueue[i].second) continue;;; // parent
            visitQueue.push_back({ adj, currNodeIndex });
        }
    }
    if (containsCycle) {
        std::cerr << "Cycle is found in the skeleton.... stop analyzing skeleton" << std::endl;
        return -1;
    }

    std::cout << "Start to mark parents." << std::endl;
    // mark parents
    vector<int> nodeIndicesQueue;
    nodeIndicesQueue.reserve(static_cast<int>(skeleton.skeleton_points.size() * 1.5));
    nodeIndicesQueue.push_back(rootNodeIndex);
    int currPointer = 0;
    float maxHeight = 0.0;
    while (currPointer < nodeIndicesQueue.size()) {
        int currNodeIndex = nodeIndicesQueue[currPointer];
        Point3d childrenMedian(0, 0, 0);
        Point3d parentNode = skeleton.skeleton_points.at(skeletonInfo[currNodeIndex].parent_node_index);
        int childCnt = 0;
        for (int adjacentNodeIndex : edgeMap.at(currNodeIndex)) {
            if (adjacentNodeIndex == skeletonInfo[currNodeIndex].parent_node_index) continue;;;

            nodeIndicesQueue.push_back(adjacentNodeIndex);
            skeletonInfo[adjacentNodeIndex].parent_node_index = currNodeIndex;
            skeletonInfo[adjacentNodeIndex].height = skeletonInfo[currNodeIndex].height + 1.0f;
            skeletonInfo[currNodeIndex].children.push_back(adjacentNodeIndex);
            maxHeight = std::max(maxHeight, skeletonInfo[adjacentNodeIndex].height);

            // sum it to help compute the tangent vector
            childrenMedian = childrenMedian + skeleton.skeleton_points.at(adjacentNodeIndex);
            childCnt += 1;
        }
        if (childCnt > 0) childrenMedian = childrenMedian / childCnt; else childrenMedian = skeleton.skeleton_points.at(currNodeIndex);
        Point3d tangentVector = childrenMedian - parentNode;
        tangentVector.normalize();
        //std::cout << "Degree of node " << currNodeIndex << " is " << edgeMap[currNodeIndex].size();
        skeletonInfo[currNodeIndex].is_branching = (edgeMap.at(currNodeIndex).size() > 2);
        // skeletonInfo[currNodeIndex].mIsEnd = (skeletonInfo[currNodeIndex].mChildren.empty());
        skeletonInfo[currNodeIndex].tangent_vector = tangentVector;
        currPointer += 1;
    }

    std::cout << "Start to marking height.";
    // compute the height & mean distance
    // refs: https://geos.osgeo.org/doxygen/classgeos_1_1algorithm_1_1Distance.html
    maxHeight = std::max(1.0f, maxHeight);
    // const int numParticipateComputeDistance = pointCloud->mVertices.rows() / skeleton.skeleton_points.rows() * 1.5;
    // vector<int> neighborsInPointCloud;

    for (const auto& kv : edgeMap) {
        const int nodeIndex = kv.first;
        //skeletonInfo[nodeIndex].average_distance = 0.5;
        skeletonInfo[nodeIndex].average_distance = radius_info.at(nodeIndex);
    }
    return rootNodeIndex;
}

void generate_branch_geometry(const Skeleton& skeleton, const vector<int>& branch_node_indices, unordered_map<int, SkeletonNodeInfo>& skeleton_info, std::vector<Point3d>& branch_mesh_vertices, std::vector<Triangle>& branch_mesh_faces) {
    using namespace Eigen;
    if (branch_node_indices.size() < 2) {
        std::cout << "Too less skeleton points in a skeleton branch\n";
        return;
    }
    branch_mesh_vertices.clear();
    branch_mesh_faces.clear();
    // vector<Point3d> branch_mesh_vertices;
    // vector<Triangle> branch_mesh_faces;

    // compute triangle size
    // const auto boundingBoxMat = CommonUtilities::computeBoundingBox(pointCloud);
    // const double diagonalLength = (boundingBoxMat.row(1) - boundingBoxMat.row(0)).norm();
    //double triangle_size = diagonalLength / pow(pointCloud->mVertices.rows(), 0.5);
    double adj_node_distance_sum = 0.0;
    for (int i = 0; i + 1 < branch_node_indices.size(); i ++) {
        auto& prev_node = skeleton.skeleton_points[branch_node_indices[i]];
        auto& next_node = skeleton.skeleton_points[branch_node_indices[i + 1]];
        adj_node_distance_sum += (prev_node - next_node).norm();
    }
    double triangle_size = adj_node_distance_sum / branch_node_indices.size() * 0.8;

    // generate a standrad cylinder
    // - determine some parameter for the cylinder
    // const double loop_ratio = GETD(GT_LOOP_RATIO);
    // const double sample_ratio = GETD(GT_SAMPLE_RATIO);
    const double loop_ratio = 10;
    const double sample_ratio = 10;

    const Point3d bottom_sp = skeleton.skeleton_points.at(branch_node_indices[0]);
    const Point3d top_sp = skeleton.skeleton_points.at(branch_node_indices.back());
    const int loop_count = std::max(static_cast<int>((top_sp - bottom_sp).norm() / triangle_size * loop_ratio), 2);

    const int standardSampleCountPerLoop = 24 * 2 * sample_ratio;
    const int minimalSampleCountPerLoop = 6;
    const int sample_count_per_loopCandidate = std::max(
        static_cast<int>(standardSampleCountPerLoop * (1.0 - skeleton_info.at(branch_node_indices[0]).height)),
        minimalSampleCountPerLoop);
    const int sample_count_per_loop = (sample_count_per_loopCandidate % 2 == 1) ? sample_count_per_loopCandidate + 1 : sample_count_per_loopCandidate;
    // - implementation for generation of cylinder

    zjl::Geometry::generate_cylinder(loop_count, sample_count_per_loop, 1.0f, 1.0f, branch_mesh_vertices, branch_mesh_faces);
    // auto branchMesh = GeometryUtilities::generateCylinder(loop_count, sample_count_per_loop, 1.0f, 1.0f);


    // transform the cylinder to make it roughly fitted to the point cloud
    // - scaling it
    float average_distance = skeleton_info.at(branch_node_indices[0]).average_distance;
    //auto scalingMat = Scaling<float>(average_distance, (top_sp - bottom_sp).norm(), average_distance);
    //branch_mesh_vertices_eigen = scalingMat * branch_mesh_vertices_eigen;


    // ===========================================
#define NODE_ON_BRANCH_AT(i) \
skeleton.skeleton_points.at(branch_node_indices[i])
    // ===========================================

    float branchSkeletonLength = 0.0f;
    for (int i = 0; i + 1 < branch_node_indices.size(); i++) {
        branchSkeletonLength += (NODE_ON_BRANCH_AT(i) - NODE_ON_BRANCH_AT(i + 1)).norm();
    }

    int nodeIndexI = 0;
    double currLength = 0.0;

    for (auto& point : branch_mesh_vertices) point.y = 0.0;
    Eigen::MatrixXf branch_mesh_vertices_eigen(3, branch_mesh_vertices.size());
    for (size_t i = 0; i < branch_mesh_vertices.size(); i ++) {
        auto& point = branch_mesh_vertices.at(i);
        branch_mesh_vertices_eigen(0, i) = point.x;
        branch_mesh_vertices_eigen(1, i) = point.y;
        branch_mesh_vertices_eigen(2, i) = point.z;
    }
    // for (int colIndex = 0; colIndex < branch_mesh_vertices_eigen.cols(); colIndex++) {
    //     branch_mesh_vertices_eigen(1, colIndex) = 0;
    // }

    __NOTE_1:
    // an ugly hack, because the tangent vec of a branching node is not that exactly
    const Point3d tRawTangenetCopy0 = skeleton_info.at(branch_node_indices[0]).tangent_vector;
    //const RowVector3f tRawTangenetCopyT = skeleton_info.at(branch_node_indices.back()).tangent_vector;
    skeleton_info[branch_node_indices[0]].tangent_vector = NODE_ON_BRANCH_AT(1) - NODE_ON_BRANCH_AT(0);
    //skeleton_info[branch_node_indices[0]].tangent_vector = NODE_ON_BRANCH_AT(1) - NODE_ON_BRANCH_AT(0);

    // determine init x/y/z
    const Point3d tInitialYAxis = skeleton_info.at(branch_node_indices[0]).tangent_vector;
    const float tYAxisZ = fabs(tInitialYAxis.z) < 1e-7 ? 1e-3 : tInitialYAxis.z;
    Point3d tInitialXAxis(tInitialYAxis.z, tInitialYAxis.y + 1.0, 
        -(tInitialYAxis.x * tInitialYAxis.x + tInitialYAxis.y * (tInitialYAxis.y + 1.0)) / tYAxisZ);
    tInitialXAxis.normalize();
    Point3d tPrevXAxis(tInitialXAxis);

    // start to transform each loop
    for (int i = 0; i < loop_count; i++) {
        float targetLengthRatio = i * 1.0 / (loop_count - 1);
        float currSegmentLength = (NODE_ON_BRANCH_AT(nodeIndexI) - NODE_ON_BRANCH_AT(nodeIndexI + 1)).norm();
        while (!(targetLengthRatio >= currLength / branchSkeletonLength && targetLengthRatio <= (currLength + currSegmentLength) / branchSkeletonLength)
            && nodeIndexI + 2 < branch_node_indices.size()) {
            nodeIndexI += 1;
            currLength += currSegmentLength;
            currSegmentLength = (NODE_ON_BRANCH_AT(nodeIndexI) - NODE_ON_BRANCH_AT(nodeIndexI + 1)).norm();
        }
        float tRatio = (targetLengthRatio * branchSkeletonLength - currLength) / currSegmentLength;

        // scale
        float currAverageDistance =
            skeleton_info.at(branch_node_indices[nodeIndexI]).average_distance * (1.0 - tRatio) +
            skeleton_info.at(branch_node_indices[nodeIndexI + 1]).average_distance * tRatio;
        //if (i == 0) currAverageDistance *= 0.6;
        Matrix3f scalingMat; scalingMat <<
            currAverageDistance,        0,          0,
            0,                          1.0f,       0,
            0,                          0,      currAverageDistance
            ;
        branch_mesh_vertices_eigen
        //branch_mesh_vertices_eigen
                .block(0, i * sample_count_per_loop, 3, sample_count_per_loop)
            =
                scalingMat * branch_mesh_vertices_eigen //branch_mesh_vertices_eigen
                .block(0, i * sample_count_per_loop, 3, sample_count_per_loop);

        // - rotate and translate
        Point3d localCenter = NODE_ON_BRANCH_AT(nodeIndexI) +
            (NODE_ON_BRANCH_AT(nodeIndexI + 1) - NODE_ON_BRANCH_AT(nodeIndexI)) * tRatio;

        Point3d localNormal =
            skeleton_info.at(branch_node_indices[nodeIndexI]).tangent_vector * (1.0 - tRatio) +
            skeleton_info.at(branch_node_indices[nodeIndexI + 1]).tangent_vector * tRatio;
        if (fabs(localNormal.dot(tPrevXAxis) - localNormal.norm() * tPrevXAxis.norm()) < 1e-7) {
            std::cerr << "Previous local X-axis is parallel to the new Y-Axis, please do something..." << std::endl;
            // TODO ...
        }
        // solve following system, (u, v is scalar)
        // 1. (u��n + v��p)n = 0
        // 2. u + v = 1
        Matrix2f tSolveLxA; tSolveLxA << 1.0f, localNormal.dot(tPrevXAxis), 1.0f, 1.0f;
        Vector2f tSolveLxb; tSolveLxb << 0.0f, 1.0f;
        Vector2f tSolveLxX = tSolveLxA.inverse() * tSolveLxb; // solve u, v

        //RowVector3f ly = localNormal;  
        RowVector3f tPrevXAxisEigen;
        tPrevXAxisEigen(0) = tPrevXAxis.x;
        tPrevXAxisEigen(1) = tPrevXAxis.y;
        tPrevXAxisEigen(2) = tPrevXAxis.z;
        RowVector3f ly; ly(0) = localNormal.x; ly(1) = localNormal.y; ly(2) = localNormal.z;  
        ly.normalize();

        RowVector3f lx = ly * tSolveLxX(0) + tPrevXAxisEigen * tSolveLxX(1); lx.normalize();

        //??????????
        //tPrevXAxis = lx;
        auto lz = ly.cross(lx);         lz.normalize();

        Matrix4f rotateMat; rotateMat <<
            lx(0), ly(0), lz(0), localCenter.x,
            lx(1), ly(1), lz(1), localCenter.y,
            lx(2), ly(2), lz(2), localCenter.z,
            0, 0, 0, 1
            ;

        branch_mesh_vertices_eigen
            .block(0, i * sample_count_per_loop, 3, sample_count_per_loop)
            =
            (rotateMat * branch_mesh_vertices_eigen
                .block(0, i * sample_count_per_loop, 3, sample_count_per_loop)
                .colwise()
                .homogeneous())
            .colwise()
            .hnormalized();
        if (i == 0) {
            branch_mesh_vertices_eigen
                .block(0, branch_mesh_vertices.size() - 2, 3, 1)
                =
                (rotateMat * branch_mesh_vertices_eigen
                    .block(0, branch_mesh_vertices.size() - 2, 3, 1)
                    .colwise()
                    .homogeneous())
                .colwise()
                .hnormalized();
        } else if (i + 1 == loop_count) {
            branch_mesh_vertices_eigen
                .block(0, branch_mesh_vertices.size() - 1, 3, 1)
                =
                (rotateMat * branch_mesh_vertices_eigen
                    .block(0, branch_mesh_vertices.size() - 1, 3, 1)
                    .colwise()
                    .homogeneous())
                .colwise()
                .hnormalized();
        }
    }
    //branchMesh->mVertices = branch_mesh_vertices_eigen.transpose();
    for (size_t i = 0; i < branch_mesh_vertices.size(); i ++) {
        auto& point = branch_mesh_vertices[i];
        point.x = branch_mesh_vertices_eigen(0, i);
        point.y = branch_mesh_vertices_eigen(1, i);
        point.z = branch_mesh_vertices_eigen(2, i);
    }

    skeleton_info[branch_node_indices[0]].tangent_vector = tRawTangenetCopy0;
    //return branchMesh;
#undef NODE_ON_BRANCH_AT
}

void generate_tree_geometry_based_on_skel(const Skeleton& skeleton, const unordered_map<int, double>& radius_info, TriMesh& tree_mesh) {
    tree_mesh.vertices.clear();
    tree_mesh.faces.clear();
    // zjl::Geometry::generate_cylinder(10, 8, 0.8f, 1.5f, tree_mesh.vertices, tree_mesh.faces);


    // analyze the skeleton
    unordered_map<int, SkeletonNodeInfo> skeletonInfo;
    int rootNodeIndex = analyze_skeleton(skeleton, skeletonInfo, radius_info);
    //int rootNodeIndex = analyzeSkeleton(pointCloud, skeleton, skeletonInfo);
    if (rootNodeIndex < 0) {
        std::cerr << "Fail to analyze the input skeleton. quit" << std::endl;
        return;
        //return nullptr;
    }

    // re-distribute average distance of point cloud to the skeleton

    // draw root point
    // RowVector3f rootPoint = skeleton.skeleton_points.row(rootNodeIndex);
    // DataPointer rootPointData = make_shared<pcp::Data>(DataType::POINT_CLOUD);
    // rootPointData->mVertices.resize(1, 3);
    // rootPointData->mVertices.row(0) = rootPoint;
    // Platform::getRendererManager().draw("root", rootPointData);

    // find a branch
    vector<vector<int>> branchPointIndicesList;
    vector<int> branchStartNodes;
    for (int nodeIndex = 0; nodeIndex < skeleton.skeleton_points.size(); nodeIndex++) {
        if (skeletonInfo[nodeIndex].is_branching || skeletonInfo[nodeIndex].is_root) {
            branchStartNodes.push_back(nodeIndex);
        }
    }
    std::cout << "Num of branches: " << branchStartNodes.size();
    for (int startNodeIndex : branchStartNodes) {
        for (int firstChild : skeletonInfo[startNodeIndex].children) {
            vector<int> branchPointIndices;
            branchPointIndices.push_back(startNodeIndex);
            int currNode = firstChild;
            while (skeletonInfo[currNode].children.size() == 1) {
                branchPointIndices.push_back(currNode);
                currNode = skeletonInfo[currNode].children.front();
            }
            branchPointIndices.push_back(currNode);
            branchPointIndicesList.push_back(branchPointIndices);
        }
    }

    // prepare data
    //vector<DataPointer> meshes;
    unordered_map<int, int> i2i;
    for (int branchIndex = 0; branchIndex < branchPointIndicesList.size(); branchIndex++) {
        std::vector<Point3d> branch_mesh_vertices;
        std::vector<Triangle> branch_mesh_faces;
        const auto& branchPointIndices = branchPointIndicesList[branchIndex];
        //MatrixXf branchVerticesData(branchPointIndices.size(), 3);
        // for (int i = 0; i < branchPointIndices.size(); i++) {
        //     branchVerticesData.row(i) = skeleton.skeleton_points.row(branchPointIndices[i]);
        // }

        generate_branch_geometry(skeleton, branchPointIndices, skeletonInfo, branch_mesh_vertices, branch_mesh_faces);
        //auto mesh = generateBranchImpl(skeleton, branchPointIndices, skeletonInfo);
        //meshes.push_back(mesh);
        
        // ========
        // Merge
        for (size_t i = 0; i < branch_mesh_vertices.size(); i ++) {
            const auto& branch_point = branch_mesh_vertices[i];
            i2i[i] = tree_mesh.vertices.size();
            tree_mesh.vertices.push_back(branch_point);
        }
        for (size_t i = 0; i < branch_mesh_faces.size(); i ++) {
            auto& branch_face = branch_mesh_faces[i];
            branch_face.vi = i2i[branch_face.vi];
            branch_face.vj = i2i[branch_face.vj];
            branch_face.vk = i2i[branch_face.vk];
            tree_mesh.faces.push_back(branch_face);
        }
        std::cout << "Generate one branch " << branchIndex << std::endl;
    }
    //auto mergedMesh = GeometryUtilities::mergeMeshes(meshes);
    //Platform::getRendererManager().draw("test_branch", mergedMesh);
    //return mergedMesh;
}

int main(int argc, char** argv) {
    // ./xxx.exe mve-scan-5-gt.obj mve-scan-5.ri mve-scan-5.ply
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " xxx.obj(skeleton) xxx.ri(radius info) xxx.ply(triangle mesh)" << endl;
        return -1;
    }
    Skeleton skel;
    TriMesh tree_mesh;
    std::unordered_map<int, double> radius_info;
    zjl::IO::read_graph_from_obj(argv[1], skel.skeleton_points, skel.skeleton_edges);
    read_radius_info_from_ri(argv[2], radius_info);
    generate_tree_geometry_based_on_skel(skel, radius_info, tree_mesh);
    zjl::IO::write_trimesh_to_ply_without_normals(argv[3], tree_mesh.vertices, tree_mesh.faces);
    return 0;
}