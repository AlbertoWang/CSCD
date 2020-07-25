#include<iostream>
#include<pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include<pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
 
int gp3_reconstruction(char* input_path, char* output_path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCLPointCloud2 cloud_blob;
  //*打开点云文件
  if (pcl::io::loadOBJFile(input_path, cloud_blob) == -1) {
      PCL_ERROR("Couldn't read file\n");
      return(-1);
  }
  pcl::fromPCLPointCloud2(cloud_blob, *cloud);

  //法线估计对象
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  //存储估计的法线
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  //定义kd树指针
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  //估计法线存储到其中
  n.compute(*normals);//Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_width_normals(new pcl::PointCloud<pcl::PointNormal>);
  //链接字段
  pcl::concatenateFields(*cloud, *normals, *cloud_width_normals);

  //定义搜索树对象
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  //点云构建搜索树
  tree2->setInputCloud(cloud_width_normals);

  //定义三角化对象
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  //存储最终三角化的网络模型
  pcl::PolygonMesh triangles;//设置连接点之间的最大距离，（即是三角形最大边长）
  gp3.setSearchRadius(200.0f);
  //设置各种参数值
  gp3.setMu(2.5f);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI_4);
  gp3.setMinimumAngle(M_PI / 18);
  gp3.setMaximumAngle(2 * M_PI / 3);
  gp3.setNormalConsistency(false);

  //设置搜索方法和输入点云
  gp3.setInputCloud(cloud_width_normals);
  gp3.setSearchMethod(tree2);

  //执行重构，结果保存在triangles中
  gp3.reconstruct(triangles);

  //保存网格图
  //pcl::io::saveOBJFile("result.obj", triangles);
  std::string output_dir = output_path;
  std::string sav = "saved mesh in:";
  sav += output_dir;
  pcl::console::print_info(sav.c_str());
  std::cout << std::endl;

  pcl::io::savePLYFile(output_dir.c_str(), triangles);

  // // 显示结果图
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("MAP3D MESH"));
  // ////设置背景;
  // viewer->setBackgroundColor(0, 0, 0);
  // //设置显示的网格
  // viewer->addPolygonMesh(triangles, "my");
  // //viewer->initCameraParameters();
  // while (!viewer->wasStopped()) {
  //     viewer->spin();
  // }

  std::cout << "success" << std::endl;
  return 0;
}