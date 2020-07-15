import polyscope as ps
import numpy as np
import math
import random

# Initialize polyscope
ps.init()

# sampled point simulator
radius = 3
point_num = 40
points = np.zeros((point_num + 4, 3))
for index in range(int(point_num/4)+1):
    x_pos = radius / int(point_num/4) * index * 1.0
    points[index*4] = [x_pos, math.sqrt(radius**2 - x_pos**2), random.uniform(-1.5, 1.5)]
    points[index*4+1] = [x_pos, -math.sqrt(radius**2 - x_pos**2), random.uniform(-1.5, 1.5)]
    points[index*4+2] = [-x_pos, math.sqrt(radius**2 - x_pos**2), random.uniform(-1.5, 1.5)]
    points[index*4+3] = [-x_pos, -math.sqrt(radius**2 - x_pos**2), random.uniform(-1.5, 1.5)]
# skeleton vertex
skeleton_point = np.zeros((1, 3))
skeleton_point_parent = np.zeros((1, 3))
# parent vertex of this skeleton vertex
skeleton_point[0] = [0,0,0]
skeleton_point_parent[0] = [0,0,3]
# edge between verities
horizontal_edge = np.zeros((1,2))
parent_edge = np.zeros((1,2))
point_edge = np.zeros((1,2))
# horizontal point
points[-1] = [0, 3, 0]
# skeleton vertex
points[-2] = [0, 0, 0]
# parent vertex of this skeleton vertex
points[-3] = [0, 0, 3]
# horizontal point to skeleton vertex
horizontal_edge[0] = [point_num+4-1, point_num+4-2]
# parent vertex to skeleton vertex
parent_edge[0] = [point_num+4-3, point_num+4-2]
# sampled point to skeleton vertex
point_edge[0] = [0, point_num+4-2]

# edges to show
horizontal = ps.register_curve_network("horizontal point to skeleton vertex", points, horizontal_edge)
horizontal.set_radius(0.005)
horizontal.set_color((0,1,0))
parent = ps.register_curve_network("parent vertex to skeleton vertex", points, parent_edge)
parent.set_radius(0.005)
parent.set_color((0,0,0))
point = ps.register_curve_network("point to skeleton vertex", points, point_edge)
point.set_radius(0.005)
point.set_color((1,0,0))
# input sampled points
ps.register_point_cloud("points", points)
ps_points = ps.get_point_cloud("points")
ps_points.set_color((1,0,0))
ps_points.set_radius(0.015)
ps.register_point_cloud("points network", points)
# this skeleton vertex
ps.register_point_cloud("skeleton point", skeleton_point)
ps_skeleton_point = ps.get_point_cloud("skeleton point")
ps_skeleton_point.set_color((0,1,0))
ps_skeleton_point.set_radius(0.025)
ps.register_point_cloud("skeleton point", skeleton_point)
# parent vertex of this skeleton vertex
ps.register_point_cloud("parent of skeleton point", skeleton_point_parent)
ps_parent_skeleton_point = ps.get_point_cloud("parent of skeleton point")
ps_parent_skeleton_point.set_color((1,0,1))
ps_parent_skeleton_point.set_radius(0.025)
ps.register_point_cloud("parent of skeleton point", skeleton_point_parent)

ps.show()