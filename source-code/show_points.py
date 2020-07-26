import sys
import polyscope as ps
import numpy as np
def show_points_in_obj_file(obj_file):
    ps.init()
    point_xyz = []
    with open(obj_file) as f_in:
        for line in f_in:
            xyz_list = line.split(' ')
            point_xyz.append(np.array([float(xyz_list[1]), float(xyz_list[2]), float(xyz_list[3])]))
    points = np.zeros((len(point_xyz), 3))
    for i in range(len(point_xyz)):
        points[i] = point_xyz[i]
    ps.register_point_cloud("points", points)
    ps_points = ps.get_point_cloud("points")
    ps_points.set_radius(0.0015)
    ps.set_autoscale_structures(True)
    ps.show()

def show_skeleton_in_obj_file(obj_file):
    ps.init()
    vertex_pos = []
    line_indices = []
    with open(obj_file) as f_in:
        for line in f_in:
            temp = line.split(' ')
            if temp[0] == 'v':
                vertex_pos.append(np.array([float(temp[1]), float(temp[2]), float(temp[3])]))
            if temp[0] == 'l':
                line_indices.append(np.array([int(temp[1]) - 1, int(temp[2]) - 1]))
    points = np.array(vertex_pos)
    edges = np.array(line_indices)
    skeleton = ps.register_curve_network("skeleton curve", points, edges)
    point_cloud = ps.register_point_cloud("point cloud", points)
    skeleton.set_radius(0.001)
    point_cloud.set_radius(0.005)
    point_cloud.set_color((1,0,0))
    ps.set_autoscale_structures(True)
    ps.show()

def show_mesh_in_ply_file(obj_file):
    ps.init()
    vertex_pos = []
    face_indices = []
    flag = False
    with open(obj_file) as f_in:
        for line in f_in:
            temp = line.split(' ')
            if line == 'end_header\n':
                print('loading data')
                flag = True
                continue
            if flag == True:
                if temp[0] == '3':
                    face_indices.append(np.array([int(temp[1]), int(temp[2]), int(temp[3])]))
                else:
                    vertex_pos.append(np.array([float(temp[0]), float(temp[1]), float(temp[2])]))
    points = np.array(vertex_pos)
    faces = np.array(face_indices)
    ps_mesh = ps.register_surface_mesh("result mesh", points, faces)
    ps.set_autoscale_structures(True)
    ps.show()

def show_points_and_skeleton(point_file, skeleton_file):
    ps.init()
    point_xyz = []
    with open(point_file) as f_in:
        for line in f_in:
            xyz_list = line.split(' ')
            point_xyz.append(np.array([float(xyz_list[1]), float(xyz_list[2]), float(xyz_list[3])]))
    points = np.zeros((len(point_xyz), 3))
    for i in range(len(point_xyz)):
        points[i] = point_xyz[i]
    ps.register_point_cloud("points", points)
    ps_points = ps.get_point_cloud("points")
    ps_points.set_radius(0.0015)

    vertex_pos = []
    line_indices = []
    with open(skeleton_file) as f_in:
        for line in f_in:
            temp = line.split(' ')
            if temp[0] == 'v':
                vertex_pos.append(np.array([float(temp[1]), float(temp[2]), float(temp[3])]))
            # if temp[0] == 'l':
            #     line_indices.append(np.array([int(temp[1]) - 1, int(temp[2]) - 1]))
    points = np.array(vertex_pos)
    # edges = np.array(line_indices)
    # skeleton = ps.register_point_cloud("skeleton curve", points, edges)
    skeleton = ps.register_point_cloud("skeleton curve", points)
    skeleton.set_radius(0.005)
    skeleton.set_color((1,0,0))
    ps.set_autoscale_structures(True)
    ps.show()

if len(sys.argv) < 3:
    print("show point cloud in obj:\npc xxx.obj")
    print("show skeleton in obj:\nskel xxx.obj")
    print("show mesh in ply:\nmesh xxx.ply")
    print("show point cloud and skeleton in obj:\npcs point_file.obj skeleton_file.obj")
    sys.exit()
if sys.argv[1] == 'pc':
    show_points_in_obj_file(sys.argv[2])
elif sys.argv[1] == 'skel':
    show_skeleton_in_obj_file(sys.argv[2])
elif sys.argv[1] == 'mesh':
    show_mesh_in_ply_file(sys.argv[2])
elif sys.argv[1] == 'pcs':
    show_points_and_skeleton(sys.argv[2], sys.argv[3])
else:
    print("show point cloud in obj:\npc xxx.obj")
    print("show skeleton in obj:\nskel xxx.obj")
    print("show mesh in ply:\nmesh xxx.ply")
    print("show point cloud and skeleton in obj:\npcs point_file.obj skeleton_file.obj")
    sys.exit()