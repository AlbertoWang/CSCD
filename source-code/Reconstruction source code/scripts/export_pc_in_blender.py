import bpy

__all__ = ["export_point_cloud"]

def export_point_cloud(filepath='D:\\pc_exported_from_blender.ply'):
    #print('hello world')
    point_cloud_obj = bpy.context.view_layer.objects.active 
    vertices = point_cloud_obj.data.vertices
    world_mat = point_cloud_obj.matrix_world
    # f = open(filepath, mode='w') 
    # f.write('ply\n')
    # f.write('format ascii 1.0\n')
    # f.write('comment written by export_pc_in_blender.py\n')
    # f.write('element vertex %d\n' % len(vertices))
    # f.write('property float x\n')
    # f.write('property float y\n')
    # f.write('property float z\n')
    # f.write('end_header\n')
    # for vertex in vertices:
    #     co = world_mat @ vertex.co # in Blender 2.78-, use mat * vector
    #     f.write('%.4f %.4f %.4f\n' % (co[0], co[1], co[2]))
    #     pass
    # print("Write %d points to file %s " % (len(vertices), filepath))
    # pass
    with open(filepath, mode='w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write('comment written by export_pc_in_blender.py\n')
        f.write('element vertex %d\n' % len(vertices))
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('end_header\n')
        for vertex in vertices:
            co = world_mat @ vertex.co # in Blender 2.78-, use mat * vector
            f.write('%.4f %.4f %.4f\n' % (co[0], co[1], co[2]))
            pass
        print("Write %d points to file %s " % (len(vertices), filepath))
        pass
    pass