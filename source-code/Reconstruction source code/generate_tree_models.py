import bpy
import os
import argparse
from random import seed, random

def synthesize_tree_models(directory, num):
    # remove the default cube model
    cube_obj = bpy.data.meshes['Cube']
    bpy.data.meshes.remove(cube_obj)

    # test if mtree addon exist 
    if 'mtree' not in dir(bpy.ops):
        print('The addon mtree is not found. Exit')
        return
    group_label = 'syn'
    mtree_type = 'mtree_node_tree'
    synthesis_group = bpy.data.node_groups.new(name=group_label, type=mtree_type) 
    trunk_node = synthesis_group.nodes.new(type='MtreeTrunk')
    branch_node = synthesis_group.nodes.new(type='MtreeBranch')
    param_node = synthesis_group.nodes.new(type='MtreeParameters')
    param_node.mesh_type = 'final'
    synthesis_group.links.new(input=trunk_node.outputs[0], output=branch_node.inputs[0])

    seed(2)
    for index in range(num):
        trunk_node.length = random() * 20 + 10
        branch_node.amount = random() * 30 + 25
        branch_node.gravity = random() * 0.5 + 0.10
        branch_node.length = random() * 16 + 10
        bpy.ops.mtree.randomize_tree(node_group_name=group_label)
        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        #result_set = bpy.ops.import_mesh.ply(filepath="./dummy_cube.ply")
        #result_set = bpy.ops.import_mesh.ply(filepath="./tree-synthesis-1-gt.ply")

        tree_obj = bpy.data.objects['tree']
        new_file_path = './tree_%d.ply' % index
        tree_obj.select_set(True)
        # print('num of vertices: ' + str(len(tree_obj.data.vertices)))

        bpy.context.view_layer.objects.active = tree_obj

        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        export_result_set = bpy.ops.export_mesh.ply(filepath=new_file_path, use_mesh_modifiers=False, use_uv_coords=False, use_colors=False, axis_up='Y')

        print(dir(tree_obj.data))
        if 'FINISHED' not in export_result_set:
            print ("Warning: failed to export ply ...")
            pass


    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--background', action='store_true')
    parser.add_argument('--python', type=str, default="xxx.py", help='please ignore this option')
    parser.add_argument('--num', type=int, default=10, help='the num of trees')
    parser.add_argument('--dir', type=str, default='models', help='the directory of the synthetic trees')
    #parser.add_argument('--translation_x', type=float, default=0.0, help='the x-coord of the translation')
    #parser.add_argument('--translation_y', type=float, default=0.0, help='the y-coord of the translation')
    #parser.add_argument('--translation_z', type=float, default=0.0, help='the z-coord of the translation')
    argument_vals = parser.parse_args()
    synthesize_tree_models(directory=argument_vals.dir, num=argument_vals.num)
    pass