import bpy
import os
import argparse
from random import seed, random


def synthesize_tree_models(directory, num, skel_dir):
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

    # small tree
    for index in range(num):
        new_file_path = '%s/tree_small_%d.ply' % (directory, index)
        skel_file_path = '%s/tree_small_%d.obj' % (skel_dir, index)
        param_node.mesh_type = 'preview'
        trunk_node.length = random() * 5 + 10
        branch_node.amount = random() * 6 + 10
        branch_node.gravity = random() * 0.5 + 0.10
        branch_node.length = random() * 4 + 6

        bpy.ops.mtree.randomize_tree(node_group_name=group_label)
        #bpy.ops.object.mtree_execute_tree(node_group_name=group_label)

        tree_obj = None 
        for obj in bpy.data.objects: 
            if obj.type == 'CURVE': tree_obj = obj # find the curve object
        if tree_obj is None:
            print("Error: Cannot find the curve object!")
            return


        tree_obj.data.bevel_depth = 0
        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')
        tree_obj.select_set(True) # Note: it cannot be omitted!
        bpy.context.view_layer.objects.active = tree_obj

        bpy.ops.object.convert(target='MESH')
        bpy.ops.export_scene.obj(filepath=skel_file_path, axis_up='Y', axis_forward='Z')

        param_node.mesh_type = 'final'
        bpy.ops.object.mtree_execute_tree(node_group_name=group_label)
        #tree_obj = bpy.data.objects['tree'] # tree object has auto-renamed, reselect the object
        tree_obj = None 
        for obj in bpy.data.objects: 
            if obj.type == 'MESH': tree_obj = obj # find the curve object
        if tree_obj is None:
            print("Error: Cannot find the curve object!")
            return

        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        tree_obj.select_set(True)
        bpy.context.view_layer.objects.active = tree_obj # Note: it cannot be omitted!

        export_result_set = bpy.ops.export_mesh.ply(filepath=new_file_path, use_mesh_modifiers=False, use_uv_coords=False, use_colors=False, axis_up='Y', axis_forward='Z')

        if 'FINISHED' not in export_result_set:
            print ("Warning: failed to export ply ...")
            pass

    # medium tree
    for index in range(num):
        new_file_path = '%s/tree_medium_%d.ply' % (directory, index)
        skel_file_path = '%s/tree_medium_%d.obj' % (skel_dir, index)
        param_node.mesh_type = 'preview'
        trunk_node.length = random() * 4 + 12
        branch_node.amount = random() * 16 + 14
        branch_node.gravity = random() * 0.4 + 0.10
        branch_node.length = random() * 4 + 9
        bpy.ops.mtree.randomize_tree(node_group_name=group_label)
        #bpy.ops.object.mtree_execute_tree(node_group_name=group_label)

        tree_obj = None 
        for obj in bpy.data.objects: 
            if obj.type == 'CURVE': tree_obj = obj # find the curve object
        if tree_obj is None:
            print("Error: Cannot find the curve object!")
            return


        tree_obj.data.bevel_depth = 0
        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')
        tree_obj.select_set(True) # Note: it cannot be omitted!
        bpy.context.view_layer.objects.active = tree_obj

        bpy.ops.object.convert(target='MESH')
        bpy.ops.export_scene.obj(filepath=skel_file_path, axis_up='Y', axis_forward='Z')

        param_node.mesh_type = 'final'
        bpy.ops.object.mtree_execute_tree(node_group_name=group_label)
        #tree_obj = bpy.data.objects['tree'] # tree object has auto-renamed, reselect the object
        tree_obj = None 
        for obj in bpy.data.objects: 
            if obj.type == 'MESH': tree_obj = obj # find the curve object
        if tree_obj is None:
            print("Error: Cannot find the curve object!")
            return

        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        tree_obj.select_set(True)
        bpy.context.view_layer.objects.active = tree_obj # Note: it cannot be omitted!

        export_result_set = bpy.ops.export_mesh.ply(filepath=new_file_path, use_mesh_modifiers=False, use_uv_coords=False, use_colors=False, axis_up='Y', axis_forward='Z')

        if 'FINISHED' not in export_result_set:
            print ("Warning: failed to export ply ...")
            pass

    # large tree
    for index in range(num):
        new_file_path = '%s/tree_large_%d.ply' % (directory, index)
        skel_file_path = '%s/tree_large_%d.obj' % (skel_dir, index)
        param_node.mesh_type = 'preview'
        trunk_node.length = random() * 8 + 16
        branch_node.amount = random() * 18 + 22
        branch_node.gravity = random() * 0.5 + 0.10
        branch_node.length = random() * 6 + 14
        bpy.ops.mtree.randomize_tree(node_group_name=group_label)
        #bpy.ops.object.mtree_execute_tree(node_group_name=group_label)

        tree_obj = None 
        for obj in bpy.data.objects: 
            if obj.type == 'CURVE': tree_obj = obj # find the curve object
        if tree_obj is None:
            print("Error: Cannot find the curve object!")
            return


        tree_obj.data.bevel_depth = 0
        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')
        tree_obj.select_set(True) # Note: it cannot be omitted!
        bpy.context.view_layer.objects.active = tree_obj

        bpy.ops.object.convert(target='MESH')
        bpy.ops.export_scene.obj(filepath=skel_file_path, axis_up='Y', axis_forward='Z')

        param_node.mesh_type = 'final'
        bpy.ops.object.mtree_execute_tree(node_group_name=group_label)
        #tree_obj = bpy.data.objects['tree'] # tree object has auto-renamed, reselect the object
        tree_obj = None 
        for obj in bpy.data.objects: 
            if obj.type == 'MESH': tree_obj = obj # find the curve object
        if tree_obj is None:
            print("Error: Cannot find the curve object!")
            return

        # check poll() to avoid exception.
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        tree_obj.select_set(True)
        bpy.context.view_layer.objects.active = tree_obj # Note: it cannot be omitted!

        export_result_set = bpy.ops.export_mesh.ply(filepath=new_file_path, use_mesh_modifiers=False, use_uv_coords=False, use_colors=False, axis_up='Y', axis_forward='Z')

        if 'FINISHED' not in export_result_set:
            print ("Warning: failed to export ply ...")
            pass
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--background', action='store_true')
    parser.add_argument('--python', type=str, default="xxx.py", help='please ignore this option')
    parser.add_argument('--num', type=int, default=10, help='the num of trees')
    parser.add_argument('--dir', type=str, default='mesh', help='the directory of the synthetic trees')
    parser.add_argument('--skel_dir', type=str, default='mesh_skel', help='the directory of the skeletons of synthetic trees')
    #parser.add_argument('--translation_x', type=float, default=0.0, help='the x-coord of the translation')
    argument_vals = parser.parse_args()
    synthesize_tree_models(directory=argument_vals.dir, num=argument_vals.num, skel_dir=argument_vals.skel_dir)
    pass
