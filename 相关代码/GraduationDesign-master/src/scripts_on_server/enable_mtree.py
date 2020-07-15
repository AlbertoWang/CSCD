import bpy

if __name__ == '__main__':
    bpy.ops.preferences.addon_enable(module="modular_tree-blender_28")
    bpy.ops.wm.save_userpref()
    pass
