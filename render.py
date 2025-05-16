import bpy

import sys
argv = sys.argv
argv = argv[argv.index("--") + 1:]

infile = argv[0]
outfile = argv[1]

bpy.context.scene.render.filepath = outfile
bpy.ops.import_scene.gltf(filepath=infile)
ob = bpy.context.selected_objects[0]
# ob.data.materials[0] = bpy.data.materials["red"]
bpy.ops.render.render(write_still=True)
bpy.ops.object.delete()
