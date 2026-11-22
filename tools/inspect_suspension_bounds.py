import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    f.seek(12)
    chunk_len = int.from_bytes(f.read(4), 'little')
    f.seek(20)
    gltf = json.loads(f.read(chunk_len).decode('utf-8'))

# Node 40 is Wheels_objs
w_node = gltf['nodes'][40]
print("Wheels node translation:", w_node.get('translation'))
# Mesh 22
w_mesh = gltf['meshes'][w_node['mesh']]
print("Wheels primitives:", len(w_mesh['primitives']))
for i, prim in enumerate(w_mesh['primitives']):
    pos_acc = gltf['accessors'][prim['attributes']['POSITION']]
    print(f"Prim {i}: min={pos_acc.get('min')}, max={pos_acc.get('max')}")

# Node 41 is suspension
s_node = gltf['nodes'][41]
print("\nSuspension node translation:", s_node.get('translation'))
s_mesh = gltf['meshes'][s_node['mesh']]
print("Suspension primitives:", len(s_mesh['primitives']))
for i, prim in enumerate(s_mesh['primitives']):
    pos_acc = gltf['accessors'][prim['attributes']['POSITION']]
    print(f"Prim {i}: min={pos_acc.get('min')}, max={pos_acc.get('max')}")
