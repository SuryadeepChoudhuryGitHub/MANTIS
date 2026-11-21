import struct
import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))

nodes = gltf.get('nodes', [])
meshes = gltf.get('meshes', [])

print(f"Total nodes: {len(nodes)}, Total meshes: {len(meshes)}")
for i, n in enumerate(nodes):
    name = n.get('name', '')
    mesh_idx = n.get('mesh', -1)
    mesh_name = meshes[mesh_idx].get('name', '') if mesh_idx >= 0 and mesh_idx < len(meshes) else ''
    print(f"[{i:2d}] node='{name}' mesh={mesh_idx} ('{mesh_name}') children={n.get('children', [])}")
