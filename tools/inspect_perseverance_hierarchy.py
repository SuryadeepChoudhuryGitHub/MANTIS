import json
import struct
import sys

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<4sII', f.read(12))
    chunk_len, chunk_type = struct.unpack('<I4s', f.read(8))
    json_bytes = f.read(chunk_len)
    data = json.loads(json_bytes.decode('utf-8'))

nodes = data.get('nodes', [])
meshes = data.get('meshes', [])
scenes = data.get('scenes', [])

print(f"Total nodes: {len(nodes)}")
print(f"Total meshes: {len(meshes)}")

# Build parent map
parent_map = {}
for i, n in enumerate(nodes):
    for c in n.get('children', []):
        parent_map[c] = i

# Find root nodes
roots = [i for i in range(len(nodes)) if i not in parent_map]
print(f"Root nodes: {roots}")

def dump_tree(node_idx, indent=0):
    node = nodes[node_idx]
    name = node.get('name', f'node_{node_idx}')
    mesh_idx = node.get('mesh')
    mesh_info = ""
    if mesh_idx is not None:
        m = meshes[mesh_idx]
        m_name = m.get('name', f'mesh_{mesh_idx}')
        prims = len(m.get('primitives', []))
        mesh_info = f" [Mesh {mesh_idx}: '{m_name}' ({prims} prims)]"
    
    t = node.get('translation')
    r = node.get('rotation')
    s = node.get('scale')
    xform_info = []
    if t: xform_info.append(f"T={t}")
    if r: xform_info.append(f"R={r}")
    if s: xform_info.append(f"S={s}")
    xform_str = (" (" + ", ".join(xform_info) + ")") if xform_info else ""
    
    prefix = "  " * indent
    print(f"{prefix}- [{node_idx}] '{name}'{mesh_info}{xform_str}")
    for c in node.get('children', []):
        dump_tree(c, indent + 1)

for r in roots:
    dump_tree(r)
