import json
import struct

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<4sII', f.read(12))
    chunk_len, chunk_type = struct.unpack('<I4s', f.read(8))
    json_bytes = f.read(chunk_len)
    data = json.loads(json_bytes.decode('utf-8'))

meshes = data.get('meshes', [])
nodes = data.get('nodes', [])

print("=== ALL MESHES ===")
for i, m in enumerate(meshes):
    name = m.get('name', '')
    prims = m.get('primitives', [])
    print(f"Mesh {i:2d}: '{name}' ({len(prims)} prims)")

print("\n=== SUSPENSION & WHEELS NODES ===")
for idx in [40, 41]:
    n = nodes[idx]
    m_idx = n.get('mesh')
    m = meshes[m_idx] if m_idx is not None else None
    print(f"Node {idx}: name='{n.get('name')}' mesh_idx={m_idx} mesh_name='{m.get('name') if m else None}'")
    if m:
        for p_idx, p in enumerate(m.get('primitives', [])):
            ext = p.get('extensions', {})
            draco = ext.get('KHR_draco_mesh_compression', {})
            attrs = p.get('attributes', {})
            mat = p.get('material')
            mat_name = data.get('materials', [])[mat].get('name', '') if mat is not None else 'NO_MAT'
            print(f"  Prim {p_idx}: mat={mat} ('{mat_name}') attrs={list(attrs.keys())} draco_attrs={list(draco.get('attributes', {}).keys())}")
