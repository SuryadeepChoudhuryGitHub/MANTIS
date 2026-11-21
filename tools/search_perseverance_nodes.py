import json
import struct

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<4sII', f.read(12))
    chunk_len, chunk_type = struct.unpack('<I4s', f.read(8))
    json_bytes = f.read(chunk_len)
    data = json.loads(json_bytes.decode('utf-8'))
    # read bin chunk
    bin_len, bin_type = struct.unpack('<I4s', f.read(8))
    bin_data = f.read(bin_len)

nodes = data.get('nodes', [])
meshes = data.get('meshes', [])

# Check node 40 (Wheels_objs) and node 41 (suspension)
for idx in [40, 41]:
    n = nodes[idx]
    m_idx = n.get('mesh')
    m = meshes[m_idx]
    print(f"Node {idx}: '{n.get('name')}', mesh {m_idx}: '{m.get('name')}'")
    t = n.get('translation', [0,0,0])
    r = n.get('rotation', [0,0,0,1])
    s = n.get('scale', [1,1,1])
    print(f"  T={t}, R={r}, S={s}")

# Let's inspect all nodes that have 'suspension', 'wheel', 'rocker', 'bogie', 'arm', 'leg', 'differential' in their names!
print("\n=== SEARCHING FOR MECHANICAL / SUSPENSION / WHEEL NODES ===")
for i, n in enumerate(nodes):
    name = n.get('name', '').lower()
    for kw in ['wheel', 'susp', 'rocker', 'bogie', 'diff', 'arm', 'leg', 'axle', 'joint', 'pivot', 'strut', 'link']:
        if kw in name:
            m_idx = n.get('mesh')
            m_name = meshes[m_idx].get('name') if m_idx is not None else 'NO_MESH'
            print(f"Node {i:2d}: '{n.get('name')}' (mesh {m_idx}: '{m_name}') | T={n.get('translation')} | R={n.get('rotation')} | S={n.get('scale')}")
            break
