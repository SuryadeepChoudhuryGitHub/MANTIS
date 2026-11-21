import struct
import json
import numpy as np

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))
    bin_len, bin_type = struct.unpack('<II', f.read(8))
    bin_data = f.read(bin_len)

meshes = gltf.get('meshes', [])
accessors = gltf.get('accessors', [])
buffer_views = gltf.get('bufferViews', [])

def read_vec3_accessor(acc_idx):
    acc = accessors[acc_idx]
    if 'bufferView' not in acc:
        return np.zeros((acc['count'], 3), dtype=np.float32)
    bv = buffer_views[acc['bufferView']]
    offset = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    stride = bv.get('byteStride', 12)
    count = acc['count']
    verts = []
    for i in range(count):
        pos = offset + i * stride
        verts.append(struct.unpack_from('<fff', bin_data, pos))
    return np.array(verts, dtype=np.float32)

mesh22 = meshes[22]
all_verts = []
for p in mesh22['primitives']:
    verts = read_vec3_accessor(p['attributes']['POSITION'])
    all_verts.append(verts)
all_verts = np.vstack(all_verts)

print(f"Total vertices in Mesh 22: {len(all_verts)}")
print(f"BBox: X=[{all_verts[:,0].min():.4f}, {all_verts[:,0].max():.4f}], Y=[{all_verts[:,1].min():.4f}, {all_verts[:,1].max():.4f}], Z=[{all_verts[:,2].min():.4f}, {all_verts[:,2].max():.4f}]")

left_verts = all_verts[all_verts[:,0] < -0.5]
right_verts = all_verts[all_verts[:,0] > 0.5]

print(f"Left verts: {len(left_verts)}, Right verts: {len(right_verts)}")

for side_name, side_v in [('Left (X < -0.5)', left_verts), ('Right (X > 0.5)', right_verts)]:
    print(f"\n--- {side_name} ---")
    front = side_v[side_v[:,2] > 0.5]
    mid = side_v[(side_v[:,2] >= -0.5) & (side_v[:,2] <= 0.5)]
    rear = side_v[side_v[:,2] < -0.5]
    for seg_name, seg_v in [('Front', front), ('Mid', mid), ('Rear', rear)]:
        if len(seg_v) > 0:
            c = seg_v.mean(axis=0)
            mins = seg_v.min(axis=0)
            maxs = seg_v.max(axis=0)
            radii_y = (maxs[1] - mins[1]) / 2.0
            radii_z = (maxs[2] - mins[2]) / 2.0
            width_x = maxs[0] - mins[0]
            print(f"  {seg_name}: count={len(seg_v)}, center=({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}), Y-span={maxs[1]-mins[1]:.4f} (R~{radii_y:.4f}), Z-span={maxs[2]-mins[2]:.4f} (R~{radii_z:.4f}), width_X={width_x:.4f}")
