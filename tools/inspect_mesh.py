import struct
import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))
    bin_bytes = f.read() # remainder is BIN chunk

meshes = gltf.get('meshes', [])
accessors = gltf.get('accessors', [])
buffer_views = gltf.get('bufferViews', [])

def get_accessor_data(acc_idx):
    acc = accessors[acc_idx]
    bv = buffer_views[acc['bufferView']]
    offset = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    # assume buffer 0
    count = acc['count']
    comp_type = acc['componentType'] # 5126 = float
    type_str = acc['type'] # VEC3, etc.
    return acc, offset, count

print(f"Mesh 22 ('{meshes[22]['name']}'): {len(meshes[22]['primitives'])} primitives")
for p_idx, p in enumerate(meshes[22]['primitives']):
    pos_acc_idx = p['attributes']['POSITION']
    acc = accessors[pos_acc_idx]
    print(f"  Prim {p_idx}: pos min={acc.get('min')}, max={acc.get('max')}, count={acc['count']}, mat={p.get('material')}")

print(f"Mesh 23 ('{meshes[23]['name']}'): {len(meshes[23]['primitives'])} primitives")
for p_idx, p in enumerate(meshes[23]['primitives']):
    pos_acc_idx = p['attributes']['POSITION']
    acc = accessors[pos_acc_idx]
    print(f"  Prim {p_idx}: pos min={acc.get('min')}, max={acc.get('max')}, count={acc['count']}, mat={p.get('material')}")

print(f"Mesh 14 ('{meshes[14]['name']}'): {len(meshes[14]['primitives'])} primitives")
for p_idx, p in enumerate(meshes[14]['primitives']):
    pos_acc_idx = p['attributes']['POSITION']
    acc = accessors[pos_acc_idx]
    print(f"  Prim {p_idx}: pos min={acc.get('min')}, max={acc.get('max')}, count={acc['count']}, mat={p.get('material')}")
