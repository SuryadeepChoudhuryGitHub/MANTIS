import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    f.seek(12)
    chunk_len = int.from_bytes(f.read(4), 'little')
    chunk_type = f.read(4)
    data = json.loads(f.read(chunk_len))

mesh_idx = data['nodes'][41]['mesh']
mesh = data['meshes'][mesh_idx]
for p_idx, p in enumerate(mesh['primitives']):
    acc_idx = p['attributes']['POSITION']
    acc = data['accessors'][acc_idx]
    mat_idx = p.get('material')
    mat_name = data['materials'][mat_idx]['name'] if mat_idx is not None else 'none'
    print(f"Prim {p_idx} ({mat_name}): min={acc.get('min')}, max={acc.get('max')}")
