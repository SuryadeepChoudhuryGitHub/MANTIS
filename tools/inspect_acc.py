import struct
import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))

meshes = gltf.get('meshes', [])
accessors = gltf.get('accessors', [])
buffer_views = gltf.get('bufferViews', [])

p0 = meshes[22]['primitives'][0]
pos_idx = p0['attributes']['POSITION']
print('Accessor for POSITION:', accessors[pos_idx])
