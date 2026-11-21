import struct
import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))

meshes = gltf['meshes']
accessors = gltf['accessors']
print(f"Total accessors: {len(accessors)}")
print("Sample accessors:")
for i in range(min(10, len(accessors))):
    print(f"  [{i}]: {accessors[i]}")
