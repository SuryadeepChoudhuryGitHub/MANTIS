import struct
import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))

print('extensionsUsed:', gltf.get('extensionsUsed'))
print('extensionsRequired:', gltf.get('extensionsRequired'))
p0 = gltf['meshes'][22]['primitives'][0]
print('primitive 0 extensions:', p0.get('extensions'))
