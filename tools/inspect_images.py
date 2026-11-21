import struct
import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    magic, version, length = struct.unpack('<III', f.read(12))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    json_bytes = f.read(chunk_len)
    gltf = json.loads(json_bytes.decode('utf-8'))

images = gltf.get('images', [])
print(f"Total images: {len(images)}")
for i, img in enumerate(images):
    print(f"  Image {i}: name={img.get('name')}, mimeType={img.get('mimeType')}, bufferView={img.get('bufferView')}, uri={img.get('uri')}")
