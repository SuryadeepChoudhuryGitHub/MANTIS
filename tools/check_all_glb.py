import struct
import json

def check_glb(path):
    with open(path, 'rb') as f:
        magic, version, length = struct.unpack('<III', f.read(12))
        chunk_len, chunk_type = struct.unpack('<II', f.read(8))
        json_bytes = f.read(chunk_len)
        gltf = json.loads(json_bytes.decode('utf-8'))
    print(f"=== {path} ===")
    print("extensionsUsed:", gltf.get('extensionsUsed'))
    print("extensionsRequired:", gltf.get('extensionsRequired'))
    print("node count:", len(gltf.get('nodes', [])))
    print("mesh count:", len(gltf.get('meshes', [])))

check_glb('model/PerseveranceRover.glb')
check_glb('model/24584_Curiosity_static.glb')
check_glb('model/24883_MER_static.glb')
