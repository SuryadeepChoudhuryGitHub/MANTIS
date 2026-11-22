import json

with open('model/PerseveranceRover.glb', 'rb') as f:
    f.seek(12)
    chunk_len = int.from_bytes(f.read(4), 'little')
    f.seek(20)
    gltf = json.loads(f.read(chunk_len).decode('utf-8'))

for i, node in enumerate(gltf.get('nodes', [])):
    name = node.get('name', '<noname>')
    m = node.get('mesh')
    t = node.get('translation')
    ch = node.get('children')
    print(f"{i:2d}: {name:32s} mesh={m} trans={t} children={ch}")
