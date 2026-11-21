import struct
import json
import rasterio

def inspect_glb(path):
    print(f"\n=================== {path} ===================")
    with open(path, 'rb') as f:
        magic, version, length = struct.unpack('<III', f.read(12))
        chunk_len, chunk_type = struct.unpack('<II', f.read(8))
        json_bytes = f.read(chunk_len)
        gltf = json.loads(json_bytes.decode('utf-8'))
    print(f"GLB version: {version}, file length: {length / 1024 / 1024:.2f} MB")
    print("extensionsUsed:", gltf.get('extensionsUsed'))
    print("extensionsRequired:", gltf.get('extensionsRequired'))
    nodes = gltf.get('nodes', [])
    meshes = gltf.get('meshes', [])
    materials = gltf.get('materials', [])
    textures = gltf.get('textures', [])
    images = gltf.get('images', [])
    accessors = gltf.get('accessors', [])
    print(f"Nodes: {len(nodes)}, Meshes: {len(meshes)}, Materials: {len(materials)}, Textures: {len(textures)}, Images: {len(images)}")
    for i, m in enumerate(meshes):
        print(f"Mesh {i} '{m.get('name')}': {len(m.get('primitives', []))} primitives")
        for p_idx, p in enumerate(m.get('primitives', [])):
            pos_idx = p['attributes'].get('POSITION')
            if pos_idx is not None:
                acc = accessors[pos_idx]
                print(f"  Prim {p_idx}: count={acc.get('count')}, min={acc.get('min')}, max={acc.get('max')}")

inspect_glb('model/terrain/mars_terrain0.glb')
inspect_glb('model/terrain/mars_terrain1.glb')

print("\n=================== model/terrain/mars_terrain2.tif ===================")
try:
    with rasterio.open('model/terrain/mars_terrain2.tif') as src:
        print(f"Driver: {src.driver}")
        print(f"Width: {src.width}, Height: {src.height}, Count: {src.count}")
        print(f"CRS: {src.crs}")
        print(f"Bounds: {src.bounds}")
        print(f"Transform: {src.transform}")
        print(f"NoData: {src.nodata}")
        print(f"Dtypes: {src.dtypes}")
except Exception as e:
    print(f"Error opening with rasterio: {e}")
