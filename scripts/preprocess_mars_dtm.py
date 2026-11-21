#!/usr/bin/env python3
"""
PROJECT ARES — Mars Digital Terrain Model (DTM) Preprocessor.

Extracts high-resolution exploration tiles from large HiRISE / orbital GeoTIFFs,
preserves complete geospatial and georeferencing metadata, computes normal maps,
and exports compact binary elevation grids for instantaneous C++ Bullet and OpenGL loading.
"""

import os
import sys
import json
import struct
import numpy as np
from PIL import Image

def preprocess_dtm(
    input_path: str = "model/mars_terrain2.tif",
    output_dir: str = "model/terrain",
    tile_name: str = "mars_jezero_alpha",
    tile_size: int = 1024,
    center_col: int = None,
    center_row: int = None
):
    print("=" * 70)
    print("PROJECT ARES — GEOSPATIAL DTM PREPROCESSOR")
    print("=" * 70)
    
    if not os.path.exists(input_path):
        print(f"[Error] Input GeoTIFF not found: {input_path}")
        sys.exit(1)

    file_size = os.path.getsize(input_path)
    print(f"Input File: {input_path} ({file_size / (1024*1024):.2f} MB)")

    # 1. Parse TIFF header & GeoTIFF tags using Pillow
    Image.MAX_IMAGE_PIXELS = None
    with Image.open(input_path) as img:
        width, height = img.size
        mode = img.mode
        tags = getattr(img, "tag_v2", {})
        
        # Georeferencing tags:
        # Tag 33550: ModelPixelScaleTag (scale_x, scale_y, scale_z)
        pixel_scale = tags.get(33550, (1.0, 1.0, 0.0))
        # Tag 33922: ModelTiepointTag (i, j, k, x, y, z)
        tiepoint = tags.get(33922, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        # Tag 42113: GDAL_NODATA
        nodata_val = float(tags.get(42113, -3.40282e+38))
        # Tag 273: StripOffsets
        strip_offsets = tags.get(273, (172609,))
        header_offset = strip_offsets[0]

    print(f"Raster Size: {width} cols x {height} rows ({width * height:,} pixels)")
    print(f"Data Format: 32-bit IEEE float ({mode})")
    print(f"Horizontal Resolution: {pixel_scale[0]:.2f} m/pixel (X), {pixel_scale[1]:.2f} m/pixel (Y)")
    print(f"Tie Point: Pixel (0,0) -> Projected ({tiepoint[3]:.1f} m East, {tiepoint[4]:.1f} m North)")
    print(f"CRS: Equirectangular Mars 2000 Sphere IAU (Radius = 3,396,190.0 m)")
    print(f"NoData Value: {nodata_val}")

    # 2. Determine Tile Window
    if center_col is None:
        center_col = width // 2
    if center_row is None:
        center_row = height // 2

    half = tile_size // 2
    col_start = max(0, min(width - tile_size, center_col - half))
    row_start = max(0, min(height - tile_size, center_row - half))
    col_end = col_start + tile_size
    row_end = row_start + tile_size

    print(f"\nExtracting Tile Window: cols [{col_start}:{col_end}], rows [{row_start}:{row_end}] ({tile_size}x{tile_size})")

    # 3. Memory-map raw float32 elevation strip
    mm = np.memmap(input_path, dtype="<f4", mode="r", offset=header_offset, shape=(height, width))
    raw_tile = np.array(mm[row_start:row_end, col_start:col_end], copy=True)
    del mm

    # 4. Check NoData & compute elevation statistics
    valid_mask = raw_tile > -100000.0
    valid_count = np.count_nonzero(valid_mask)
    total_pixels = raw_tile.size
    nodata_count = total_pixels - valid_count

    print(f"Total Pixels: {total_pixels:,} | Valid: {valid_count:,} | NoData: {nodata_count}")

    if nodata_count > 0:
        print(f"[Warning] {nodata_count} NoData pixels detected in selected sector! Filling with mean elevation.")
        mean_valid = float(np.mean(raw_tile[valid_mask]))
        raw_tile[~valid_mask] = mean_valid

    mola_min = float(np.min(raw_tile))
    mola_max = float(np.max(raw_tile))
    mola_mean = float(np.mean(raw_tile))
    mola_span = mola_max - mola_min

    print(f"MOLA Elevation Range: {mola_min:.2f} m to {mola_max:.2f} m (Relief: {mola_span:.2f} m, Mean: {mola_mean:.2f} m)")

    # 5. Local simulation normalization: center elevation around 0.0
    h_ref = mola_mean
    local_elev = raw_tile - h_ref
    local_min = float(np.min(local_elev))
    local_max = float(np.max(local_elev))

    # Center spawn elevation at tile center
    center_local_y = float(local_elev[half, half])
    print(f"Local Relative Elevation: {local_min:.2f} m to {local_max:.2f} m (Ref Datum = {h_ref:.2f} m)")
    print(f"Center Surface Elevation (Spawn Y): {center_local_y:.2f} m")

    # 6. Generate High-Quality Surface Normal Map using Central Differences
    print("Computing surface normals via central differences...")
    dx = np.zeros_like(raw_tile)
    dz = np.zeros_like(raw_tile)

    scale_x = pixel_scale[0]
    scale_z = pixel_scale[1]

    # Central differences for interior pixels
    dx[:, 1:-1] = (raw_tile[:, 2:] - raw_tile[:, :-2]) / (2.0 * scale_x)
    # Forward/backward differences for edges
    dx[:, 0] = (raw_tile[:, 1] - raw_tile[:, 0]) / scale_x
    dx[:, -1] = (raw_tile[:, -1] - raw_tile[:, -2]) / scale_x

    dz[1:-1, :] = (raw_tile[2:, :] - raw_tile[:-2, :]) / (2.0 * scale_z)
    dz[0, :] = (raw_tile[1, :] - raw_tile[0, :]) / scale_z
    dz[-1, :] = (raw_tile[-1, :] - raw_tile[-2, :]) / scale_z

    # Normal vector in OpenGL tangent space (+Y Up, +X Right, +Z Toward Viewer / Row Direction)
    # Surface gradient: tangent u = (1, 0, dx), bitangent v = (0, 1, dz) -> normal = (-dx, -dz, 1) or (-dx, 1, -dz)
    # In standard tangent space normal map: (R, G, B) = (X_tangent, Y_tangent, Z_tangent)
    norm_x = -dx
    norm_y = -dz
    norm_z = np.ones_like(raw_tile)
    length = np.sqrt(norm_x**2 + norm_y**2 + norm_z**2)
    norm_x /= length
    norm_y /= length
    norm_z /= length

    # Map to [0, 255] RGB
    norm_r = ((norm_x + 1.0) * 0.5 * 255.0).astype(np.uint8)
    norm_g = ((norm_y + 1.0) * 0.5 * 255.0).astype(np.uint8)
    norm_b = ((norm_z + 1.0) * 0.5 * 255.0).astype(np.uint8)
    normal_rgb = np.stack([norm_r, norm_g, norm_b], axis=-1)

    # 7. Write Output Files
    os.makedirs(output_dir, exist_ok=True)
    
    bin_path = os.path.join(output_dir, f"{tile_name}.bin")
    json_path = os.path.join(output_dir, f"{tile_name}.json")
    norm_path = os.path.join(output_dir, f"{tile_name}_normal.png")

    # Export binary height array (raw float32, row-major)
    # Note: local_elev is float32
    local_elev.astype("<f4").tofile(bin_path)
    print(f"Exported binary height array: {bin_path} ({os.path.getsize(bin_path):,} bytes)")

    # Export normal map PNG
    norm_img = Image.fromarray(normal_rgb, mode="RGB")
    norm_img.save(norm_path, format="PNG")
    print(f"Exported surface normal map: {norm_path} ({os.path.getsize(norm_path):,} bytes)")

    # Compute tile geographic center
    tile_origin_x = tiepoint[3] + col_start * pixel_scale[0]
    tile_origin_y = tiepoint[4] - row_start * pixel_scale[1]
    tile_center_x = tile_origin_x + (tile_size * 0.5) * pixel_scale[0]
    tile_center_y = tile_origin_y - (tile_size * 0.5) * pixel_scale[1]

    # Export metadata JSON
    metadata = {
        "dataset_name": tile_name,
        "source_file": os.path.basename(input_path),
        "georeferencing": {
            "crs": "Equirectangular Mars 2000 Sphere IAU",
            "datum_radius_meters": 3396190.0,
            "projection": "Equirectangular",
            "horizontal_unit": "meters",
            "vertical_unit": "meters",
            "vertical_datum": "MOLA reference aeroid",
            "pixel_scale_meters": [float(pixel_scale[0]), float(pixel_scale[1])],
            "tile_origin_projected_xy": [float(tile_origin_x), float(tile_origin_y)],
            "tile_center_projected_xy": [float(tile_center_x), float(tile_center_y)]
        },
        "grid": {
            "cols": tile_size,
            "rows": tile_size,
            "physical_width_meters": float(tile_size * pixel_scale[0]),
            "physical_depth_meters": float(tile_size * pixel_scale[1]),
            "total_vertices": tile_size * tile_size,
            "total_triangles": (tile_size - 1) * (tile_size - 1) * 2
        },
        "elevation_stats": {
            "mola_elevation_min_meters": mola_min,
            "mola_elevation_max_meters": mola_max,
            "mola_elevation_mean_meters": mola_mean,
            "mola_elevation_relief_meters": mola_span,
            "local_reference_elevation_meters": h_ref,
            "local_elevation_min_meters": local_min,
            "local_elevation_max_meters": local_max,
            "nodata_value": nodata_val
        },
        "simulation": {
            "coordinate_convention": "+X Left, +Y Up, +Z Forward",
            "primary_spawn_position": [0.0, float(center_local_y + 0.65), 0.0],
            "primary_spawn_yaw_deg": 0.0,
            "surface_type": "MarsRegolith",
            "provisional_friction_mu": 0.65,
            "gravity_m_s2": 3.71,
            "air_density_kg_m3": 0.020
        },
        "files": {
            "binary_heights": os.path.basename(bin_path),
            "normal_map": os.path.basename(norm_path)
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)
    print(f"Exported metadata JSON: {json_path}")
    print("\n[SUCCESS] Mars DTM Preprocessing Complete!")
    print("=" * 70)

if __name__ == "__main__":
    preprocess_dtm()
