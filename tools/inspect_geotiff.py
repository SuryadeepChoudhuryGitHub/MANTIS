import rasterio
import numpy as np

with rasterio.open('model/terrain/mars_terrain2.tif') as src:
    print("=== mars_terrain2.tif Deep Inspection ===")
    print("Dimensions:", src.width, "x", src.height)
    print("Bands:", src.count)
    print("CRS:", src.crs)
    print("CRS to_string:", src.crs.to_string())
    print("CRS wkt snippet:", str(src.crs.wkt)[:300])
    print("Transform:", src.transform)
    print("Bounds:", src.bounds)
    print("NoData value:", src.nodata)
    print("Res:", src.res)
    
    # Read a sample block from center to inspect values
    cx = src.width // 2
    cy = src.height // 2
    window = rasterio.windows.Window(cx - 500, cy - 500, 1000, 1000)
    data = src.read(1, window=window)
    valid = data[data != src.nodata]
    print(f"Sample center 1000x1000 window: total={data.size}, valid={valid.size}")
    if valid.size > 0:
        print(f"Sample valid min={valid.min():.2f}, max={valid.max():.2f}, mean={valid.mean():.2f}, std={valid.std():.2f}")
