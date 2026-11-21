import numpy as np

data = np.fromfile('model/terrain/mars_jezero_alpha.bin', dtype=np.float32)
print(f"Total points: {len(data)}, min={data.min():.4f}, max={data.max():.4f}, mean={data.mean():.4f}, std={data.std():.4f}")
# Also let's inspect the distribution or a 10x10 slice
data_2d = data.reshape((1024, 1024))
print("Center (512, 512):", data_2d[512, 512])
print("Corner (0,0):", data_2d[0, 0])
print("Corner (1023,1023):", data_2d[1023, 1023])
print("Elevation quantiles: 1%={:.2f}, 10%={:.2f}, 50%={:.2f}, 90%={:.2f}, 99%={:.2f}".format(
    *np.percentile(data, [1, 10, 50, 90, 99])
))
