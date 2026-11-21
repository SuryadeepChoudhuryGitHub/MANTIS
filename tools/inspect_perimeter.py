import numpy as np

data = np.fromfile('model/terrain/mars_jezero_alpha.bin', dtype=np.float32).reshape((1024, 1024))
top_edge = data[0, :]
bottom_edge = data[1023, :]
left_edge = data[:, 0]
right_edge = data[:, 1023]
perimeter = np.concatenate([top_edge, bottom_edge, left_edge, right_edge])
print(f"Perimeter min={perimeter.min():.2f}, max={perimeter.max():.2f}, mean={perimeter.mean():.2f}, std={perimeter.std():.2f}")
