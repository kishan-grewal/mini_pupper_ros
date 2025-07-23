# test_sensor.py
from occupancy_grid import OccupancyGrid
from sensor_model import LiDARSensorModel
import math
import matplotlib.pyplot as plt

grid = OccupancyGrid(width_meters=1.0, height_meters=1.0, resolution=0.001)
sensor = LiDARSensorModel()


def test_ray(x, y, yaw, angle, dist):
    sensor.process_lidar_ray(grid, x, y, yaw, angle, dist, max_range=10.0)


n = 1000
d = 0.3
for i in range(n+1):
    angle = (2 * math.pi / n) * i
    test_ray(0.0, 0.0, 0.0, angle, d * i / n)


# Plot the heatmap
plt.imshow(grid.grid, origin='lower', cmap='gray', vmin=0.0, vmax=1.0)
plt.title("Occupancy Grid")
plt.xlabel("Grid X")
plt.ylabel("Grid Y")
plt.colorbar(label='Occupancy Probability')
plt.grid(False)
plt.show()
