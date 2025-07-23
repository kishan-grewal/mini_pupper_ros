import numpy as np


class OccupancyGrid:
    def __init__(self, width_meters=4.0, height_meters=4.0, resolution=0.1):
        self.resolution = resolution
        self.width_cells = int(width_meters / resolution) + 1
        self.height_cells = int(height_meters / resolution) + 1
        self.grid = np.full((self.height_cells, self.width_cells), 0.5, dtype=np.float32)
        self.origin_x = width_meters / 2.0
        self.origin_y = height_meters / 2.0

    def world_to_grid(self, x, y):
        gx = int((self.origin_x + x) / self.resolution)
        gy = int((self.origin_y + y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.resolution - self.origin_x
        y = gy * self.resolution - self.origin_y
        return x, y

    def is_valid_cell(self, gx, gy):
        return 0 <= gx < self.width_cells and 0 <= gy < self.height_cells

    def update_cell(self, x, y, probability):
        gx, gy = self.world_to_grid(x, y)
        self.update_cell_by_index(gx, gy, probability)

    def update_cell_by_index(self, gx, gy, probability):
        if self.is_valid_cell(gx, gy):
            self.grid[gy, gx] = probability

    def get_cell_probability(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        return self.get_cell_probability_by_index(gx, gy)

    def get_cell_probability_by_index(self, gx, gy):
        if self.is_valid_cell(gx, gy):
            return self.grid[gy, gx]
        return 0.50
