import math


class LiDARSensorModel:
    def __init__(self, prob_hit=1.0, prob_miss=0.0):
        self.prob_hit = prob_hit
        self.prob_miss = prob_miss

    def bresenham_line(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = max(dx, dy)

        if steps == 0:
            return [(x0, y0)]

        for i in range(steps + 1):
            x = round(x0 + i * (x1 - x0) / steps)
            y = round(y0 + i * (y1 - y0) / steps)
            cells.append((x, y))

        return cells

    def process_lidar_ray(self, grid, x, y, yaw, angle, distance, max_range):
        a = yaw + angle
        if distance < max_range and not math.isinf(distance) and not math.isnan(distance):
            end_x = x + distance * math.cos(a)
            end_y = y + distance * math.sin(a)
            hit = True
        else:
            end_x = x + max_range * math.cos(a)
            end_y = y + max_range * math.sin(a)
            hit = False

        gx0, gy0 = grid.world_to_grid(x, y)
        gx1, gy1 = grid.world_to_grid(end_x, end_y)
        ray_cells = self.bresenham_line(gx0, gy0, gx1, gy1)

        for i, (gx, gy) in enumerate(ray_cells):
            if not grid.is_valid_cell(gx, gy):
                continue
            p = self.prob_hit if hit and i == len(ray_cells) - 1 else self.prob_miss
            grid.update_cell_by_index(gx, gy, p)
