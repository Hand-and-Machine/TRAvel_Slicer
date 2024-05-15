import Rhino
import rhinoscriptsyntax as rs

# Grid Class
class Grid:
    def __init__(self, points, width):
        bbox = rs.BoundingBox(points)
        self.minX = bbox[0].X - width
        self.minY = bbox[0].Y - width
        self.width = float(width)

        self.max_x_idx = int((bbox[2].X - self.minX) // self.width) + 1
        self.max_y_idx = int((bbox[2].Y - self.minY) // self.width) + 1

        self.grid = [[[] for _ in range(self.max_y_idx+1)] for _ in range(self.max_x_idx+1)]

        for point in points:
            x_idx = int((point.X - self.minX) // self.width)
            y_idx = int((point.Y - self.minY) // self.width)

            self.grid[x_idx][y_idx].append(point)

    def get_neighbors(self, point, neighborhood=1):
        x_idx = max(min(int((point.X - self.minX) // self.width), self.max_x_idx), 0)
        y_idx = max(min(int((point.Y - self.minY) // self.width), self.max_y_idx), 0)

        # start with center
        points = self.grid[x_idx][y_idx]

        # retrieve neighbors
        for x in range(max(0, x_idx-neighborhood), min(self.max_x_idx, x_idx+neighborhood+1)):
            for y in range(max(0, y_idx-neighborhood), min(self.max_y_idx, y_idx+neighborhood)+1):
                if not (x==x_idx and y==y_idx):
                    points = points + self.grid[x][y]
        
        return points
