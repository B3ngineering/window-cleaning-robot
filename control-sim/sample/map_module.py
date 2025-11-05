import random

class Grid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

        # Initialize each cell randomly to 0, 1, or 2
        self.grid = [[random.randint(0, 2) for _ in range(width)] for _ in range(height)]

    def get_tile(self, x: int, y: int):
        return self.grid[y][x]

    def set_tile(self, x: int, y: int, value: int):
        self.grid[y][x] = value