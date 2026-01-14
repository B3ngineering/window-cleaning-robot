import random

class Grid:
    def __init__(self, width: int = 20, height: int = 15):
        self.width = width
        self.height = height
        self.grid = self._generate_building()

    def _generate_building(self):
        """Generate a building facade with walls and windows (clean/dirty)."""
        # Start with all walls
        grid = [[2 for _ in range(self.width)] for _ in range(self.height)]

        # Create windows in a regular pattern
        # Windows are 2x2, spaced 3 apart horizontally, 2 apart vertically
        window_width = 2
        window_height = 2
        h_spacing = 3  # horizontal gap between windows
        v_spacing = 2  # vertical gap between windows

        # Start windows a bit from the edges
        start_x = 2
        start_y = 2

        x = start_x
        while x + window_width <= self.width - 1:
            y = start_y
            while y + window_height <= self.height - 1:
                # Create a window (randomly clean=0 or dirty=1)
                for wy in range(window_height):
                    for wx in range(window_width):
                        # Each window pane is randomly clean or dirty
                        grid[y + wy][x + wx] = random.choice([0, 1])
                y += window_height + v_spacing
            x += window_width + h_spacing

        return grid

    def get_tile(self, x: int, y: int):
        return self.grid[y][x]

    def set_tile(self, x: int, y: int, value: int):
        self.grid[y][x] = value