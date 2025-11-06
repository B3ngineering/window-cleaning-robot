from map_module import Grid

class Robot:
    def __init__(self, water: int = 100, power: int = 100, home: tuple[int, int] = (0, 0)):
        self.x = home[0]
        self.y = home[1]
        self.water = water
        self.power = power
        self.home = home

    def clean_tile(self, grid: Grid):
        if grid.get_tile(self.x, self.y) == 1:
            grid.set_tile(self.x, self.y, 0)
            self.water -= 1
            self.power -= 1
        
    def move(self, grid: Grid, direction: str):
        if direction == "up":
            self.y -= 1
        elif direction == "down":
            self.y += 1
        elif direction == "left":
            self.x -= 1
        elif direction == "right":
            self.x += 1
        self.power -= 1