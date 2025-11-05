from sample.map_module import Grid
from sample.robot_module import Robot

def main():
    grid = Grid(10, 10)
    robot = Robot(water=100, power=100, home=(0, 0))
    robot.clean_tile(grid)
    print(grid.get_tile(robot.x, robot.y))

if __name__ == "__main__":
    main()