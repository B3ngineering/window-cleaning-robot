from sample.map_module import Grid
from sample.robot_module import Robot
from sample.gui import start_visualization

def main():
    grid = Grid(width=20, height=15)
    robot = Robot(water=100, power=100, home=(0, 0))
    app = start_visualization(grid=grid, robot=robot)
    app["root"].mainloop()


if __name__ == "__main__":
    main()