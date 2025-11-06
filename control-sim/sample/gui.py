import tkinter as tk
from map_module import Grid
from robot_module import Robot


def start_visualization(width: int = 1500, height: int = 1500, title: str = "Window Cleaning Sim", grid: Grid = None, robot: Robot = None):
    """Create the base Tkinter UI and return a small handle to the widgets.
    Returns a dict with keys: root, canvas, status_var.
    """
    root = tk.Tk()
    root.title(title)

    # Main drawing area
    canvas = tk.Canvas(root, width=width, height=height, bg="white", highlightthickness=0)
    canvas.pack(fill="both", expand=True, padx=8, pady=8)

    def draw():
        canvas.delete("all")
        if not grid:
            return
        tile_size = min(width / grid.width, height / grid.height)

        # If a robot is present and on a dirty tile, clean it
        if robot is not None:
            if grid.get_tile(robot.x, robot.y) == 1:
                robot.clean_tile(grid)
                update_resources()

        # Draw the grid
        for x in range(grid.width):
            for y in range(grid.height):
                tile_value = grid.grid[y][x]
                x0, y0, x1, y1 = x * tile_size, y * tile_size, (x + 1) * tile_size, (y + 1) * tile_size
                color_map = {
                    0: "white",   # clean
                    1: "yellow",  # dirty
                    2: "brown",   # wall
                }
                fill = color_map.get(tile_value, "black")
                canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="gray")

        # Draw robot if provided
        if robot is not None:
            robot_x0 = robot.x * tile_size
            robot_y0 = robot.y * tile_size
            robot_x1 = (robot.x + 1) * tile_size
            robot_y1 = (robot.y + 1) * tile_size
            canvas.create_oval(robot_x0 + 2, robot_y0 + 2, robot_x1 - 2, robot_y1 - 2, fill="blue", outline="darkblue", width=2)

        update_resources()
    # Controls for movement
    if robot is not None and grid is not None:
        controls = tk.Frame(root)
        controls.pack(fill="x", padx=8, pady=(0, 8))

        def can_move(nx: int, ny: int) -> bool:
            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                return False
            # Block walls
            # return grid.get_tile(nx, ny) != 2
            # Don't block walls for now
            return True

        def move_up():
            nx, ny = robot.x, robot.y - 1
            if can_move(nx, ny):
                robot.move(grid, "up")
                draw()

        def move_down():
            nx, ny = robot.x, robot.y + 1
            if can_move(nx, ny):
                robot.move(grid, "down")
                draw()

        def move_left():
            nx, ny = robot.x - 1, robot.y
            if can_move(nx, ny):
                robot.move(grid, "left")
                draw()

        def move_right():
            nx, ny = robot.x + 1, robot.y
            if can_move(nx, ny):
                robot.move(grid, "right")
                draw()

        tk.Button(controls, text="Up", command=move_up).pack(side="left")
        tk.Button(controls, text="Down", command=move_down).pack(side="left", padx=(6, 0))
        tk.Button(controls, text="Left", command=move_left).pack(side="left", padx=(6, 0))
        tk.Button(controls, text="Right", command=move_right).pack(side="left", padx=(6, 0))

    resource_frame = tk.Frame(root)
    resource_frame.pack(fill="x", padx=8, pady=(0, 8))

    water_var = tk.StringVar()
    water_label = tk.Label(resource_frame, textvariable=water_var, anchor="w")
    water_label.pack(side="left", padx=(6, 0))

    power_var = tk.StringVar()
    power_label = tk.Label(resource_frame, textvariable=power_var, anchor="w")
    power_label.pack(side="left", padx=(6, 0))

    def update_resources():
        water_var.set(f"Water: {robot.water}")
        power_var.set(f"Power: {robot.power}")

    # Initial draw
    draw()

    # Status line
    status_var = tk.StringVar(value="Ready")
    status = tk.Label(root, textvariable=status_var, anchor="w")
    status.pack(fill="x", padx=8, pady=(0, 8))

    return {"root": root, "canvas": canvas, "status_var": status_var}


def main():
    grid = Grid(width=10, height=10)
    robot = Robot(water=100, power=100, home=(0, 0))
    app = start_visualization(grid=grid, robot=robot)
    app["root"].mainloop()


if __name__ == "__main__":
    main()