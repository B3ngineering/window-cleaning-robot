import tkinter as tk
from map_module import Grid


def start_visualization(width: int = 1500, height: int = 1500, title: str = "Window Cleaning Sim", grid: Grid = None):
    """Create the base Tkinter UI and return a small handle to the widgets.
    Returns a dict with keys: root, canvas, status_var.
    """
    root = tk.Tk()
    root.title(title)

    # Main drawing area
    canvas = tk.Canvas(root, width=width, height=height, bg="white", highlightthickness=0)
    canvas.pack(fill="both", expand=True, padx=8, pady=8)

    if grid:
        tile_size = min(width / grid.width, height / grid.height)
        
        for x in range(grid.width):
            for y in range(grid.height):
                tile_value = grid.grid[y][x]
                x0 = x * tile_size
                y0 = y * tile_size
                x1 = (x + 1) * tile_size
                y1 = (y + 1) * tile_size
                
                if tile_value == 0:  # clean
                    fill = "white"
                elif tile_value == 1:  # dirty
                    fill = "yellow"
                elif tile_value == 2:  # wall
                    fill = "brown"
                else:
                    fill = "black"
                
                canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="gray")

    # Status line
    status_var = tk.StringVar(value="Ready")
    status = tk.Label(root, textvariable=status_var, anchor="w")
    status.pack(fill="x", padx=8, pady=(0, 8))

    return {"root": root, "canvas": canvas, "status_var": status_var}


def main():
    grid = Grid(width=10, height=10)
    app = start_visualization(grid=grid)
    app["root"].mainloop()


if __name__ == "__main__":
    main()