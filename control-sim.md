# Planning for Control Loop testing

## Tasks

### Grid

- We need a rectangular grid of some kind
- Each tile will be the dimensions of the robot, lets say 1x1 for now
- Each tile on the grid has a state: clean, dirty, wall

### Robot

- The robot is has a position on the grid, a water level, and a power level
- It also has a dedicated home position

### Planning

- On startup, the robot should compute a path from its position to the nearest window, and the grid should be in its memory
- The plan will give the robot all movements it needs to navigate to each dirty window and turn it to clean

### Traversal

- While the robot is traversing, it expends 1 power per tile moved
- When it reaches a dirty window, it loses one water to turn it to clean
- Once all windows are clean, it returns to home
- If power or water is below a certain threshold (10% of initial), the robot returns to home as quickly as possible
