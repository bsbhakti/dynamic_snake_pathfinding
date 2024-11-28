from instances import scenarios
from dicbs import dicbs

class GridEnvironment:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def is_valid(self, position):
        x, y = position
        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x][y] == 0

# Assuming dicbs and slpa_search functions are defined here or imported

for i, scenario in enumerate(scenarios):
    print(f"\nScenario {i+1}")
    agents = scenario["agents"]
    goals = scenario["goals"]
    dynamic_obstacles = [
        (item["position"], item["start_time"], item["duration"]) for item in scenario["dynamic_obstacles"]
    ]
    grid = scenario["grid"]
    
    env = GridEnvironment(grid)
    paths = dicbs(agents, goals, env, dynamic_obstacles)

    if paths:
        for idx, path in enumerate(paths):
            print(f"Path for agent {idx}: {[(pos, time) for pos, time in path]}")
    else:
        print("No solution found.")
