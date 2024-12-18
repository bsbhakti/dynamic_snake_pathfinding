import json
import time
from instances import scenarios
from snakeProblem import SnakeProblem

class GridEnvironment:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def is_valid(self, position):
        x, y = position
        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x][y] == 0

results = []

# Process each scenario
for i, scenario in enumerate(scenarios):
    print(f"\nProcessing Scenario {i+1}")
    agents = scenario["agents"]
    goals = scenario["goals"]
    my_map = scenarios[i]['grid']
    dynamic_obstacles = [
        (item["position"], item["start_time"], item["duration"]) for item in scenario["dynamic_obstacles"]
    ]
    max_time = max(start_time + duration for _, start_time, duration in dynamic_obstacles)


    grid = scenario["grid"]
    
    env = GridEnvironment(grid)

    for j in range (len(my_map)):
        for k in range (len(my_map[j])):
            if my_map[j][k] == 1:
                dynamic_obstacles.append(((j,k),0,float('inf')))

    # Measure time taken by dicbs
    start_time = time.time()

    problem = SnakeProblem(env,agents, goals, dynamic_obstacles, max_time)
    paths = problem.find_solution()
    # paths = dicbs(agents, goals, env, dynamic_obstacles)

    end_time = time.time()
    elapsed_time = end_time - start_time

    if paths:
        print(f"Solution found in {elapsed_time:.2f} seconds")
        agent_paths = {f"agent{idx+1}": [(pos, time) for pos, time in path] for idx, path in enumerate(paths)}
    else:
        print(f"No solution found in {elapsed_time:.2f} seconds")
        agent_paths = "No solution"

    # Save results
    results.append({
        "scenario_id": i + 1,
        "agents": agents,
        "goals": goals,
        "dynamic_obstacles": scenario["dynamic_obstacles"],
        "grid": grid,
        "paths": agent_paths,
        "time_taken": elapsed_time
    })
    # break

# Write results to JSON (flat format)
output_file = "results.json"
with open(output_file, "w") as f:
    json.dump(results, f)

print(f"\nResults saved to {output_file}")
