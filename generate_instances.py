import random
import argparse

def generate_dynamic_obstacles(grid_size, num_obstacles):
    """Generate dynamic obstacles with random positions, start times, and durations."""
    obstacles = []
    for _ in range(num_obstacles):
        x = random.randint(0, grid_size - 1)
        y = random.randint(0, grid_size - 1)
        start_time = random.randint(0, 5)  # Shorter start time range
        duration = random.randint(1, 3)  # Shorter durations
        obstacles.append({"position": (x, y), "start_time": start_time, "duration": duration})
    return obstacles

def generate_instance(grid_size, num_obstacles):
    """Generate a single instance."""
    # Create a grid with reduced static obstacle density
    grid = [[0 if random.random() > 0.1 else 1 for _ in range(grid_size)] for _ in range(grid_size)]
    
    # Ensure start and goal positions are not blocked
    start1, start2 = (0, 0), (grid_size - 1, grid_size - 1)
    goal1, goal2 = (grid_size - 1, grid_size - 1), (0, 0)
    grid[start1[0]][start1[1]] = 0
    grid[goal1[0]][goal1[1]] = 0
    grid[start2[0]][start2[1]] = 0
    grid[goal2[0]][goal2[1]] = 0

    # Generate fewer dynamic obstacles
    dynamic_obstacles = generate_dynamic_obstacles(grid_size, max(1, num_obstacles // 2))  # Reduce obstacle count

    return {
        "agents": [start1, start2],
        "goals": [goal1, goal2],
        "dynamic_obstacles": dynamic_obstacles,
        "grid": grid
    }

def generate_instances(num_instances):
    """Generate a list of instances with simplified configurations."""
    instances = []
    for _ in range(num_instances):
        # Use smaller grid sizes and fewer obstacles
        grid_size = random.choice([5, 6, 7])  # Keep grid sizes small
        num_obstacles = random.randint(1, grid_size // 2)  # Reduce obstacle density
        instance = generate_instance(grid_size, num_obstacles)
        instances.append(instance)
    return instances

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Generate simplified multi-agent pathfinding scenarios.")
    parser.add_argument(
        "--num_instances",
        type=int,
        default=10,
        help="Number of instances to generate (default: 10)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="instances.py",
        help="Output file name (default: instances.py)"
    )
    args = parser.parse_args()

    # Generate instances
    instances = generate_instances(args.num_instances)

    # Save to the specified file
    with open(args.output, "w") as f:
        f.write("scenarios = [\n")
        for instance in instances:
            f.write(f"    {instance},\n")
        f.write("]\n")

    print(f"Generated {args.num_instances} simplified instances and saved to {args.output}")

if __name__ == "__main__":
    main()
