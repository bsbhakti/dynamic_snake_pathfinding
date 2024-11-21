def generate_snake_game_instance(rows, cols, num_agents, agents_positions):
    """
    Generate a snake game instance with obstacles only at the edges.

    Args:
    - rows (int): Number of rows in the grid.
    - cols (int): Number of columns in the grid.
    - num_agents (int): Number of agents.
    - agents_positions (list): List of tuples containing (start_row, start_col, goal_row, goal_col).

    Returns:
    - str: The formatted snake game instance as a string.
    """
    # Create the grid
    grid = []
    for r in range(rows):
        if r == 0 or r == rows - 1:  # Top and bottom edges
            grid.append(["@"] * cols)
        else:  # Middle rows with obstacles on the left and right edges
            grid.append(["@"] + ["."] * (cols - 2) + ["@"])

    # Convert the grid to a string
    grid_str = "\n".join([" ".join(row) for row in grid])

    # Add the number of agents
    agents_str = f"{num_agents}"

    # Add the agent positions
    agent_positions_str = "\n".join([
        f"{start_row} {start_col} {goal_row} {goal_col}"
        for start_row, start_col, goal_row, goal_col in agents_positions
    ])

    # Combine everything into the instance format
    instance = f"{rows} {cols}\n{grid_str}\n{agents_str}\n{agent_positions_str}"
    return instance


# Parameters
rows = 40  # Grid height
cols = 60  # Grid width
num_agents = 1 # Number of agents
agents_positions = [
    (2, 2, 35, 55),  # Agent 1: Start (2,2), Goal (35,55)
]

# Generate the instance
snake_game_instance = generate_snake_game_instance(rows, cols, num_agents, agents_positions)

# Print the instance
f = open("exp1.txt", "a")
f.write(snake_game_instance)
f.close()


