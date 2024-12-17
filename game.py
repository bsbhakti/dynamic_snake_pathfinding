import pygame
import time
import random
from dicbs import dicbs  # Import dicbs function
from run_exp import import_mapf_instance
from instances import scenarios
from pdb import set_trace as bp

# Import map and scenarios
# my_map, _, _ = import_mapf_instance("exp1.txt")

# Get scenario for two agents
i = 48
starts_1 = scenarios[i]['agents'][0]
goals_1 = scenarios[i]['goals'][0]
starts_2 = scenarios[i]['agents'][1]
goals_2 = scenarios[i]['goals'][1]
my_map = scenarios[i]['grid']
dynamic_obstacles = [(scenarios[i]['dynamic_obstacles'][0]['position'], scenarios[i]['dynamic_obstacles'][0]['start_time'], scenarios[i]['dynamic_obstacles'][0]['duration'])]

# Print agent start and goal positions
print("Start Agent 1: ", starts_1)
print("Goal Agent 1: ", goals_1)
print("Start Agent 2: ", starts_2)
print("Goal Agent 2: ", goals_2)

# GridEnvironment class for dicbs
class GridEnvironment:
    def __init__(self, my_map):
        self.my_map = my_map
        self.rows = len(my_map)
        self.cols = len(my_map[0])

    def is_valid(self, position):
        x, y = position
        return 0 <= x < self.rows and 0 <= y < self.cols and self.my_map[x][y] == 0

# Snake parameters
snake_speed = 2
block_size = 80

# Window size
window_x = len(my_map[0]) * block_size  # Number of columns
window_y = len(my_map) * block_size     # Number of rows

# Define colors
black = pygame.Color(0, 0, 0)
white = pygame.Color(255, 255, 255)
red = pygame.Color(255, 0, 0)
green_head = pygame.Color(0, 255, 0)
green_body = pygame.Color(0, 180, 0)
blue_head = pygame.Color(0, 0, 255)
blue_body = pygame.Color(0, 0, 180)

# Load images
bomb_image = pygame.image.load("Images/bomb.png")
bomb_image = pygame.transform.scale(bomb_image, (block_size, block_size))

banana_image = pygame.image.load("Images/banana.png")
banana_image = pygame.transform.scale(banana_image, (block_size, block_size))

pineapple_image = pygame.image.load("Images/pineapple.png")
pineapple_image = pygame.transform.scale(pineapple_image, (block_size, block_size))

# Initialize pygame
pygame.init()
pygame.display.set_caption('Two Snakes Game')
game_window = pygame.display.set_mode((window_x, window_y))
fps = pygame.time.Clock()

# Snake positions and paths
snake_body_1 = []  # Snake body for agent 1
snake_body_2 = []  # Snake body for agent 2

# Fruit positions
fruit_position_1 = [goals_1[0] * block_size, goals_1[1] * block_size]
prev_fruit_position_1 = [starts_1[0] * block_size, starts_1[1] * block_size]

fruit_position_2 = [goals_2[0] * block_size, goals_2[1] * block_size]
prev_fruit_position_2 = [starts_2[0] * block_size, starts_2[1] * block_size]

# Game over function
def game_over():
    print("Game Over!")
    pygame.quit()
    quit()

# Initialize GridEnvironment
env = GridEnvironment(my_map)

# Call dicbs for paths
agents_1 = [(prev_fruit_position_1[1] // block_size, prev_fruit_position_1[0] // block_size)]
goals_1 = [(fruit_position_1[1] // block_size, fruit_position_1[0] // block_size)]

agents_2 = [(prev_fruit_position_2[1] // block_size, prev_fruit_position_2[0] // block_size)]
goals_2 = [(fruit_position_2[1] // block_size, fruit_position_2[0] // block_size)]

paths_1, paths_2 = dicbs([agents_1[0], agents_2[0]], [goals_1[0], goals_2[0]], env, dynamic_obstacles)

if not paths_1 or not paths_1[0]:
    print("No path found for Agent 1!")
    game_over()

if not paths_2 or not paths_2[0]:
    print("No path found for Agent 2!")
    game_over()

snake_path_1 = [(pos[1] * block_size, pos[0] * block_size) for pos, _ in paths_1]
snake_path_2 = [(pos[1] * block_size, pos[0] * block_size) for pos, _ in paths_2]

# Main Game Loop
snake_index_1 = 0
snake_index_2 = 0
time_step = 0  # Add a time step counter

while True:
    # End the game if both snakes finish their paths
    if snake_index_1 >= len(snake_path_1) and snake_index_2 >= len(snake_path_2):
        print("Both agents have reached their goals!")
        game_over()

    # Update Snake 1
    if snake_index_1 < len(snake_path_1):
        snake_position_1 = snake_path_1[snake_index_1]
        snake_body_1.insert(0, list(snake_position_1))
        if len(snake_body_1) > 1:
            snake_body_1.pop()
        snake_index_1 += 1

    # Update Snake 2
    if snake_index_2 < len(snake_path_2):
        snake_position_2 = snake_path_2[snake_index_2]
        snake_body_2.insert(0, list(snake_position_2))
        if len(snake_body_2) > 1:
            snake_body_2.pop()
        snake_index_2 += 1

    # Check for collisions with dynamic obstacles
    for obs in dynamic_obstacles:
        obs_position, obs_start_time, obs_duration = obs
        if time_step >= obs_start_time and time_step < obs_start_time + obs_duration:
            obstacle_pixel_position = (obs_position[1] * block_size, obs_position[0] * block_size)

            # Snake collisions
            if snake_body_1[0] == list(obstacle_pixel_position) or snake_body_2[0] == list(obstacle_pixel_position):
                print(f"Snake hit a dynamic obstacle at timestep {time_step}!")
                game_over()

    # Clear the game window
    game_window.fill(black)

    # Draw Dynamic Obstacles
    for obs in dynamic_obstacles:
        obs_position, obs_start_time, obs_duration = obs
        if time_step >= obs_start_time and time_step < obs_start_time + obs_duration:
            j, i = obs_position
            game_window.blit(bomb_image, (j * block_size, i * block_size))

    # Draw Snake 1 (Green with a distinct head)
    for index, pos in enumerate(snake_body_1):
        if index == 0:  # Head of the snake
            pygame.draw.circle(game_window, green_head, (pos[0] + block_size // 2, pos[1] + block_size // 2), block_size // 2)
        else:
            pygame.draw.rect(game_window, green_body, pygame.Rect(pos[0], pos[1], block_size, block_size))

    # Draw Snake 2 (Blue with a distinct head)
    for index, pos in enumerate(snake_body_2):
        if index == 0:  # Head of the snake
            pygame.draw.circle(game_window, blue_head, (pos[0] + block_size // 2, pos[1] + block_size // 2), block_size // 2)
        else:
            pygame.draw.rect(game_window, blue_body, pygame.Rect(pos[0], pos[1], block_size, block_size))

    # Draw Goal Positions
    game_window.blit(banana_image, (fruit_position_1[0], fruit_position_1[1]))
    game_window.blit(pineapple_image, (fruit_position_2[0], fruit_position_2[1]))

    # Update game display
    pygame.display.update()
    time_step += 1  # Increment time step
    fps.tick(snake_speed)  # Control game speed
