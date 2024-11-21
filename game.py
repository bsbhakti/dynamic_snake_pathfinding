# Got this code from https://www.geeksforgeeks.org/snake-game-in-python-using-pygame-module/
import pygame
import time
import random
from single_agent_planner import compute_heuristics
from single_agent_solver import SingleAgentSolver
from run_exp import import_mapf_instance

my_map, starts, goals = import_mapf_instance("exp1.txt")
solver = SingleAgentSolver(my_map, starts, goals) 
print("Start: ",starts[0])
print("Goals: ",goals[0])


snake_speed = 10

# Window size
window_x = 600
window_y = 400


# defining colors
black = pygame.Color(0, 0, 0)
white = pygame.Color(255, 255, 255)
red = pygame.Color(255, 0, 0)
green = pygame.Color(0, 255, 0)
blue = pygame.Color(0, 0, 255)

# fruit_image = pygame.image.load('a1.jpeg')
# fruit_image = pygame.transform.scale(fruit_image, (10, 10))
# Initialising pygame
pygame.init()

# Initialise game window
pygame.display.set_caption('Snake game')
game_window = pygame.display.set_mode((window_x, window_y))

# FPS (frames per second) controller
fps = pygame.time.Clock()
# defining snake default position 
snake_position = [starts[0][0]*10, starts[0][1]*10]

# defining first 4 blocks of snake
# body
snake_body = [  [100, 50],
                [90, 50],
                [80, 50],
                [70, 50]
            ]
fruit_position = [goals[0][0]*10, goals[0][1]*10]
prev_fruit_position = starts

# fruit_position = [random.randrange(1, (window_x//10)) * 10,
#                   random.randrange(1, (window_y//10)) * 10]
fruit_spawn = True

# setting default snake direction 
# towards right
direction = 'RIGHT'
change_to = direction
# initial score
score = 0

# displaying Score function
def show_score(choice, color, font, size):
  
    # creating font object score_font 
    score_font = pygame.font.SysFont(font, size)
    
    # create the display surface object
    # score_surface
    score_surface = score_font.render('Score : ' + str(score), True, color)
    
    # create a rectangular object for the 
    # text surface object
    score_rect = score_surface.get_rect()
    
    # displaying text
    game_window.blit(score_surface, score_rect)
    # game over function
def game_over():
    print("game over", snake_position,snake_body)
    # creating font object my_font
    my_font = pygame.font.SysFont('times new roman', 50)
    
    # creating a text surface on which text 
    # will be drawn
    game_over_surface = my_font.render('Your Score is : ' + str(score), True, red)
    
    # create a rectangular object for the text
    # surface object
    game_over_rect = game_over_surface.get_rect()
    
    # setting position of the text
    game_over_rect.midtop = (window_x/2, window_y/4)
    
    # blit will draw the text on screen
    game_window.blit(game_over_surface, game_over_rect)
    pygame.display.flip()
    
    # after 2 seconds we will quit the 
    # program
    time.sleep(2)
    
    # deactivating pygame library
    pygame.quit()
    
    # quit the program
    quit()
    # Main Function

def make_random_move(snake_x,snake_y):
    moves = [[snake_x+10,snake_y], [snake_x-10,snake_y], [snake_x,snake_y-10],[snake_x,snake_y+10]]
    random_move =  random.choice(moves)
    # print(f"rnadom move:{random_move}, snake_body: {snake_body}]")
    while(random_move in snake_body):
        random_move =  random.choice(moves)
    return random_move

def find_path():
    return solver.find_solution()

count = 0
while True and count <1:
    count+=1
  
    # # handling key events
    # for event in pygame.event.get():
    #     if event.type == pygame.KEYDOWN:
    #         if event.key == pygame.K_UP:
    #             change_to = 'UP'
    #         if event.key == pygame.K_DOWN:
    #             change_to = 'DOWN'
    #         if event.key == pygame.K_LEFT:
    #             change_to = 'LEFT'
    #         if event.key == pygame.K_RIGHT:
    #             change_to = 'RIGHT'

    # # If two keys pressed simultaneously 
    # # we don't want snake to move into two directions
    # # simultaneously
    # if change_to == 'UP' and direction != 'DOWN':
    #     direction = 'UP'
    # if change_to == 'DOWN' and direction != 'UP':
    #     direction = 'DOWN'
    # if change_to == 'LEFT' and direction != 'RIGHT':
    #     direction = 'LEFT'
    # if change_to == 'RIGHT' and direction != 'LEFT':
    #     direction = 'RIGHT'

    # # Moving the snake
    # if direction == 'UP':
    #     snake_position[1] -= 10
    # if direction == 'DOWN':
    #     snake_position[1] += 10
    # if direction == 'LEFT':
    #     snake_position[0] -= 10
    # if direction == 'RIGHT':
    #     snake_position[0] += 10
    # snake_position = make_random_move(snake_position[0],snake_position[1])
    # goal_for_solver = (fruit_position[0]//10, fruit_position[1]//10)
    # solver.goals = list(goal_for_solver)
    # for goal in goals:
    #         solver.heuristics = []
    #         solver.heuristics.append(compute_heuristics(my_map, goal_for_solver))
    # solver.heuristics = compute_heuristics(my_map,goal_for_solver)
    # print("this is huerisitic ", solver.heuristics)
    # solver.starts = [prev_fruit_position]
    print("this is prev",[prev_fruit_position[0]//10, prev_fruit_position[1]//10] )
    print("this is fruit",[fruit_position[0]//10, fruit_position[1]//10] )


    solver = SingleAgentSolver(my_map,(prev_fruit_position[0]//10, prev_fruit_position[1]//10),(fruit_position[0]//10, fruit_position[1]//10))
    snake_path = find_path()
    for snake_position in snake_path[0]:
    # Snake body growing mechanism 
    # if fruits and snakes collide then scores will be 
    # incremented by 10
        snake_position = [snake_position[0]*10, snake_position[1]* 10]
        print("Snake pos", snake_position)
        snake_body.insert(0, list(snake_position))
        print(f"Snake: {snake_position}, Fruit: {fruit_position}, Snake body: {snake_body}")

        if snake_position[0] == fruit_position[0] and snake_position[1] == fruit_position[1]:
            score += 10
            fruit_spawn = False
        else:
            snake_body.pop()
            
        if not fruit_spawn:
            prev_fruit_position = fruit_position
            fruit_position = [random.randrange(1, (window_x//10)) * 10, 
                            random.randrange(1, (window_y//10)) * 10]
            
        fruit_spawn = True
        game_window.fill(black)
        
        for pos in snake_body:
            # print("this is out", pos)
            
            pygame.draw.rect(game_window, green, pygame.Rect(
            pos[0], pos[1], 10, 10),border_radius=5)
            
        pygame.draw.rect(game_window, white, pygame.Rect(
        fruit_position[0], fruit_position[1], 10, 10),border_radius=2)

        # game_window.blit(fruit_image, (fruit_position[0], fruit_position[1])) 

        # Game Over conditions
        if snake_position[0] < 0 or snake_position[0] > window_x-10:
            game_over()
        if snake_position[1] < 0 or snake_position[1] > window_y-10:
            game_over()
        
        # Touching the snake body
        for block in snake_body[1:]:
            if snake_position[0] == block[0] and snake_position[1] == block[1]:
                game_over()
        
        # displaying score continuously
        show_score(1, white, 'times new roman', 20)
        
        # Refresh game screen
        pygame.display.update()

        # Frame Per Second /Refresh Rate
        fps.tick(snake_speed)
        