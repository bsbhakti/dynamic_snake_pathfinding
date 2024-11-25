from utils import euclidean_distance, compute_heuristics
from lpa_star import lpa_star
import time as timer


class SnakeProblem:
    def __init__(self,map):
        self.map = map
        self.num_of_agents = 1 # change this to reflect the actual number of agents
        self.goals = [(3,3)] # change this to be the fruit position
        self.starts = [(1,0)]  # change this to be the snake position
        self.num_expanded = 0
        self.maxX = 10
        self.maxY = 10
        

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            local = {}
            for loc in self.get_all_states():
                local[loc] = euclidean_distance(loc, goal)
            self.heuristics.append(local)
        # print("this is h n", self.heuristics)

    def get_all_states(self):
        all_states = []
        for i in range(0, self.maxX):
            for j in range(0, self.maxY):
                all_states.append((i,j))
        return all_states

    def possible_moves(self, loc): # change this to account for boundaries?
       
        left = (loc[0]-1, loc[1] )
        right  = (loc[0]+1, loc[1] )
        down = (loc[0], loc[1]-1 )
        up = (loc[0], loc[1]+1)
        moves = [left, right, down, up]
        # return [ ,, ,, (loc[0], loc[1] )  ] #does the snake wait?
        # print("moves", moves)
        valid_moves = [move for move in moves if 0 <= move[0] < self.maxX and 0 <= move[1] < self.maxY]
        return valid_moves
        
    
    def get_all_successors(self, loc):
        succ = self.possible_moves(loc)
        print("these are all the succ ", succ)
        return succ



    def find_solution(self):
        start_time = timer.time()
        result = []

        for i in range(self.num_of_agents):  # Find path for each agent
            lpa_star(self,self.starts[i],self.goals[i],i,self.heuristics[i])
            # if path is None:
            #     raise BaseException('No solutions')
            # result.append(path)

        ##############################

        # self.CPU_time = timer.time() - start_time

        # print("\n Found a solution! \n")
        # print("CPU time (s):    {:.2f}".format(self.CPU_time))
        # # print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        # return result





    