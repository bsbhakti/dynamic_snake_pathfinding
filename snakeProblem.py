from single_agent_planner import is_constrained
from utils import euclidean_distance, compute_heuristics
import time as time
from dicbs import dicbs


class SnakeProblem:
    def __init__(self,map, agents, goals, dynamic_obstacles):
        self.map = map
        self.goals = goals # change this to be the fruit position
        self.starts = agents  # change this to be the snake position
        self.num_expanded = 0
        self.maxX = self.map.rows
        self.maxY = self.map.cols
        self.dynamic_obstacles = dynamic_obstacles
        

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
        wait = (loc[0], loc[1])

        moves = [left, right, down, up, wait]
        # return [ ,, ,, (loc[0], loc[1] )  ] #does the snake wait?
        # print("moves", moves)
        valid_moves = [move for move in moves if 0 <= move[0] < self.maxX and 0 <= move[1] < self.maxY]
        return valid_moves

    def is_constrained(self,loc,time,agent_cons):
        if(agent_cons):
            if(time in agent_cons["vertex"]):
                if(loc in agent_cons["vertex"][time]):
                    return True

            if(time in agent_cons["edge"]):
                if(loc in agent_cons["edge"][time]):
                    return True

            if(time in agent_cons["env"]):
                if(loc in agent_cons["env"][time]):
                    return True
        return False


    
    def get_all_successors(self, loc,agent_cons, curr_time = 0):
        # if time == 9000:
        #     succ = self.possible_moves(loc)
        #     if((1,2) in succ):
        #         succ.remove((1,2))
        #     # print("these are all the succ1 ", succ)

        #     return succ
        # print("this is agent cons", agent_cons)

        succ = self.possible_moves(loc)
        invalid_succ = []
        for pos in succ:
            # print("calling is_cons on ", pos,curr_time,agent_cons)
            if(self.is_constrained(pos, curr_time,agent_cons)):
                invalid_succ.append(pos)
            
        
        # print("these are all the succ ", succ, invalid_succ)
        return (succ,invalid_succ)

    def get_all_predecessors(self, loc,agent_cons, time = 0):
        # if time == 9000:
        #     succ = self.possible_moves(loc)
        #     if((1,2) in succ):
        #         succ.remove((1,2))
        #     # print("these are all the succ1 ", succ)

        #     return succ

        succ = self.possible_moves(loc)
        invalid_succ = []
        
        # print("these are all the succ ", succ)
        return succ




    def find_solution(self):
           # Measure time taken by dicbs
       
        paths = dicbs(self.starts, self.goals,self.heuristics, self.dynamic_obstacles, self,3, True)
        return paths
    


        # for i in range(self.num_of_agents):  # Find path for each agent
        #         lpa_star(self,self.starts[i],self.goals[i],i,self.heuristics[i])
                # if path is None:
                #     raise BaseException('No solutions')
                # result.append(path)

            ##############################

            # self.CPU_time = timer.time() - start_time

            # print("\n Found a solution! \n")
            # print("CPU time (s):    {:.2f}".format(self.CPU_time))
            # # print("Sum of costs:    {}".format(get_sum_of_cost(result)))

            # return result





    