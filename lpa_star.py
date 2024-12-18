import heapq
from collections import defaultdict

from single_agent_planner import is_constrained
path_costs = {
    ((0, 0), (0, 1)): 1,  # Right from (0,0) to (0,1)
    ((0, 1), (0, 0)): 1,  # Left from (0,1) to (0,0)
    ((0, 0), (1, 0)): 1,  # Down from (0,0) to (1,0)
    ((1, 0), (0, 0)): 1,  # Up from (1,0) to (0,0)
    ((0, 1), (1, 1)): 1,  # Down from (0,1) to (1,1)
    ((1, 1), (0, 1)): 1,  # Up from (1,1) to (0,1)
    ((1, 0), (1, 1)): 1,  # Right from (1,0) to (1,1)
    ((1, 1), (1, 0)): 1   # Left from (1,1) to (1,0)
}


def lpa_star(start_loc, goal_loc,problem, agent_id, heuristics, agent_constraints=None, replan = False, agent_state=False, path = None):
    if(replan):
        ##print("REPLANNING")
        gVal = agent_state[0]
        rhs = agent_state[1]
        prev_cons = agent_state[2]
        # #print("replan",prev_cons)
        open_list = agent_state[3]
        path = path
        # ##print(gVal,"\n", rhs, "\n", open_list, "\n")
    else:
        prev_cons = None
        path = None
        open_list = []
        rhs = defaultdict(lambda: float('inf'))  # Lazy initialization to inf
        gVal = defaultdict(lambda: float('inf'))  # Lazy initialization to inf
    
    #####print("this is goal", goal_loc)
    # rhs[goal_loc] = 
    # gVal[goal_loc] = 0

    vertex_cons = {}
    edge_cons = {}
    #print("received this cons for agent isnide lpa ", agent_id, agent_constraints, start_loc)
    h_values = heuristics[agent_id]
    if(agent_constraints):
        agent_constraints = agent_constraints["negative"] #JUST EXTRACTING NEGATIVE FOR NOW
        ###print("in lpa star, agent cons",agent_id,agent_constraints)
        for i in agent_constraints["env"]:
            if(i in agent_constraints["vertex"]):
                vertex_cons[i] = agent_constraints["env"][i],agent_constraints["vertex"][i]
            else:
                vertex_cons[i] = agent_constraints["env"][i]

        for i in agent_constraints["vertex"]:
            if(i not in vertex_cons):
                vertex_cons[i] = agent_constraints["vertex"][i]
        edge_cons = agent_constraints["edge"]
        ###print("this is vertex cons and env obstacles inside lpa ", vertex_cons)
        ###print("this is edge cons inside lpa ", edge_cons)


    def calculate_key(loc, time):
        return min(gVal[(loc, time)] + time, rhs[(loc, time)] + h_values[loc] + time), min(gVal[(loc, time)], rhs[(loc, time)])


    def initialize():
        if not replan:
            ##print("NOT REPLANNING")
            rhs[(start_loc, 0)] = 0
            push_node(open_list, start_loc, calculate_key(start_loc, 0),0)
         
        else:
            rhs[(start_loc, 0)] = 0
            gVal[(start_loc, 0)] = float('inf')
            ##print("init replanning", goal_loc, gVal[(start_loc,0)], rhs[(start_loc, 0)], gVal, rhs)
            push_node(open_list, start_loc, calculate_key(start_loc, 0),0)
            # ##print(gVal, "\n", rhs, "\n", open_list, "\n")
            return


    def update_vertex(node_loc, node_time, agent_cons=None, is_constrained=False):
        # #print("update vertex called ", node_loc, node_time,   rhs[(node_loc, node_time)])
        if node_loc in vertex_cons.get(node_time, []):
            # #print("THIS IS constrained loc ", node_loc, node_time)
            rhs[(node_loc,node_time)] = float('inf')
            return
        if (node_loc, node_time) != (start_loc, 0):
            # Get predecessors (or successors in case of bidirectional checks)
            rhs[(node_loc, node_time)] = float('inf')
            next_states, _ = problem.get_all_successors(node_loc, agent_cons, node_time-1)
            for succ in next_states:
                # Check vertex constraint
                if node_time - 1 in vertex_cons.get(succ, []):
                    # #print("vertex cons on  ",[succ, node_loc], node_time )
                    continue  # Skip constrained states
                if edge_cons.get(node_time) and [succ, node_loc] in edge_cons.get(node_time, []):
                    ##print("edge cons on  ",[succ, node_loc], node_time )
                    continue
                c = 1  # Edge cost
                # if(node_loc == (0,0) or node_loc == (0,1)):
                # #print("succ is ", succ, gVal[(succ, node_time - 1)], node_time-1)
                rhs[(node_loc, node_time)] = min(rhs[(node_loc, node_time)], gVal[(succ, node_time - 1)] + c)
            # if(node_loc == (0,0) or node_loc == (0,1)):
            # #print("updated rhs val of ", (node_loc,node_time),  rhs[(node_loc, node_time)], gVal[(node_loc, node_time)])
            # Update the open list
            remove_node(open_list, (node_loc, node_time))
            if gVal[(node_loc, node_time)] != rhs[(node_loc, node_time)]:
                ##print("pushing node with key ", node_loc, node_time,calculate_key(node_loc, node_time))
                push_node(open_list, node_loc, calculate_key(node_loc, node_time), node_time)

    def compute_shortest_path(agent_cons=None, time=0):
        # ##print("open list and goal ",open_list, goal_loc)
        while (
            top_key(open_list)[0] < calculate_key(goal_loc, time)[0] or
            rhs[(goal_loc, time)] != gVal[(goal_loc, time)]
        ):
            loc, time = pop_node(open_list)

            if gVal[(loc, time)] > rhs[(loc, time)]:
                gVal[(loc, time)] = rhs[(loc, time)]
                next_states, _ = problem.get_all_successors(loc, agent_cons, time)
                for succ in next_states:
                    update_vertex(succ, time + 1, agent_cons)
            else:
                gVal[(loc, time)] = float('inf')
                next_states, _ = problem.get_all_successors(loc, agent_cons, time)
                for succ in next_states:
                    update_vertex(succ, time + 1, agent_cons)
                update_vertex(loc, time + 1, agent_cons)
        goal_time = None
        min_gval = float('inf')

        for t in range(0, 1000):  # Arbitrary upper limit for time
            if (goal_loc, t) in gVal and gVal[(goal_loc, t)] < min_gval:
                min_gval = gVal[(goal_loc, t)]
                goal_time = t

        # ##print(f"Goal time identified: {goal_time} with gVal: {min_gval}")
        return goal_time

    def createPath(goal_loc, goal_time, agent_cons=None):
        path = []  # To store the reconstructed path
        current_loc, current_time = goal_loc, goal_time  # Start at goal location and time

        # ###print("Starting path reconstruction from:", (current_loc, current_time))
        while (current_loc, current_time) != (start_loc, 0):  # Stop when reaching the start
            path.append((current_loc, current_time))
            temp = float('inf')  # Initialize to find the minimum gVal
            best_prev = None  # Best predecessor state

            # Loop through predecessors (or successors with time-1)
            next_states, _ = problem.get_all_successors(current_loc, agent_cons, current_time - 1)

            for prev_loc in next_states:
                if (prev_loc, current_time - 1) in gVal:
                    if gVal[(prev_loc, current_time - 1)] < temp:
                        temp = gVal[(prev_loc, current_time - 1)]
                        best_prev = prev_loc

            if best_prev is None:
                ###print("Error: No valid predecessor found! Path reconstruction failed.")
                return []

            # Move to the best predecessor
            current_loc = best_prev
            current_time -= 1

        path.append((start_loc, 0))  # Add the start location
        # ###print("Path reconstruction complete.")
        return path[::-1]  # Reverse the path to go from start to goal

    def main(path, prev_cons):
        f = True
        endPath = []
        initialize()
        time =  0 
        itr = 0
        # prev_cons = ['']

        while f:
            if itr > 100:
                #print("Max iterations reached, terminating.")
                return path
           
            if(replan):
                #print("Top Key in open_list: ", top_key(open_list))
                if(top_key(open_list) == (float('inf'), float('inf'))):
                    #print("open list inf", open_list)
                    return
                #print("Goal RHS:", rhs.get((goal_loc, time), float('inf')))
                #print("Goal gVal:", gVal.get((goal_loc, time), float('inf')))

            itr+=1

            goal_time =  compute_shortest_path(agent_constraints,time)
            ##print("Shortest path computation done.")
            path = createPath(goal_loc,goal_time,agent_constraints)

            
            # #print("path", path)
            # return
            if(prev_cons != agent_constraints and agent_constraints):
                # return
                ##print(agent_constraints)
                if(not prev_cons):
                    prev_cons = agent_constraints
                #print("lpa star agent cons have changed ")
                for type in prev_cons:
                    # #print("this is type inside ", type)
                    if(type =="vertex" or type == "env"):
                        for t in prev_cons[type]:
                            for pos in prev_cons[type][t]:
                                if(prev_cons[type][t] not in agent_constraints[type][t]):
                                    #print("this is cons inside ",t, prev_cons[type][t], agent_constraints[type][t])
                                    gVal[(pos, t)] = float('inf')
                                    rhs[(pos, t)] = float('inf')
                                    succs,_ = problem.get_all_successors(pos,agent_constraints, t +1) #want to change cost of u->v
                                    for succ in succs:
                                        update_vertex(succ,t+1,agent_constraints, True)
                                        # start_loc = pos
                                    time = t
                                    # return 
                            # break #???
                    else:
                        for t in prev_cons[type]:
                            for pos in prev_cons[type][t]:
                                #print("WRITE EDGE CONS STUFF")
                                
                                pass
                prev_cons = agent_constraints  
                
                
            else:
                f = False   
        #print("changin prev cons", prev_cons)        
        return (path, gVal, rhs, prev_cons, open_list)
    
    return (main(path, prev_cons))


def push_node2(open_list, node, priority):
    # #####print("pushing node ", node)
    heapq.heappush(open_list, (priority[0],priority[1],(node[0],node[1])))
    # #####print("this is open " ,open_list)

def push_node(open_list, node_loc, priority, time):
    heapq.heappush(open_list, (priority[0], priority[1], time, node_loc))

  

def pop_node(open_list):
    if(len(open_list) == 0):
        return
    # ####print("this is open list ", open_list)
    _, _, time,node = heapq.heappop(open_list)
    # #print("popped ", node)
    return (node,time)


def top_key(open_list):
    if len(open_list) == 0:
        return (float('inf'), float('inf'))
    # ####print("this is top",open_list[0])
    # #print("\nopen list", open_list, "\n")
    return open_list[0]

def remove_node(open_list, node):
    rem = (float('-inf'), float('-inf'))
    update(open_list,node,rem)
    a = pop_node(open_list)

def update(open_list, node, priority):
    loc, time = node  # Unpack location and time

    for index, (p1, p2, t, loc_in_list) in enumerate(open_list):
        # Check if node (location, time) matches
        if loc == loc_in_list and t == time:
            # If the new priority is not better, do nothing
            if (p1, p2) <= priority:
                return
            # Remove the old entry and add the updated one
            del open_list[index]
            heapq.heappush(open_list, (priority[0], priority[1], time, loc))
            heapq.heapify(open_list)
            return

    # If the node was not found, add it to the open list
    push_node(open_list, node[0], priority, node[1])
def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']
