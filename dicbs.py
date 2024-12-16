import heapq
from pdb import set_trace as bp
from collections import defaultdict
from utils import get_location
from lpa_star import lpa_star
from copy import deepcopy



def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_constrained(position, time, constraints):
    """Check if a vertex or edge constraint exists."""
    return (position, time) in constraints

def is_edge_constrained(current_pos, next_pos, time, constraints):
    """Check if an edge constraint exists for a swap conflict."""
    return ((current_pos, next_pos), time) in constraints

def detect_collision(path1, path2, i,j):
    if len(path1) > len(path2):
       longer_list = path1
       shorter_list = path2
       agent1 = i
       agent2 = j
    else:
        longer_list = path2
        shorter_list = path1
        agent1 = j
        agent2 = i

    for time, pos in enumerate(longer_list):
        #vertex collision
        pos = pos[0]
        # print("checking ", time,(get_location(shorter_list,time), pos))
        if(get_location(shorter_list,time) == pos):
            # print(get_location(path2,time), pos, time)
            return {'a1': agent1, 'a2': agent2, 'loc': [pos], 'timestep':time, 'vertex': True}
        #edge collision
        if(get_location(shorter_list,time+1) == pos and get_location(shorter_list,time) == get_location(longer_list,time+1)):
            # print("found edge collision", pos,get_location(path2,time+1), get_location(path1,time-1), time, i )
            return {'a1': agent1, 'a2': agent2, 'loc': [pos,get_location(longer_list,time+1)], 'timestep':time+1, 'vertex': False}

def detect_collisions(paths):
    res = []
    print("detecting collision for ", paths)

    for i, p1 in enumerate(paths):
        for j, p2 in enumerate(paths[i+1:]):
            # print()
            collision = detect_collision(p1,p2,i,i+j+1)
            if(collision is not None):
                res.append(collision)
    # print("these are first collisions",res)
    return res

def standard_splitting(collision):
    res = []
    # print(collision)
    if(collision is None):
        return None

    if(collision['vertex']):
        cons1 = {"agent": collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False, "positive": False}
        cons2 = {"agent": collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":True,"end":False,  "positive": False}
        res.append(cons1)
        res.append(cons2)

    else:
        # {'a1': i, 'a2': j, 'loc': [pos,get_location(path2,time+1)], 'timestamp':time+1, 'vertex': False}
        cons1 = {"agent": collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],"vertex":False,"end":False, "positive": False}
        cons2 = {"agent": collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'],"vertex":False,"end":False, "positive": False}
        res.append(cons1)
        res.append(cons2)

    # print("after standard splitting", res)
    return res

def environment_changes(EC_prev, t, dynamic_obstacles):
    EC_t = set()
    for obstacle in dynamic_obstacles:
        position, start_time, duration = obstacle
        if start_time <= t < start_time + duration:
            EC_t.add((position, t))
    for pos, time in EC_prev:
        if time >= t:
            EC_t.add((pos, time))
    return EC_t

def detect_conflicts(paths):
    """Detect both vertex, edge, and waiting conflicts."""
    conflicts = []
    positions_at_time = defaultdict(list)
    max_time = max(len(path) for path in paths)

    for t in range(max_time):
        positions_at_time.clear()

        # Detect vertex conflicts
        for agent_id, path in enumerate(paths):
            position = path[t] if t < len(path) else path[-1]
            positions_at_time[position].append((agent_id, t))

        for position, agents_and_times in positions_at_time.items():
            # Vertex conflicts
            if len(agents_and_times) > 1:
                for i in range(len(agents_and_times)):
                    for j in range(i + 1, len(agents_and_times)):
                        agent_i, time_i = agents_and_times[i]
                        agent_j, time_j = agents_and_times[j]
                        if time_i == time_j:  # Same timestep conflict
                            conflicts.append(('vertex', agent_i, agent_j, position, time_i))

        # Detect edge conflicts
        for agent_id, path in enumerate(paths):
            if t + 1 < len(path):
                current_pos = path[t]
                next_pos = path[t + 1]
                for other_id, other_path in enumerate(paths):
                    if other_id != agent_id and t + 1 < len(other_path):
                        other_current_pos = other_path[t]
                        other_next_pos = other_path[t + 1]
                        if current_pos == other_next_pos and next_pos == other_current_pos:
                            conflicts.append(('edge', agent_id, other_id, (current_pos, next_pos), t + 1))

        # Detect waiting conflicts
        for agent_id, path in enumerate(paths):
            if t < len(path) and t + 1 < len(path):
                position = path[t]
                if position == path[t + 1]:  # Agent is waiting
                    for other_id, other_path in enumerate(paths):
                        if other_id != agent_id and t + 1 < len(other_path):
                            other_position = other_path[t + 1]
                            if other_position == position:  # Another agent wants to pass through
                                conflicts.append(('waiting', agent_id, other_id, position, t + 1))
    print("this is conflicts ",conflicts)
    return conflicts

def cbs_h2(agents, goals, heuristics, constraints_per_agent, problem):
    paths = []
    for agent_id, (agent, goal) in enumerate(zip(agents, goals)):
        constraints = constraints_per_agent.get(agent_id, {})
      
        path = lpa_star(agent,goal,problem,agent_id, heuristics)
        # path = slpa_search(agent,goal,problem.map)
        if not path:
            return None
        print("path was found by lpa")
        paths.append(path)
    print("initial paths found by cbs_h2 ", paths)
    return paths

def is_agent_affected(path, EC_t):
    for (pos,time) in (path):
        if (pos, time) in EC_t:
            return True
    return False

def add_cons(type, time, loc, agent_id, all_cons,type2 ): #type is if its pos or negatove type2 is vertex edge
    # print("this is the cons to add ", type, loc, time, agent_id, type2)
    if type not in all_cons[agent_id]:
        all_cons[agent_id][type] = {}
    if type2 not in all_cons[agent_id][type]:
        all_cons[agent_id][type][type2] = {}
    if time not in all_cons[agent_id][type][type2]:
            all_cons[agent_id][type][type2][time] = []

    # Append the location to the list
    if loc not in all_cons[agent_id][type][type2][time]:
        if(type2 == "vertex"):
            all_cons[agent_id][type][type2][time].append(loc[0])
        else:
            all_cons[agent_id][type][type2][time].append(loc)
    # print("here", loc, all_cons)  
    return all_cons


    
def dicbs(agents, goals, heuristics, dynamic_obstacles,problem ,alpha=3): #agents is start loc of agents
    t = 0
    EC = set()
    ECT = []
    num_of_generated = 0       
    constraints = {agent_id: {"positive":{"vertex":{}, "edge": {}, 'env': {}},"negative":{"vertex":{}, "edge": {}, 'env': {}}} for agent_id in range(len(agents))}
    # print("beg cons", constraints)
    collisions = []

    root_paths = cbs_h2(agents, goals, heuristics, constraints, problem)
  
    if not root_paths:
        return None
    root = {'cost': sum(len(path) for path in root_paths),
            'constraints':  {agent_id: constraints[agent_id].copy() for agent_id in constraints},
            'paths': [path.copy() for path in root_paths]}
    # ECT['Root'] = {
    #     'cost': sum(len(path) for path in paths),
    #     'constraints': {agent_id: constraints[agent_id].copy() for agent_id in constraints},
    #     'paths': [path.copy() for path in paths]
    # }
    root["collisions"] = detect_collisions(root_paths)
    num_of_generated =  push_node(ECT,root, num_of_generated)
    node = {}

    max_time = max(start_time + duration for _, start_time, duration in dynamic_obstacles)

    while t <= max_time or len(ECT) > 0 :
        print("Iteration: ", t)
        affected_agents = set()
        EC_t = environment_changes(EC, t, dynamic_obstacles)
        print("This is change ", EC_t)
        node = pop_node(ECT)
        print("this node popped ", node)
        newNode = deepcopy(node)
        if not EC_t and t > 0:
            if(node["collisions"] == []):
                print("no conflict and env change")
                t += 1
                continue  # Ensure the loop continues until max_time
            #if you have collisions you need to add constraints??

        else:
            EC = EC_t.copy()

        # new_constraints = {agent_id: constraints[agent_id].copy() for agent_id in constraints}

        # print("this is ECT ", EC_t)
        # print("affected agent before ", affected_agents)

        # Go through env changes and find cons and affected agents
        for change in EC_t:
            for agent_id in range(len(agents)):
                if is_agent_affected(newNode["paths"][agent_id], EC_t):
                    print("this is change ", change)
                    print("this agent is affected ", agent_id,)
                    agent_constraints = newNode["constraints"][agent_id]
                    print("this agent is affected ", agent_id,agent_constraints)

                    # agent_constraints['env'].append(change)
                    agent_constraints["negative"]['env'][change[1]] = [change[0]]

                    affected_agents.add(agent_id)
                    print("this is agent constraints for affected agent ", agent_id,  agent_constraints["negative"]['env'])
                    newNode["constraints"][agent_id] = agent_constraints

    # Find new paths for agents affected by env changes
        for agent_id in affected_agents:
            backtrack_time = max(0, t - alpha)
            previous_path = newNode["paths"][agent_id]
            # print("prev path ", previous_path)
            print("backtrack time ", backtrack_time)

            start_position = previous_path[backtrack_time]
            start_time = len(previous_path) - backtrack_time
            print("trying to find new path for agent with cons ",agent_id, newNode["constraints"][agent_id], start_position[0])
            new_path = None
            new_path = lpa_star(start_position[0],goals[agent_id], problem, agent_id, heuristics,newNode["constraints"][agent_id], True )
            if new_path:
                print("new path found for agent ", agent_id , new_path)
                newNode["paths"][agent_id] = previous_path[:backtrack_time] + new_path
                newNode["constraints"][agent_id] = {'constraints': constraints[agent_id].copy()}
            else:
                return None
        newNode["cost"] = sum(len(path) for path in newNode["paths"]),

        # Find collisions in the path changes according to env cons - if no env changes then newNode is just the node we popped
        if(newNode == node):
            collisions = node["collisions"]
        else:
            collisions = detect_collisions(newNode["paths"])
        print("these are the collisions ", collisions)

        if(len(collisions) > 0):
            collision_cons = standard_splitting(collisions[0])
            # print("these are the constraints ", collision_cons)
            for constraint in collision_cons:
                agent_id = constraint["agent"]
                childNode = {'cost': 0,
                'constraints': deepcopy(newNode["constraints"]), 
                'paths': deepcopy( newNode["paths"]),
                'collisions': []}
                # affected_agents.add(agent)
                if constraint["vertex"]:
                    childNode["constraints"] = add_cons("negative",constraint['timestep'],constraint['loc'],agent_id,childNode["constraints"],"vertex")
                   
                    # newNode["constraints"].append(constraint)
                else: 
                   childNode["constraints"]= add_cons("negative",constraint['timestep'],constraint['loc'],agent_id,childNode["constraints"],"edge")

                # new_constraints[agent].add(constraint)
                previous_path = childNode["paths"][agent_id]
            # print("prev path ", previous_path)
                backtrack_time = max(0, t - alpha)
                # print("backtrack time ", backtrack_time)
                start_position = previous_path[backtrack_time]
                start_time = len(previous_path) - backtrack_time
                print("trying to find new path for agent with cons ",agent_id, childNode["constraints"][agent_id], start_position[0])
                new_path = None
                new_path = lpa_star(start_position[0],goals[agent_id], problem, agent_id, heuristics,childNode["constraints"][agent_id], True )
                if new_path:
                    childNode["paths"][agent_id] = previous_path[:backtrack_time] + new_path
                    print("new path found for agent ", agent_id , childNode["paths"][agent_id])
                    print("all path in this node " , childNode["paths"])
                    childNode["cost"] = sum(len(path) for path in childNode["paths"])
                    childNode["collisions"] = detect_collisions(childNode["paths"])
                    print("this child generated with collisions ", childNode)
                    num_of_generated = push_node(ECT,childNode, num_of_generated)
                else:
                    return None
            print("this is affected agents and their cons", affected_agents,  childNode["constraints"][agent_id])

       
        # collisions = detect_collisions(paths)
        # print("collisions in the new path ", collisions)
        # constraints = new_constraints
        # ECT['Root']['constraints'] = {agent_id: constraints[agent_id].copy() for agent_id in constraints}
        # ECT['Root']['paths'] = [path.copy() for path in paths]
        t += 1
    print("end the thing ",t ,len(ECT), node)

    if not validate_paths_against_obstacles(node["paths"], dynamic_obstacles):
        return None

    return node["paths"]

def validate_paths_against_obstacles(paths, dynamic_obstacles):
    """Validate that the computed paths respect all dynamic obstacle constraints."""
    obstacle_constraints = {}
    for position, start_time, duration in dynamic_obstacles:
        for t in range(start_time, start_time + duration):
            obstacle_constraints[(position, t)] = True

    for path in paths:
        for position, time in path:
            if (position, time) in obstacle_constraints:
                return False
    return True

def push_node(open_list, node, num_of_generated):
        heapq.heappush(open_list, (node['cost'], len(node['collisions']),num_of_generated, node))
        return num_of_generated +1
        # print("Generate node {}".format(self.num_of_generated))
def pop_node(open_list):
        _, _, id, node = heapq.heappop(open_list)
        # print("Expand node {}".format(id))
        return node