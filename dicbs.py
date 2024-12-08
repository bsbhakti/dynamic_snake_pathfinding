import heapq
from pdb import set_trace as bp
from collections import defaultdict
from utils import get_location
from lpa_star import lpa_star

def slpa_search(start, goal, env, constraints=None, start_time=0):
    open_set = []
    start_state = (start, start_time)
    g_score = {start_state: 0}
    f_score = {start_state: heuristic(start, goal)}
    heapq.heappush(open_set, (f_score[start_state], start, start_time))
    came_from = {}

    while open_set:
        _, current_pos, current_time = heapq.heappop(open_set)
        current_state = (current_pos, current_time)

        if current_pos == goal:
            return reconstruct_path(came_from, current_state)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]:  # Include waiting
            neighbor = (current_pos[0] + dx, current_pos[1] + dy)
            next_time = current_time + 1
            neighbor_state = (neighbor, next_time)

            if not env.is_valid(neighbor):
                continue

            if constraints and is_constrained(neighbor, next_time, constraints):
                continue

            tentative_g_score = g_score[current_state] + 1

            if neighbor_state not in g_score or tentative_g_score < g_score[neighbor_state]:
                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = tentative_g_score
                f_score[neighbor_state] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor_state], neighbor, next_time))

    return None  # No path found

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
        if(get_location(shorter_list,time) == pos):
            # print(get_location(path2,time), pos)
            return {'a1': agent1, 'a2': agent2, 'loc': [pos], 'timestep':time, 'vertex': True}
        #edge collision
        if(get_location(shorter_list,time+1) == pos and get_location(shorter_list,time) == get_location(longer_list,time+1)):
            # print("found edge collision", pos,get_location(path2,time+1), get_location(path1,time-1), time, i )
            return {'a1': agent1, 'a2': agent2, 'loc': [pos,get_location(longer_list,time+1)], 'timestep':time+1, 'vertex': False}

def detect_collisions(paths):
    res = []

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
        constraints = constraints_per_agent.get(agent_id, set())
        path = lpa_star(agent,goal,problem,agent_id, heuristics)
        # path = slpa_search(agent,goal,problem.map)
        if not path:
            return None
        print("path was found by lpa")
        paths.append(path)
    print("initial paths found by cbs_h2 ", paths)
    return paths

def is_agent_affected(path, EC_t):
    for i,pos in enumerate(path):
        if (pos, i) in EC_t:
            return True
    return False

def dicbs(agents, goals, heuristics, dynamic_obstacles,problem ,alpha=3): #agents is start loc of agents
    t = 0
    EC = set()
    ECT = {}
    constraints = {agent_id: {} for agent_id in range(len(agents))}

    paths = cbs_h2(agents, goals, heuristics, constraints, problem)
  
    if not paths:
        return None
    ECT['Root'] = {
        'cost': sum(len(path) for path in paths),
        'constraints': {agent_id: constraints[agent_id].copy() for agent_id in constraints},
        'paths': [path.copy() for path in paths]
    }

    max_time = max(start_time + duration for _, start_time, duration in dynamic_obstacles)

    while t <= max_time:
        EC_t = environment_changes(EC, t, dynamic_obstacles)
        if not EC_t and t > 0:
            collisions = detect_collisions(paths)
            print("this is collision ", collisions)
            
            if not collisions:
                print("no conflict and env change")
                t += 1
                continue  # Ensure the loop continues until max_time

        else:
            EC = EC_t.copy()
        print("this is ECT ", EC_t)

        new_constraints = {agent_id: constraints[agent_id].copy() for agent_id in constraints}
        affected_agents = set()

        print("this is ECT ", EC_t)
        print("this is new constraints ", new_constraints)
        print("affected agent before ", affected_agents)

        # for agent_id, path in enumerate(paths):
        #     if is_agent_affected(path, EC_t):
        #         affected_agents.add(agent_id)
        #         print("this agent is affected ", agent_id, change)
        #         for change in EC_t:
        #             new_constraints[agent_id].add(change)

        for change in EC_t:
            for agent_id in range(len(agents)):
                if is_agent_affected(paths[agent_id], EC_t):
                    print("this is change ", change)
                    print("this agent is affected ", agent_id)
                    agent_constraints = new_constraints[agent_id]
                    agent_constraints['env'] = change
                    affected_agents.add(agent_id)


        # for agent_id in range(len(agents)):
        #     agent_constraints = new_constraints[agent_id]
        #     for change in EC_t:
        #         print("this is change ", change)
        #         agent_constraints['env'] = change
        #     if is_agent_affected(paths[agent_id], EC_t):
        #         print("this agent is affected ", agent_id)
        #         affected_agents.add(agent_id)
        collisions = detect_collisions(paths)
        if(len(collisions) > 0):
            collision_cons = standard_splitting(collisions[0])
            print("these are the constraints ", collision_cons)

            for constraint in collision_cons:
                agent = constraint["agent"]
                new_constraints[agent].add(constraint)

        # conflicts = detect_conflicts(paths)
        # for conflict_type, agent1, agent2, conflict_data, time in conflicts:
        #     if conflict_type == 'vertex':
        #         # Vertex conflict: two agents occupying the same position
        #         constraints[agent1].add((conflict_data, time))
        #         constraints[agent2].add((conflict_data, time+1))
        #     elif conflict_type == 'edge':
        #         # Edge conflict: two agents swapping positions
        #         current_pos, next_pos = conflict_data
        #         constraints[agent1].add(((current_pos, next_pos), time))
        #         constraints[agent2].add(((next_pos, current_pos), time))
        #     elif conflict_type == 'waiting':
        #         # Waiting conflict: agent1 is waiting, agent2 wants to pass through
        #         constraints[agent1].add((conflict_data, time))  # Prevent agent1 from waiting
        #         constraints[agent2].add((conflict_data, time))  # Prevent agent2 from entering at the same time
        #     # affected_agents.update([agent1, agent2])

        print("this is affected agents ", affected_agents)
        for agent_id in affected_agents:
            backtrack_time = max(0, t - alpha)
            previous_path = paths[agent_id]
            print("prev path ", previous_path)
            print("backtrack time ", backtrack_time)

            start_position = previous_path[backtrack_time]
            start_time = len(previous_path) - backtrack_time
            # new_path = slpa_search(
            #     start_position, goals[agent_id], env,
            #     constraints[agent_id], start_time=start_time
            # )
            print("trying to find new path ", start_position, goals[agent_id])
            new_path = None
            new_path = lpa_star(start_position,goals[agent_id], problem, agent_id, heuristics)
            if new_path:
                paths[agent_id] = previous_path[:backtrack_time] + new_path
                ECT[agent_id] = {'constraints': constraints[agent_id].copy(), 'path': paths[agent_id].copy()}
            else:
                return None

        constraints = new_constraints
        ECT['Root']['constraints'] = {agent_id: constraints[agent_id].copy() for agent_id in constraints}
        ECT['Root']['paths'] = [path.copy() for path in paths]
        t += 1

    if not validate_paths_against_obstacles(paths, dynamic_obstacles):
        return None

    return paths

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