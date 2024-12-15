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


def lpa_star(start_loc, goal_loc,problem, agent_id, heuristics, agent_constraints=None):
    open_list = []
    rhs = defaultdict(lambda: float('inf'))  # Lazy initialization to inf
    gVal = defaultdict(lambda: float('inf'))  # Lazy initialization to inf
    #print("this is goal", goal_loc)
    # rhs[goal_loc] = 
    # gVal[goal_loc] = 0
    start_node = {}
    node_map = {}
    vertex_cons = {}
    edge_cons = None
    print("received this cons for agent isnide lpa ", agent_id, agent_constraints)
    h_values = heuristics[agent_id]
    if(agent_constraints):
        agent_constraints = agent_constraints["negative"] #JUST EXTRACTING NEGATIVE FOR NOW
        for i in agent_constraints["env"]:
            if(i in agent_constraints["vertex"]):
                vertex_cons[i] = agent_constraints["env"][i],agent_constraints["vertex"][i]
            else:
                vertex_cons[i] = agent_constraints["env"][i]

        for i in agent_constraints["vertex"]:
            if(i not in vertex_cons):
                vertex_cons[i] = agent_constraints["vertex"][i]
        print(vertex_cons,"look here")
        # vertex_cons = agent_constraints["env"] + agent_constraints["vertex"]
        edge_cons = agent_constraints["edge"]
        print("this is vertex cons and env obstacles inside lpa ", vertex_cons)
        print("this is edge cons inside lpa ", edge_cons)


    def calculate_key(loc, time):
        return min(gVal[(loc, time)], rhs[(loc, time)]) + h_values[loc] + time, min(gVal[(loc, time)], rhs[(loc, )])


    def initialize():
       
        print("this is in init ", start_loc, goal_loc)
        print('this is agent cons inside lpa ', agent_constraints)
        rhs[start_loc] = 0
        start_node["loc"] = start_loc
        start_node['g_val']= gVal[start_loc] 
        start_node['h_val']= h_values[start_loc]
        start_node['parent']=  "root"
        start_node['time'] = 0
        start_node['priority'] = calculate_key(start_loc, 0)

        # rhs[start_loc] = 0
        rhs[(start_loc, 0)] = 0
        push_node(open_list, (start_loc, 0), calculate_key(start_loc, 0))
        # #print("calling push1")
        # push_node(open_list,start_node["loc"], calculate_key(start_loc, 0),0)

    def update_vertex(node_loc, node_time, agent_cons= None, is_constrained=False):
        loc = node_loc
        c = 1
        print("calling next vertex on ", node_loc)
        if node_loc != start_loc:
            m = float('inf')
            #print("calling succ3")
            # should be calling pred but because pred and succ are the same here we call succ
            next_states,invalid_succ = problem.get_all_successors(node_loc,agent_cons,node_time)
            print("this is invalid succ ", invalid_succ, next_states)
            rhs[node_loc] = float('inf')
            for succ  in next_states:
                # VERTEX AND ENV CONS
                if(succ in invalid_succ):
                    print("invalidating c ", succ, node_time)
                    c = float('inf')
                    gVal[succ] = float('inf')
                    rhs[succ] = float('inf')

                    # rhs[succ] = float('inf')
                else:
                    c = 1
                # if(succ in invalid_succ):
                #     print("invalidating gval and rhs ", succ, node_time)
                #     gVal[loc] = float('inf')
                #     rhs[loc] = float('inf')
                #     c = float('inf')
                # if(is_constrained):
                #     print("invalidating edge ", succ,node_loc, node_time)
                #     c = float('inf')
                #     # rhs[succ] = float('inf')
                # else:
                #     c = 1
                # if(is_constrained):
                #     print("invalidating gval and rhs ",  succ,node_loc, node_time)
                #     gVal[loc] = float('inf')
                #     rhs[loc] = float('inf')
                #     c = float('inf')

                # # EDGE CONS
                # if(node_time == 9000 and succ == (1,3) and node_loc == (2,3)):
                #     #print("updating edge costs between ", succ, node_loc)
                #     c = float('inf')
                #     # rhs[succ] = float('inf')
                # else:
                #     c = 1
                # if(node_time == 9000 and succ == (1,3) and node_loc == (2,3)):
                #     gVal[loc] = float('inf')
                #     rhs[loc] = float('inf')
                #     c = float('inf')
                
                
                new_gval = gVal[succ]+c
                #print("prev rhs of loc ", loc,  rhs[node_loc])
                rhs[node_loc] = min(rhs[node_loc], new_gval)
                # print("updated rhs of loc ", loc,  rhs[node_loc],gVal[node_loc], succ)

                # if(rhs[succ] < new_gval ):
                #     rhs[node_loc] = min(rhs[node_loc], new_gval)
                    # #print("updated rhs of loc ", loc,  rhs[node_loc], succ)


                    # #print("this is node map ", node_map)
                    # node["parent"] = node_map[(succ,node["time"]-1)]
            # #print("updating rhs of loc", node["loc"], m)
            # rhs[node["loc"]] = m
        k = calculate_key(node_loc,node_time)

        # if node_map.get((loc, node["time"])):  # If node exisats in the map
        remove_node(open_list, node_loc, node_time)
        #print("prev open list", open_list)
        #print("removing node, new priority",k,node_loc)
        if gVal[loc] != rhs[loc]:
            # node["priority"] = k
            print("pushing node", node_loc, gVal[node_loc])
            # node["time"] = node["time"] + 1
            push_node(open_list, node_loc, k, node_time)
            #print("new open list", open_list)

        # if(k[0], k[1],node["loc"],node in open_list):
        #     #print("removing")
        #     remove_node(open_list,node, node_map)
        # if(gVal[node["loc"]] != rhs[node["loc"]]):
        #     #print("pushing node ", node)
        #     push_node(open_list, node, calculate_key(node["loc"]),node_map)
        #print("done")

    def compute_shortest_path(agent_cons=None, time = 0):
        # #print("this is goal node's priority",calculate_key(goal_loc))
        while (top_key(open_list)[0] < calculate_key(goal_loc, time)[0] or top_key(open_list)[1] < calculate_key(goal_loc,time)[1])  or rhs[goal_loc] != gVal[goal_loc]:
            (loc,time) = pop_node(open_list)
            #print("node pooped", loc, time)

            if(gVal[loc] > rhs[loc]):
                gVal[loc] = rhs[loc]
                # node["g_val"] = rhs[node["loc"]]
                #print("updated gVal of node with loc ",loc,  gVal[loc])
                succs,_ =  problem.get_all_successors(loc,agent_cons,time)
                for succ in succs:
                    # print("got succ", succ)
                    # child = {'loc': succ,
                    #     'g_val': node['g_val'] + 1,
                    #     'h_val': h_values[succ],
                    #     'parent': node,
                    #     'time': node['time'] + 1,
                    #     'priority': calculate_key(succ) }
                    # child_priority  = calculate_key(succ, time)
                    #print("calling update here1",child_priority)
                    update_vertex(succ,time, agent_cons)
            else:
                gVal[loc] = float('inf')
                #print("calling succ")
                succs,_ =  problem.get_all_successors(loc,agent_cons,time)
                for succ in succs:
                    # child = {'loc': succ,
                    #     'g_val': node['g_val'] + 1,
                    #     'h_val': h_values[succ],
                    #     'parent': node,
                    #     'time': node['time'] + 1,
                    #     'priority': calculate_key(succ) }
                    #print("calling update here2", succ)
                    update_vertex(succ,time,agent_cons)
                #print("calling update here3", succ)
                
                update_vertex(succ,time,agent_cons)
        return True

    def createPath(goal_loc,time,agent_cons=None):
        way = []
        print("start loc inside create path ", start_loc, goal_loc)
        while goal_loc != start_loc:
            temp = float('inf')
            minState = None
            way.append(goal_loc)
            # looping over all successor nodes
            # print("calling succ2", gVal)
            if(goal_loc == (1,0)):
                print("goal is 1,0")
            next_states,_ = problem.get_all_successors(goal_loc,agent_cons,time)
            # print("done with get succ in create path")
            # if(time == 9000 and (1,2) in next_states):
            #     next_states.remove((1,2))

            # print(gVal)
    
            for successor in next_states:
                # print("this is succ", successor)
                if(successor == (0,0)):
                    print("found 0,0", gVal[successor])
                if temp > gVal[successor]:
                    temp = gVal[successor]
                    minState = successor
                    # print("changing min state ", minState)

            #updating goal state value
            goal_loc = minState
            # return
            # print("this is new goal", goal_loc)

        way.append(goal_loc)
        print("returning create path")
        return way[::-1]


  

    # # obstacles = {5:[(2,3)]} #vertex constraint
    # obstacles = {}
    # # obstacles_edge = {4: [[(1,3), (2,3)]] } #edge constraint
    # obstacles_edge = { } #edge constraint

    def main(start_loc):
        #print("in main")
        f = True
        endPath = []
        initialize()
        time = 0

        while f:
            print(f"Calling shortest path at time {time}", agent_constraints)
            f = not compute_shortest_path(agent_constraints,time)
            print("Shortest path computation done. Success:", not f)
            path = createPath(goal_loc,time,agent_constraints)
            print(path)

            for i,pos in enumerate(path[:len(path)-1]):
                next_pos = path[i+1]
                endPath.append(pos)
                #print("this is curr->next ", pos, next_pos)
                if(vertex_cons and i+1 in vertex_cons):
                    cons = vertex_cons[i+1]
                    print(i+1," time is constrained ", cons,agent_constraints,vertex_cons)
                  
                    if(next_pos in cons):
                        print("updating because cons",next_pos )
            
                        gVal[next_pos] = float('inf')
                        rhs[next_pos] = float('inf')

                        succs,_ =problem.get_all_successors(next_pos,agent_constraints, i+1) #want to change cost of u->v
                        for succ in succs:
                            update_vertex(succ,i+1,agent_constraints, True)
                            start_loc = pos
                            print("new start ", pos)
                            # break
                    else:
                        #print("loc not const")
                        break
                if(edge_cons and i in edge_cons):
                    consts = edge_cons[i]
                    print("edge")
                    for const in consts:
                        #print("look edge cons ", const)
                        if(const[0] == pos and const[1] == next_pos):
                            #print("violation of edge cons")
                            update_vertex(next_pos, i,agent_constraints)
                            start_loc = pos
                            break
                            # for succ,_ in problem.get_all_successors(next_pos,9000):
                            #     update_vertex(succ,9000)
                            #     start_loc = pos
                            #     # #print("new start ", pos)
                            #     break


                if next_pos is goal_loc:
                        f = False
                        print("goal found")
                        break
                time +=1
            print("Restarting path search...")
            compute_shortest_path(agent_constraints, time)
              
            # path = createPath(goal_loc,time)
            # #print(path)
            
            # return
            print("dome")
        print("calling create path")
        print(gVal)
        print(rhs)

        path = createPath(goal_loc,0, agent_constraints)
        print(path)
        return path
            # #print(reconstruct_path())
            # Waitfor changes in edge costs; 
            # {21} forall directed edges (u,v) with changed edge costs 
            # {22} Update the edge cost c(u,v); {23} UpdateVertex(v);
    
    return (main(start_loc))


def push_node(open_list, node_loc, priority, time):
    # #print("pushing node ", node)
    heapq.heappush(open_list, (priority[0],priority[1],time,node_loc))
    # #print("this is open " ,open_list)
  

def pop_node(open_list):
    if(len(open_list) == 0):
        return
    _, _, time,loc = heapq.heappop(open_list)
    return (loc,time)


def top_key(open_list):


    if len(open_list) == 0:
        return (float('inf'), float('inf'))
    #print("this is top",open_list[0])
    #print("open list", open_list)
    return open_list[0]

def remove_node(open_list, node_loc, time):
    rem = (float('-inf'), float('-inf'))
    update(open_list,node_loc,rem, time)
    a = pop_node(open_list)

def update(open_list, node_loc, priority,time):
    for index, (p1, p2,t, loc) in enumerate(open_list):
        if node_loc == loc and t == time:
                if (p1,p2) <= priority:
                    break
                del open_list[index]
                open_list.append((priority[0],priority[1], time, loc))
                heapq.heapify(open_list)
                break
    else:
            #print("calling push2")
            push_node(open_list,node_loc, priority,time)

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']




#get all successors returns the actual pos and the obstructed pos. 
# the obs pos will then be set to inf