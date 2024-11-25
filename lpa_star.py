import heapq

def lpa_star(problem, start_loc, goal_loc,agent,h_values):
    open_list = []
    rhs = {}
    gVal = {}
    start_node = {}
    node_map = {}

    def calculate_key(loc):
        return min(gVal[loc],rhs[loc]) + h_values[loc], min(gVal[loc],rhs[loc])


    def initialize():
        for state in problem.get_all_states():
            rhs[state] = gVal[state] = float('inf')
        rhs[start_loc] = 0
        start_node["loc"] = start_loc
        start_node['g_val']= gVal[start_loc] 
        start_node['h_val']= h_values[start_loc]
        start_node['parent']=  "root"
        start_node['time'] = 0
        start_node['priority'] = calculate_key(start_loc)

        rhs[start_loc] = 0
        print("calling push1")
        push_node(open_list,start_node, calculate_key(start_loc))

    def update_vertex(node):
        loc = node["loc"]
        if node["loc"] != start_loc:
            m = float('inf')
            next_states = problem.get_all_successors(node)
            
            for succ  in next_states:
                new_gval = gVal[succ]+1
                if(rhs[succ] < new_gval ):
                    rhs[node["loc"]] = min(rhs[node["loc"]], new_gval)
                    print("updated rhs of loc ", loc,  rhs[node["loc"]])


                    # print("this is node map ", node_map)
                    # node["parent"] = node_map[(succ,node["time"]-1)]
            # print("updating rhs of loc", node["loc"], m)
            # rhs[node["loc"]] = m
        k = calculate_key(node["loc"])

        # if node_map.get((loc, node["time"])):  # If node exisats in the map
        remove_node(open_list, node)
        print("removing node, new priority",k,node)
        if gVal[loc] != rhs[loc]:
            node["priority"] = k
            print("pushing node", node)
            push_node(open_list, node, k)

        # if(k[0], k[1],node["loc"],node in open_list):
        #     print("removing")
        #     remove_node(open_list,node, node_map)
        # if(gVal[node["loc"]] != rhs[node["loc"]]):
        #     print("pushing node ", node)
        #     push_node(open_list, node, calculate_key(node["loc"]),node_map)

    def compute_shortest_path():
        print("this is goal node's priority",calculate_key(goal_loc))
        while (top_key(open_list)[0] < calculate_key(goal_loc)[0] and top_key(open_list)[1] < calculate_key(goal_loc)[1])  or rhs[goal_loc] != gVal[goal_loc]:
            node = pop_node(open_list)
            print("node pooped", node)

            if(gVal[node["loc"]] > rhs[node["loc"]]):
                gVal[node["loc"]] = rhs[node["loc"]]
                node["g_val"] = rhs[node["loc"]]
                print("updated gVal of node with loc ",node["loc"],  gVal[node["loc"]])
                for succ in problem.get_all_successors(node):
                    child = {'loc': succ,
                        'g_val': node['g_val'] + 1,
                        'h_val': h_values[succ],
                        'parent': node,
                        'time': node['time'] + 1,
                        'priority': calculate_key(succ) }
                    print("calling update here1",child)
                    update_vertex(child)
            else:
                gVal[node["loc"]] = float('inf')
                for succ in problem.get_all_successors(node):
                    child = {'loc': succ,
                        'g_val': node['g_val'] + 1,
                        'h_val': h_values[succ],
                        'parent': node,
                        'time': node['time'] + 1,
                        'priority': calculate_key(succ) }
                    print("calling update here2", child)
                    update_vertex(child)
                print("calling update here3", node)
                
                update_vertex(node)


  



    def main():
        initialize()
        while True:
            compute_shortest_path()
            # print(reconstruct_path())
            # Waitfor changes in edge costs; 
            # {21} forall directed edges (u,v) with changed edge costs 
            # {22} Update the edge cost c(u,v); {23} UpdateVertex(v);
            return 
    
    main()




        



def push_node(open_list, node, priority):
    # print("pushing node ", node)
    heapq.heappush(open_list, (priority[0],priority[1],node['loc'], node))
    # print("this is open " ,open_list)

# def push_node(open_list, node, priority):
#     loc, time = node["loc"], node["time"]
#     if (loc, time) in node_map:
#         existing_node = node_map[(loc, time)]
#         existing_priority = existing_node["priority"]

#         # Replace only if the new priority is better
#         if priority < existing_priority:
#             remove_node(open_list, existing_node, node_map)

#     # Add the node to the map and heap
#     node_map[(loc, time)] = node
#     node["priority"] = priority  # Update the node's priority
#     heapq.heappush(open_list, (priority[0], priority[1], time, node))
  

def pop_node(open_list):
    if(len(open_list) == 0):
        return
    _, _, _,curr = heapq.heappop(open_list)
    return curr


def top_key(open_list):
    print("this is top",open_list[0])
    print("open list", open_list)

    if len(open_list) == 0:
        return (float('inf'), float('inf'))
    return open_list[0]

def remove_node(open_list, node):
    rem = (float('-inf'), float('-inf'))
    update(open_list,node,rem)
    a = pop_node(open_list)

def update(open_list, node, priority):
    for index, (p1, p2, loc,item) in enumerate(open_list):
        if node == item:
                if (p1,p2) <= priority:
                    break
                del open_list[index]
                open_list.append((priority[0],priority[1],loc, item))
                heapq.heapify(open_list)
                break
    else:
            print("calling push2")
            push_node(open_list,node, priority)

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']