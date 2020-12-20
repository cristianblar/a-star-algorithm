from queue import PriorityQueue


def shortest_path(M,start,goal):

    if start not in M.intersections or goal not in M.intersections:
        return None
    
    if start == goal:
        return [goal]
    
    
        
    def calculate_distance(intersection_1, intersection_2):  # Euclidean distance for g distances
        distance = (((M.intersections[intersection_2][0] - M.intersections[intersection_1][0]) ** 2 +
                    (M.intersections[intersection_2][1] - M.intersections[intersection_1][1]) ** 2) ** 0.5)
        
        return distance
    

    def h_calculator(M, goal):  # Returns a list with index = intersection number, the list contains heuristic distance (h)
        h_distances = [0 for _ in range(len(M.intersections))]
        
        for intersection in M.intersections.keys():
            h_distances[intersection] = calculate_distance(intersection, goal)
        
        return h_distances
    
        
    h_distances = h_calculator(M, goal)
    
    g_distances = {}
    
    g_distances[start] = 0
    
    previous_reference = {}  # We will use this to build the result_path
    
    previous_reference[start] = None

    frontier = PriorityQueue()
    
    frontier.put((0, start))  # (f, intersection number)
    
    found_flag = False
    
        
    while not frontier.empty():
        
        current_intersection = frontier.get()[1]  # VISITING next node
        
        if current_intersection == goal:  # Stop if the visited node is the target node
            found_flag = True
            break
        
        for connected_intersection in M.roads[current_intersection]:
            
            # g:
            path_cost = calculate_distance(current_intersection, connected_intersection) + g_distances[current_intersection]
            
            # Cheapest way first...
            if connected_intersection not in g_distances or path_cost < g_distances[connected_intersection]:
                g_distances[connected_intersection] = path_cost
                previous_reference[connected_intersection] = current_intersection
                # Then, heuristic way is considered...
                f = g_distances[connected_intersection] + h_distances[connected_intersection]  # f = g + h
                frontier.put((f, connected_intersection))

                
    if not found_flag:  # This will return function if goal node is unreachable from the start node
        return None

    # Building the result path:
    
    result_path = [goal]
    
    previous_intersection = previous_reference[goal]
    
    while previous_intersection is not None:
        result_path.insert(0, previous_intersection)
        previous_intersection = previous_reference[previous_intersection]
    
    
    return result_path