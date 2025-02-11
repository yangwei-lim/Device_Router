from Module.DB import *

def route_two_pins(grid, source: tuple, targets: list) -> list:
    """
    @brief      Routing two pins from single source to the nearest target.
    @param      source: The source node
    @param      target: The list of target nodes
    @return     path: The path from source to the nearest target
    @return     step: The step count of each node
    """
    # Wave propagation using BFS to get step count
    print("   >> Wave Propagation...", end="")
    target = bfs_multi_target(grid, source, targets)               # multi target breadth first search

    if target:
        print("Backtracking...", end="")
        path = dfs_backtrack(grid, source, target)                 # depth first search backtracking

        if path:
            print("Success. Path Length: {}".format(len(path)))
        else:
            path = None
            print("Failed.")
    else:
        # set path to None
        path = None
        print("Failed.")

    return path


def route_multi_pins(grid, pins: list) -> list:
    """
    @brief      Routing multiple pins using the method of multiple sources in routed path.
    @param      pins  The pins in list form
    @return     The path and step count of each node
    """
    # initialize variables
    paths = []

    # Phase 1: Get the first path
    targets = pins.copy()
    source = targets.pop(0)

    path = route_two_pins(grid, source, targets)  # route two pins

    if path:                                                # if path found
        # self.grid.addObstacle_coord(path)                 # mark path as obstacle
        paths.append(path)                                  # add path to list
        targets.remove(path[-1])                            # remove target from pins
    else:
        return None


    # Phase 2: Get the rest of the paths
    sources = targets.copy()                                # set the remain pins as sources
    targets = paths[0].copy()                               # set the first target as target 

    while sources:                                          # while there are sources     
        source = sources.pop(0)                             # set path nodes as source

        path = route_two_pins(grid, source, targets)     # route two pins
        if path:                                                # if path found
            # self.grid.addObstacle_coord(path)                 # mark path as obstacle
            paths.append(path)                                  # add path to list
            targets.extend(path)                                # add path to targets

        # not necessary but this is for reiteration of whole grid creation
        else:
            return None
            
    path_len = sum([len(path) for path in paths])
    print("Total Path Length: {}".format(path_len))

    return paths


def route_multi_pins_2(grid, pins: list) -> list:
    """
    @brief      Routing multiple pins using the method of multiple sources in routed path.
    @param      pins  The pins in list form
    @return     The path and step count of each node
    """
    # initialize variables
    paths = []

    # initialize source and target
    targets = pins.copy()
    source = targets.pop(0)

    # Phase 1: Get the first path
    path = route_two_pins(grid, source, [targets[0]])           # route two pins
    if path:                                                    # if path found
        # self.grid.addObstacle_coord(path)                       # mark path as obstacle
        paths.append(path)                                      # add path to list
        targets.remove(path[-1])                                # remove target from pins
        sources = path.copy()                                   # add path to sources
    else:
        return None

    # Phase 2: Get the rest of the paths based on heuristic
    while targets:                                              # while there are targets
        best_len = 99999999
        src_tar_idx = [0, 0]
        for i, source in enumerate(sources):
            for j, target in enumerate(targets):
                # get length
                # path_len = abs(target[0] - source[0]) + abs(target[1] - source[1]) + abs(target[2] - source[2])
                path_len = abs(target.x - source.x) + abs(target.y - source.y) + abs(target.z - source.z)   
                best_len = path_len if path_len < best_len else best_len
                src_tar_idx = [i, j] if path_len == best_len else src_tar_idx

        if sources[src_tar_idx[0]].x == targets[src_tar_idx[1]].x and sources[src_tar_idx[0]].y == targets[src_tar_idx[1]].y and sources[src_tar_idx[0]].z == targets[src_tar_idx[1]].z:
            targets.remove(sources[src_tar_idx[0]])
            continue

        path = route_two_pins(grid, sources[src_tar_idx[0]], [targets[src_tar_idx[1]]])  # route two pins
        if path:                                                # if path found
            paths.append(path)                                      # add path to list
            targets.remove(path[-1])                                # remove target from pins
            sources.extend(path)                                    # add path to sources
        else:
            print("Failed to route path. func: route_multi_pins_2")
            print("source: {}, target: {}".format((sources[src_tar_idx[0]].x, sources[src_tar_idx[0]].y, sources[src_tar_idx[0]].z), (targets[src_tar_idx[1]].x, targets[src_tar_idx[1]].y, targets[src_tar_idx[1]].z)))
            return None

    return paths
        

def route_multi_pins_group(grid, pins: list) -> list:
    """
    @brief      Routing multiple pins using the method of multiple sources in routed path.
    @param      pins  The pins in list form
    @return     The path and step count of each node
    """
    # initialize variables
    paths = []
    group = []

    # get the group of pins from the pin list
    for pin_list in pins:
        # if there is only one pin in the group
        if len(pin_list) == 1:
            group.append(pin_list)
            continue

        # in each group of pins, route the pins
        path = route_multi_pins_2(grid, pin_list)

        if path:
            # get each nodes from the path and make it a list
            node = [coor for seg in path for coor in seg]

            # add the list of nodes (unique) to the group
            group.append(list(set(node)))
        else:
            # need to reiterate the whole grid creation
            return None

    # 1st group of pins
    sources = group.pop(0) if group else None

    while group:
        # get the next group of pins
        targets = group.pop(0)

        best_len = 99999999
        src_tar_idx = [0, 0]
        # compare the nodes from the source and target group
        for i, source in enumerate(sources):
            for j, target in enumerate(targets):
                # get the best length from the different nodes
                path_len = abs(target.x - source.x) + abs(target.y - source.y) + abs(target.z - source.z)
                best_len = path_len if path_len < best_len else best_len
                src_tar_idx = [i, j] if path_len == best_len else src_tar_idx

        if sources[src_tar_idx[0]].x == targets[src_tar_idx[1]].x and sources[src_tar_idx[0]].y == targets[src_tar_idx[1]].y and sources[src_tar_idx[0]].z == targets[src_tar_idx[1]].z:
            targets.remove(sources[src_tar_idx[0]])
            continue

        # route two pins after get the best length
        path = route_two_pins(grid, sources[src_tar_idx[0]], [targets[src_tar_idx[1]]])  # route two pins
        if path:
            paths.append(path)                                      # add path to list
            sources.extend(path)                                    # add path to sources
        
        # not necessary but this is for reiteration of whole grid creation
        else:
            return None

    return paths
    

def bfs_multi_target(grid, source, targets: list):
    """
    @brief      Breath first search algorithm for step counting.
    """
    # initialization
    for lay in grid:
        for row in lay:
            for node in row:
                # initialize visited and step
                node.visited = False
                node.step = None

    # initialize queue
    queue = []
    queue.append(source) 

    # mark source as visited
    source.visited = True
    source.step = 0

    while queue:
        # dequeue
        curr_node = queue.pop(0)

        # add neighbors to queue    
        for neighbor in curr_node.get_neighbors():

            # if neighbor not visited
            if not neighbor.visited:
                # if neighbor is target (destination reached)
                if neighbor in targets:
                    neighbor.visited = True               # mark neighbor as visited
                    neighbor.step = curr_node.step + 1    # increment step count
                    return neighbor                       # exit function                                        
                
                # if neighbor is not an obstacle
                if not neighbor.obstacle:
                    neighbor.visited = True                                        # mark neighbor as visited
                    neighbor.step = curr_node.step + 1    # increment step count
                    queue.append(neighbor)   
                
    # all neighbors visited and no path found
    print(">> Wave Prop: No Path Found.") 
    return None


def dfs_backtrack(grid, source, target) -> list:
    """
    @brief      Depth first search algorithm for backtracking (Iterative method).
    @param      source:     The source node
    @param      target:     The target node
    @param      steps:      The step count of each node
    @return     path:       The path from source to target
    """
    # initialize visited and steps matrix
    for lay in grid:
        for row in lay:
            for node in row:
                # initialize visited
                node.visited = False

    # initialize stack
    stack = []
    stack.append(target)

    # mark target as visited
    target.visited = True
    path = []

    while stack:
        # pop
        curr_node = stack.pop()

        # mark current as visited
        curr_node.visited = True
        path.append(curr_node)

        # add neighbors to stack    
        for neighbor in curr_node.get_neighbors():
                
                # if neighbor not visited and neighbor has a step count
                if not neighbor.visited and neighbor.step is not None:
    
                    # if neighbor is source (destination reached)
                    if neighbor == source:
                        path.append(neighbor)                   # add to path
                        path.reverse()                          # reverse path
                        return path
                    
                    # if neighbor has one step count less than current node
                    if neighbor.step == curr_node.step - 1:
                        stack.append(neighbor)

    # all neighbors visited and no path found
    print(">> Backtrack: No Path Found.") 
    return None