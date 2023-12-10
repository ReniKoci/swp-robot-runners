from queue import PriorityQueue
from python.models import Env, Action


def single_agent_plan(env, start: int, start_direct: int, end: int):
    print(start, start_direct, end)
    path = []
    # AStarNode (u,dir,t,f)
    open_list = PriorityQueue()
    s = (start, start_direct, 0, getManhattanDistance(env, start, end))
    open_list.put([0, s])
    all_nodes = dict()
    close_list = set()
    parent = {(start, start_direct): None}
    all_nodes[start * 4 + start_direct] = s
    while not open_list.empty():
        curr = (open_list.get())[1]
        close_list.add(curr[0] * 4 + curr[1])
        if curr[0] == end:
            curr = (curr[0], curr[1])
            while curr != None:
                path.append(curr)
                curr = parent[curr]
            path.pop()
            path.reverse()

            break
        neighbors = get_neighbors(env, curr[0], curr[1])
        # print("neighbors=",neighbors)
        for neighbor in neighbors:
            if (neighbor[0] * 4 + neighbor[1]) in close_list:
                continue
            next_node = (
                neighbor[0],
                neighbor[1],
                curr[2] + 1,
                getManhattanDistance(env, neighbor[0], end),
            )
            parent[(next_node[0], next_node[1])] = (curr[0], curr[1])
            open_list.put([next_node[3] + next_node[2], next_node])
    print(path)
    return path


def getManhattanDistance(env, loc1: int, loc2: int) -> int:
    loc1_x = loc1 // env.cols
    loc1_y = loc1 % env.cols
    loc2_x = loc2 // env.cols
    loc2_y = loc2 % env.cols
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y)


def validateMove(env, loc: int, loc2: int) -> bool:
    loc_x = loc // env.cols
    loc_y = loc % env.cols
    if loc_x >= env.rows or loc_y >= env.cols or env.map[loc] == 1:
        return False  # loc is not on map or loc is an obstacle
    loc2_x = loc2 // env.cols
    loc2_y = loc2 % env.cols
    if abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1:
        return False  # would move more than 1 cell
    return True


def get_neighbors(env: Env, location: int, direction: int) -> list[tuple[int, int]]:
    """
    returns all possible position-orientation combinations that can be reached after one time step (move forward, turn left, turn right)
    :param env: the env obj
    :param location: the current node index
    :param direction: the current orientation
    :return: list of two or three tuples[node index, orientation]
    """
    neighbors = []
    # forward
    candidates = [
        location + 1,
        location + env.cols,
        location - 1,
        location - env.cols,
    ]
    forward = candidates[direction]
    new_direction = direction
    is_valid_move = forward >= 0 and forward < len(env.map) and validateMove(env, forward, location)
    if is_valid_move:
        neighbors.append((forward, new_direction))
    # when we just turn left or right we don't have to validate because we do not change nodes
    # turn left
    new_direction = direction - 1
    if new_direction == -1:
        new_direction = 3
    neighbors.append((location, new_direction))
    # turn right
    new_direction = direction + 1
    if new_direction == 4:
        new_direction = 0
    neighbors.append((location, new_direction))
    # print("debug!!!!!!!", neighbors)
    return neighbors


def naive_a_star(env: Env, time_limit):
    print("I am planning")
    actions = [Action.W for i in range(len(env.curr_states))]
    for i in range(0, env.num_of_agents):
        print("python start plan for agent ", i, end=" ")
        path = []
        if len(env.goal_locations[i]) == 0:
            print(i, " does not have any goal left", end=" ")
            path.append(
                (
                    env.curr_states[i].location,
                    env.curr_states[i].orientation,
                )
            )
        else:
            print(" with start and goal: ", end=" ")
            path = single_agent_plan(
                env,
                env.curr_states[i].location,
                env.curr_states[i].orientation,
                env.goal_locations[i][0][0],
            )

        print("current location:", path[0][0], "current direction: ", path[0][1])
        if path[0][0] != env.curr_states[i].location:
            actions[i] = Action.FW
        elif path[0][1] != env.curr_states[i].orientation:
            incr = path[0][1] - env.curr_states[i].orientation
            if incr == 1 or incr == -3:
                actions[i] = Action.CR
            elif incr == -1 or incr == 3:
                actions[i] = Action.CCR
    # print(actions)
    actions = [int(a) for a in actions]
    # print(actions)
    return actions  # np.array(actions, dtype=int)
