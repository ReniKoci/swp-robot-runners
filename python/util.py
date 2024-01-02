from typing import Optional
from queue import PriorityQueue

from python.models import Env, Action


class DistanceMap:
    """
    A class that can be used to calculate the shortest distance from a given cell to a target cell.
    It uses the A* algorithm to calculate the distance.
    The distances are saved. If a distance is requested for a cell that hasn't been visited yet, the distance is
    calculated efficiently by using the previous distances.
    """
    def __init__(self, target: int, env: Env):
        number_of_cells = len(env.map)
        self.target: int = target
        self.open_list: PriorityQueue[
            tuple[int, int, int, int, int]] = PriorityQueue()  # f, h, g, position, orientation
        self.close_list: set = set()
        self.distance_map: list[Optional[int]] = [None] * (number_of_cells * 4)
        self.last_start = None

    def get_distance(self, env: Env, start_cell: int, start_orientation: int) -> int:
        """
        Returns the shortest distance from start_cell to the target cell.
        Only calculates the distance if it hasn't been calculated before.
        :param env:
        :param start_cell:
        :param start_orientation:
        :return:
        """
        if self.open_list.empty() and self.close_list:
            raise RuntimeError(f"no valid path found from {start_cell} to {self.target}")

        # add target as starting point if this is the first call
        if self.open_list.empty():
            for orientation in range(4):
                g = 0
                h = getManhattanDistance(env, self.target, start_cell)
                f = g + h
                self.open_list.put((f, h, g, self.target, orientation))
                self.distance_map[self.target * 4 + orientation] = 0

        if (dist := self.distance_map[start_cell * 4 + start_orientation]) is not None:
            return dist

        if self.last_start != start_cell:
            # update all h values in open_list
            new_open_list = PriorityQueue()
            while not self.open_list.empty():
                f, h, g, position, orientation = self.open_list.get()
                h = getManhattanDistance(env, position, start_cell)
                f = g + h
                new_open_list.put((f, h, g, position, orientation))
            self.open_list = new_open_list
            self.last_start = start_cell

        # do backwards A* to get distance (target -> start_cell)
        target = start_cell
        target_orientation = start_orientation
        while not self.open_list.empty():
            f, h, g, position, orientation = (self.open_list.get())
            self.close_list.add(position * 4 + orientation)
            neighbors = get_neighbors(env, position, orientation, reverse=True)
            for neighbor in neighbors:
                neighbor_position, neighbor_orientation = neighbor
                if (neighbor_position * 4 + neighbor_orientation) in self.close_list:
                    continue
                neighbor_g = g + 1
                neighbor_h = getManhattanDistance(env, neighbor_position, target)
                neighbor_f = neighbor_g + neighbor_h
                self.open_list.put((neighbor_f, neighbor_h, neighbor_g, neighbor_position, neighbor_orientation))

                pos_ori_hash = neighbor_position * 4 + neighbor_orientation
                shortest_distance = self.distance_map[pos_ori_hash]
                if shortest_distance is None or neighbor_g < shortest_distance:
                    self.distance_map[pos_ori_hash] = neighbor_g
                if neighbor_position == target and neighbor_orientation == target_orientation:
                    return neighbor_g
        raise RuntimeError(f"no valid path found from {start_cell} to {self.target}")


def a_star(env, start: int, start_direct: int, end: int):
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
        neighbors = get_neighbors(env, curr[0], curr[1], reverse=True)
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


def get_neighbors(env: Env, location: int, direction: int, reverse=False) -> list[tuple[int, int]]:
    """
    returns all possible position-orientation combinations that can be reached after one time step (move forward, turn left, turn right)
    :param reverse: set to True to get the neighbors for backwards A*
    :param env: the env obj
    :param location: the current node index
    :param direction: the current orientation
    :return: list of two or three tuples[node index, orientation]
    """
    neighbors = []
    if reverse:
        # backwards
        candidates = [
            location - 1,
            location - env.cols,
            location + 1,
            location + env.cols,
        ]
    else:
        # forwards
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
    actions = [Action.W for i in range(len(env.curr_states))]
    for i in range(0, env.num_of_agents):
        path = []
        if len(env.goal_locations[i]) == 0:
            path.append(
                (
                    env.curr_states[i].location,
                    env.curr_states[i].orientation,
                )
            )
        else:
            path = a_star(
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


def convert_1d_to_2d_coordinate(location: int, cols: int) -> tuple[int, int]:
    y = location // cols
    x = location % cols
    return x, y


def convert_2d_to_1d_coordinate(coordinate: tuple[int, int], cols: int) -> int:
    x, y = coordinate
    return y * cols + x
