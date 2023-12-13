from typing import List, Tuple, Set
from queue import PriorityQueue
from python.models import Env, Action, BasePlanner
from python.util import getManhattanDistance, get_neighbors


class SpaceTimeAStarPlanner(BasePlanner):
    def __init__(self, pyenv=None) -> None:
        super().__init__(pyenv, "Space-Time-A-Star-Planner")
        print(f"{self.name} created")

    def initialize(self, preprocess_time_limit: int):
        return True  # todo: implement

    def plan(self, time_limit) -> list[int]:
        return self.sample_priority_planner(time_limit)

    def space_time_plan(
            self,
            start: int,
            start_direct: int,
            end: int,
            reservation: Set[Tuple[int, int, int]],
    ) -> List[Tuple[int, int]]:
        """
        finds the shortest path
        :param start: the start cell index
        :param start_direct: the orientation of the robot
        :param end: the target cell index
        :param reservation: the reservation table
        :return: the shortest path if it exists - list of (node index, orientation) tuples
        """
        print(start, start_direct, end)
        path = []
        open_list = PriorityQueue()  # list of all cells to look at
        all_nodes = {}  # loc+dict, t
        parent = {}
        s = (start, start_direct, 0,
             getManhattanDistance(self.env, start, end))  # node info: start, orientation, g or time step?, f
        open_list.put((s[3], id(s), s))  # heuristic value, unique id, node info
        # all_nodes[(start * 4 + start_direct, 0)] = s
        parent[(start * 4 + start_direct, 0)] = None  # safe the parent node; key: position hash, distance from start
        # why start * 4 + start_direct ?
        # because: this results in a unique hash of the postion/orientation (4 orientations -> if orientation changes: at least +1; if cell changes: at least +4)
        # this is a hash that is used to check if a position/orientation-combination was already looked at

        while not open_list.empty():  # look at all cells in the open list
            n = open_list.get()  # get the node with the lowest f value
            # print("n=",n)
            _, _, curr = n

            curr_location, curr_direction, curr_g, _ = curr

            if (curr_location * 4 + curr_direction, curr_g) in all_nodes:
                continue  # skip if this node was already looked at
            all_nodes[(curr_location * 4 + curr_direction, curr_g)] = curr
            if curr_location == end:
                while True:
                    path.append((curr[0], curr[1]))  # append position, orientation to path
                    curr = parent[(curr[0] * 4 + curr[1], curr[
                        2])]  # previous node is the parent -> get parent by position hash, g (dist from start)
                    if curr is None:
                        break  # start was reached
                    # curr = curr[5]
                path.pop()
                path.reverse()
                break

            neighbors = get_neighbors(self.env, curr_location, curr_direction)

            for neighbor in neighbors:
                # it's not really the neighbor we are checking, it is more the next possible position+orientation
                neighbor_location, neighbor_direction = neighbor

                if (neighbor_location, -1, curr[2] + 1) in reservation:
                    continue  # the edge neighbor_location --to--> neighbor_location is already reserved in the next timestep

                if (neighbor_location, curr_location, curr[2] + 1) in reservation:
                    continue  # the edge neighbor_location --to--> current_location is already reserved in the next timestep
                # todo: shouldn't we also check if the neighbor location is reserved at all (also from different directions)

                neighbor_key = (neighbor_location * 4 + neighbor_direction, curr[2] + 1)

                if neighbor_key in all_nodes:
                    old = all_nodes[neighbor_key]
                    if curr_g + 1 < old[2]:  # the neighbor was already visited, but we found a shorter route
                        old = (old[0], old[1], curr_g + 1, old[3], old[4])  # todo: old is not updated correctly!?
                else:
                    next_node = (
                        neighbor_location,
                        neighbor_direction,
                        curr_g + 1,
                        getManhattanDistance(self.env, neighbor_location, end),
                    )

                    open_list.put(
                        (next_node[3] + next_node[2], id(next_node), next_node)
                    )

                    parent[
                        (neighbor_location * 4 + neighbor_direction, next_node[2])
                    ] = curr

        for v in path:
            print(f"({v[0]},{v[1]}), ", end="")
        print()
        return path

    def sample_priority_planner(self, time_limit: int):
        actions = [Action.W.value] * len(self.env.curr_states)
        reservation = set()  # loc1, loc2, t

        for i in range(self.env.num_of_agents):
            print("start plan for agent", i)
            path = []
            if not self.env.goal_locations[i]:
                print(", which does not have any goal left.")
                path.append(
                    (
                        self.env.curr_states[i].location,
                        self.env.curr_states[i].orientation,
                    )
                )
                reservation.add((self.env.curr_states[i].location, -1, 1))

        for i in range(self.env.num_of_agents):
            print("start plan for agent", i)
            path = []
            if self.env.goal_locations[i]:
                print("with start and goal:")
                path = self.space_time_plan(  # get the shortest possible path
                    self.env.curr_states[i].location,
                    self.env.curr_states[i].orientation,
                    self.env.goal_locations[i][0][0],
                    reservation,
                )

            if path:
                print("current location:", path[0][0], "current direction:", path[0][1])
                if path[0][0] != self.env.curr_states[i].location:
                    actions[i] = Action.FW.value
                elif path[0][1] != self.env.curr_states[i].orientation:
                    incr = path[0][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = Action.CR.value
                    elif incr == -1 or incr == 3:
                        actions[i] = Action.CCR.value

                last_loc = -1
                t = 1
                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                    last_loc = p[0]
                    t += 1

        return actions
