from typing import List, Tuple, Set
from queue import PriorityQueue
from python.models import Env, Action, BasePlanner
from python.util import getManhattanDistance, get_neighbors


class SpaceTimeAStarPlanner(BasePlanner):
    reservation: Set[Tuple[int, int, int]] = set()
    # (cell id 1, cell id 2, timestep relative to current timestep [one_based])
    cell_hash_to_robot_id: dict[Tuple[int, int, int], int] = {}
    # (cell id, -1, timestep [one_based]): robot id
    next_actions = list[int]
    # next action for each robot
    time_step = 0
    # current time step

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
    ) -> List[Tuple[int, int]]:
        """
        finds the shortest path
        :param start: the start cell index
        :param start_direct: the orientation of the robot
        :param end: the target cell index
        :return: the shortest path if it exists - list of (node index, orientation) tuples
        """
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
            _, _, curr = n
            current_time_step = curr[2]
            next_time_step = current_time_step + 1

            curr_location, curr_direction, curr_g, _ = curr

            if (curr_location * 4 + curr_direction, curr_g) in all_nodes:
                continue  # skip if this node was already looked at
            all_nodes[(curr_location * 4 + curr_direction, curr_g)] = curr
            if curr_location == end:
                while True:
                    path.append((curr[0], curr[1]))  # append position, orientation to path
                    curr = parent[(curr[0] * 4 + curr[1], curr[2])]  # previous node is the parent -> get parent by position hash, g (dist from start)
                    if curr is None:
                        break  # start node was reached
                path.pop()
                path.reverse()
                break

            neighbors = get_neighbors(self.env, curr_location, curr_direction)

            for neighbor in neighbors:
                # it's not really the neighbor we are checking, it is more the next possible position+orientation
                neighbor_location, neighbor_direction = neighbor

                if self.is_reserved(curr_location, neighbor_location, next_time_step):
                    continue

                neighbor_key = (neighbor_location * 4 + neighbor_direction, next_time_step)

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

        return path

    def is_reserved(self, start: int, end: int, time_step: int):
        """
        check if the target cell is already reserved + check if the edge is reserved
        :return: true if move is already reserved
        """
        if end == -1:
            end = start
        if (end, -1, time_step) in self.reservation:
            return True  # the end cell is already reserved

        if (end, start, time_step) in self.reservation:
            return True  # the edge end --to--> start is already reserved in the next timestep
        return False


    def sample_priority_planner(self, time_limit: int):
        self.reservation = set()
        self.next_actions = [Action.W.value] * len(self.env.curr_states)

        # reserve waiting cell for all robots that don't have any goals left
        for robot_id in range(self.env.num_of_agents):
            path = []
            if not self.env.goal_locations[robot_id]:
                path.append(
                    (
                        self.env.curr_states[robot_id].location,
                        self.env.curr_states[robot_id].orientation,
                    )
                )
                self.add_reservation(self.env.curr_states[robot_id].location, -1, 1, robot_id)

        # plan and reserve path for one robot at a time
        for robot_id in range(self.env.num_of_agents):
            path = []
            if self.env.goal_locations[robot_id]:
                path = self.space_time_plan(  # get the shortest possible path
                    self.env.curr_states[robot_id].location,
                    self.env.curr_states[robot_id].orientation,
                    self.env.goal_locations[robot_id][0][0]
                )

            last_loc = self.env.curr_states[robot_id].location
            if path:
                if path[0][0] != self.env.curr_states[robot_id].location:
                    self.next_actions[robot_id] = Action.FW.value
                elif path[0][1] != self.env.curr_states[robot_id].orientation:
                    incr = path[0][1] - self.env.curr_states[robot_id].orientation
                    if incr == 1 or incr == -3:
                        self.next_actions[robot_id] = Action.CR.value
                    elif incr == -1 or incr == 3:
                        self.next_actions[robot_id] = Action.CCR.value

                time_step = 1
                for p in path:
                    self.add_reservation(last_loc, p[0], time_step, robot_id)
                    last_loc = p[0]
                    time_step += 1
            if not path:
                # there is no path for robot i -> he will wait -> reserve his waiting position BUT:
                # it is possible that the waiting cell is already reserved -> the robot that reserved the cell has to be stopped
                # to prevent a crash
                waiting_position = (last_loc, -1, 1)
                if self.is_reserved(*waiting_position):
                    # check who reserved it and cancel his actions
                    self.handle_conflict(*waiting_position)
                else:
                    self.add_reservation(*waiting_position, robot_id)

        return self.next_actions

    def add_reservation(self, start: int, end: int, time_step: int, robot_index: int):
        """
        add a path to the reservation table
        :param start: start cell index
        :param end: end cell index: -1 if same as start
        :param time_step: reservation timestep
        :param robot_index: id of the reserving robot
        """
        if end == -1:
            end = start
        cell_hash = (end, -1, time_step)
        self.reservation.add(cell_hash)  # reserve the end cell itself
        if start != end:
            edge_hash = (start, end, time_step)
            self.reservation.add(edge_hash)  # reserve the edge
        self.cell_hash_to_robot_id[cell_hash] = robot_index  # to make it easy to lookup which robot reserved which cell

    def handle_conflict(self, start: int, end: int, time_step: int):
        # todo: revoke all the reservations of the robot that reserved (start, end, time_step)
        # todo: check if there is an easy & quick reroute of the colliding robot possible
        colliding_robot_id = self.cell_hash_to_robot_id[(start, end, time_step)]
        self.next_actions[colliding_robot_id] = Action.W.value  # make colliding robot wait
        # if the colliding robot which will now wait would collide with another robot -> stop the other robot also
        stopped_robot_location = self.env.curr_states[colliding_robot_id].location
        wait_cell_hash_of_stopped_robot = (stopped_robot_location, -1, time_step)
        if self.is_reserved(*wait_cell_hash_of_stopped_robot):
            self.handle_conflict(*wait_cell_hash_of_stopped_robot)
        self.add_reservation(*wait_cell_hash_of_stopped_robot, colliding_robot_id)
