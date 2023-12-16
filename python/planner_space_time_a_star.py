import math
from typing import List, Tuple, Set
from queue import PriorityQueue

from python.models import Action, BasePlanner
from python.util import getManhattanDistance, get_neighbors
import numpy as np

from python.visualization_a_star import animate_combined, visualize_grid_with_lowest_g, visualize_explored_count, \
    animate_combined_v2


class SpaceTimeAStarPlanner(BasePlanner):
    reservation: Set[Tuple[int, int, int]] = set()
    # (cell id 1, cell id 2, timestep relative to current timestep [one_based])
    cell_hash_to_robot_id: dict[Tuple[int, int, int], int] = {}
    # (cell id, -1, timestep [one_based]): robot id
    next_actions = list[int]
    # next action for each robot
    time_step = 0

    # current time step

    def __init__(self, pyenv=None, visualize=False) -> None:
        super().__init__(pyenv, "Space-Time-A-Star-Planner")
        self.VISUALIZE = visualize
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

        if self.VISUALIZE:
            open_list_visualization_data = []  # Data for visualization
            explored_counts = {}
            explored_counts_list = [{}]
            grid_data = []
            grid_data_v2 = []
            current_frame_data_v2 = {}
            min_f_values_v2 = {}
            lowest_g_values = {}
            lowest_f_values = {}

        h = getManhattanDistance(self.env, start, end)  # heuristic approximation
        g = 0  # distance traveled
        node_info = (start, start_direct, g, h)
        open_list.put((g + h, id(node_info), node_info))
        # all_nodes[(start * 4 + start_direct, 0)] = s
        position_direction_hash = start * 4 + start_direct
        # why start * 4 + start_direct ?
        # because: this results in a unique hash of the postion/orientation (4 orientations -> if orientation changes: at least +1; if cell changes: at least +4)
        # this is a hash that is used to check if a position/orientation-combination was already looked at
        parent[(position_direction_hash, g)] = None  # safe the parent node

        while not open_list.empty():  # look at all cells in the open list
            if self.VISUALIZE:
                open_list_visualization_data.append([(n[2][0], n[2][2]) for n in open_list.queue])
                current_frame_data = {}
                current_f_values = {}
                current_f_values_v2 = {}

            node = open_list.get()  # get the node with the lowest f value
            h, node_id, current_node_info = node
            position, orientation, g, h = current_node_info
            current_time_step = g  # it is the same, when planning was started in time_step 0
            next_time_step = current_time_step + 1

            if self.VISUALIZE:
                explored_counts[position] = explored_counts.get(position, 0) + 1
                explored_counts_list.append(explored_counts.copy())
                # Update explored counts and lowest g values
                if position not in lowest_g_values or g < lowest_g_values[position]:
                    lowest_g_values[position] = g
                # Mark the current cell
                current_frame_data[position] = {'current': True}

            if (position * 4 + orientation, g) in all_nodes:
                continue  # skip if this node was already looked at - at the current time step
            all_nodes[(position * 4 + orientation, g)] = current_node_info
            if position == end:
                while True:  # yey, we found a path
                    path.append((current_node_info[0], current_node_info[1]))  # append position, orientation to path
                    current_node_info = parent[(current_node_info[0] * 4 + current_node_info[1], current_node_info[
                        2])]  # previous node is the parent -> get parent by position hash, g (dist from start)
                    if current_node_info is None:
                        break  # start node was reached which has no parent
                path.pop()  # remove the start node
                path.reverse()
                break

            neighbors = get_neighbors(self.env, position, orientation)
            neighbors.append((position, orientation))  # also check if we can wait on the current field
            for neighbor in neighbors:
                # it's not really the neighbor we are checking, it is more the next possible position+orientation
                neighbor_location, neighbor_direction = neighbor

                if self.is_reserved(position, neighbor_location, next_time_step):
                    continue

                neighbor_key = (neighbor_location * 4 + neighbor_direction, next_time_step)

                if neighbor_key in all_nodes:
                    old = all_nodes[neighbor_key]
                    if g + 1 < old[2]:  # the neighbor was already visited, but we found a shorter route
                        old = (old[0], old[1], g + 1, old[3], old[4])
                else:
                    next_g = g + 1
                    next_h = getManhattanDistance(self.env, neighbor_location, end)
                    next_node_info = (
                        neighbor_location,
                        neighbor_direction,
                        next_g,
                        next_h,
                    )
                    next_f = next_g + next_h
                    open_list.put(
                        (next_f, id(next_node_info), next_node_info)
                    )

                    parent[
                        (neighbor_location * 4 + neighbor_direction, next_g)
                    ] = current_node_info

            if self.VISUALIZE:
                current_f_values_v2[(position, orientation)] = [g+h]
                for node in open_list.queue:
                    f = node[0]
                    pos = node[2][0]
                    lowest_f = lowest_f_values.get(pos, math.inf)
                    if f < lowest_f:
                        lowest_f_values[pos] = f
                    current_lowest_f = current_f_values.get(pos, math.inf)
                    if f < current_lowest_f:
                        current_f_values[pos] = f
                    ori = node[2][1]
                    current_lowest_f = min_f_values_v2.get((pos, ori), math.inf)
                    if f < current_lowest_f:
                        min_f_values_v2[(pos, ori)] = f

                    l = current_f_values_v2.get((pos, ori))
                    if l is not None:
                        l.append(f)
                    else:
                        current_f_values_v2[(pos, ori)] = [f]

                for pos in range(0, self.env.cols * self.env.rows):
                    current_frame_data[pos] = {
                        'f_value': current_f_values.get(pos, np.inf),
                        'lowest_f_value': lowest_f_values.get(pos, np.inf),
                        'lowest_g_value': lowest_g_values.get(pos, np.inf),
                        'visit_count': explored_counts.get(pos, 0),
                        'current': current_frame_data.get(pos, {}).get('current', False)
                    }
                grid_data.append(current_frame_data)

                for pos in range(0, self.env.cols * self.env.rows):
                    orientations_data = []
                    for ori in range(4):  # Assuming 4 orientations
                        is_current = pos == position and ori == orientation
                        f_val = min_f_values_v2.get((pos, ori), np.inf)  # f_value for specific orientation

                        orientation_data = {
                            'min_f_value': f_val,
                            'f_value': current_f_values_v2.get((pos, ori), [math.inf]),
                            'current': is_current
                        }
                        orientations_data.append(orientation_data)

                    current_frame_data_v2[pos] = {'orientations': orientations_data}

                grid_data_v2.append(current_frame_data_v2.copy())
        if self.VISUALIZE:
            pass
            visualize_grid_with_lowest_g(open_list_visualization_data, self.env.map,
                                         grid_size=(self.env.cols, self.env.rows))
            visualize_explored_count(explored_counts, self.env.map, grid_size=(self.env.cols, self.env.rows))
            animate_combined_v2(grid_data_v2, self.env.map, interval=50, grid_size=(self.env.cols, self.env.rows),
                                filename=f"{self.env.map_name}_start_{start}_end_{end}.gif")
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
        # todo only do replanning each nth step
        # todo: stop when time_limit is reached?
        # todo: implement random restarts
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
