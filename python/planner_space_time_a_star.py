import math
import os
import random
import time
from copy import copy
from typing import Tuple, Set, Optional
from queue import PriorityQueue
from python.models import Action, BasePlanner, Heuristic
from python.util import getManhattanDistance, get_neighbors, DistanceMap, get_valid_forwards_neighbor_cell, \
    get_robot_position_map


class SpaceTimeAStarPlanner(BasePlanner):
    debug_mode = True
    verbose = False

    def __init__(self, pyenv=None, visualize=False, animate=False, replanning_period=8, time_horizon=10, restarts=False,
                 heuristic: Heuristic = Heuristic.TRUE_DISTANCE) -> None:
        super().__init__(pyenv, "Space-Time-A-Star-Planner")
        self.reservation: Set[Tuple[int, int, int]] = set()
        # (cell id 1, cell id 2, timestep relative to current timestep [one_based])
        self.edge_hash_to_robot_id: dict[Tuple[int, int, int], int] = {}
        # (cell id, -1, timestep [one_based]): robot id
        self.next_actions: list[list[int]]
        # next action for each robot
        self.last_planning_step = -math.inf
        self.distance_maps = {}  # in here we store the distance map for each target cell while ignoring robots

        # load values from environment variables if present
        self.replanning_period = replanning_period
        if r := os.getenv("replanning_period"):
            self.replanning_period = int(r)
        self.time_horizon = time_horizon
        if t := os.getenv("time_horizon"):
            self.time_horizon = int(t)
        self.random_restarts = restarts
        if r := os.getenv("restarts"):
            self.random_restarts = r == "True"
        self.heuristic = heuristic
        if h := os.getenv("heuristic"):
            self.heuristic = h
        self.try_fix_waiting_robots = True
        if t := os.getenv("try_fix_waiting_robots"):
            self.try_fix_waiting_robots = t == "True"

        self.VISUALIZE = visualize
        if visualize:
            from python.visualization_a_star import AStarVisualizer
            self.visualizer = AStarVisualizer()
            self.visualizer.GENERATE_ANIMATIONS = animate
        random.seed(42)

    def initialize(self, preprocess_time_limit: int):
        # todo use initialization time to autotune parameters (replanning_period, time_horizon, try_fix_waiting_robots,
        #  parallelization, ...)
        # todo preprocess distance maps
        return True  # todo: implement preprocessing or optimal pathfinding

    def plan(self, time_limit) -> list[int]:
        if self.last_planning_step + self.replanning_period <= self.env.curr_timestep:
            self.last_planning_step = self.env.curr_timestep
            return self.plan_with_random_restarts(time_limit)
        else:
            return self.next_actions[self.env.curr_timestep - self.last_planning_step]

    def plan_with_random_restarts(self, time_limit: Optional[int] = None, time_buffer=0) -> list[int]:
        start = time.time()
        num_of_robots = self.env.num_of_agents
        priority_order = tuple(range(num_of_robots))
        try:
            first_solution, path_length_sum, waiting_robots_count, waiting_robot_ids = self.sample_priority_planner(
                time_limit,
                priority_order)
        except RuntimeError as e:
            print(e)
        if not self.random_restarts:
            return first_solution

        tried_priority_orders = {priority_order}
        min_waiting_robots = waiting_robots_count
        min_path_length_sum = path_length_sum
        best_next_actions = copy(self.next_actions)
        got_best_actions_through_fix_step = False
        best_iteration = 0
        # todo: maybe prioritize
        #  - the robots that had collisions in the last planning step
        #  - the robots that are closest to their goal
        number_of_restarts = 10
        number_of_restarts = min(number_of_restarts, math.factorial(num_of_robots) - 1)
        iteration_count = 0
        # todo: parallelize
        last_step_was_fix_step = False
        while True:
            iteration_count += 1
            if self.verbose:
                print(f"iteration {iteration_count}___________________________________________________________")
            if time_limit:
                if (time.time() - start) + time_buffer >= time_limit:
                    break
            if not time_limit:
                if iteration_count > number_of_restarts:
                    break
            try_to_fix_waiting_robots = self.try_fix_waiting_robots and not last_step_was_fix_step
            if try_to_fix_waiting_robots:
                _, new_path_length_sum, waiting_robots_count, waiting_robot_ids = self.sample_priority_planner(
                    time_limit,
                    priority_order,
                    try_fix_stuck_robots=waiting_robot_ids)
                last_step_was_fix_step = True  # makes sure fix is only tried once per priority order
            else:
                last_step_was_fix_step = False
                while True:
                    new_priority_order = list(priority_order)
                    random.shuffle(new_priority_order)
                    new_priority_order = tuple(new_priority_order)
                    if new_priority_order not in tried_priority_orders:
                        priority_order = new_priority_order
                        tried_priority_orders.add(priority_order)
                        break
                _, new_path_length_sum, waiting_robots_count, waiting_robot_ids = self.sample_priority_planner(
                    time_limit,
                    priority_order)
            if waiting_robots_count < min_waiting_robots or (
                    waiting_robots_count == min_waiting_robots and new_path_length_sum < min_path_length_sum):
                min_waiting_robots = waiting_robots_count
                min_path_length_sum = new_path_length_sum
                best_next_actions = copy(self.next_actions)
                got_best_actions_through_fix_step = last_step_was_fix_step
                if self.verbose:
                    best_iteration = iteration_count
        self.next_actions = best_next_actions
        if self.debug_mode:
            print(f"iteration count: {iteration_count}")
            print(f"best actions through fix step: {got_best_actions_through_fix_step}")
            if self.verbose:
                print(f"best iteration: {best_iteration}")
        return best_next_actions[0]
        # todo how to determine the best solution? ideas:
        #  lowest sum of path lengths, lowest number of waiting robots, lowest sum of h values
        #  (caution: when an agent reaches its goal, he will get a new target with a new bigger h)

    def get_heuristic_value(self, start: int, orientation: int, end: int) -> int:
        """
        get the heuristic value for the given start and end cell
        :param orientation: orientation of the robot
        :param start: start cell index
        :param end: end cell index
        :return: the heuristic value
        """
        if self.heuristic == Heuristic.TRUE_DISTANCE:
            return self.get_true_distance(start, orientation, end)
        elif self.heuristic == Heuristic.MANHATTAN:
            return getManhattanDistance(self.env, start, end)
        else:
            raise RuntimeError(f"unknown heuristic {self.heuristic}")

    def get_true_distance(self, start: int, start_orientation: int, end: int) -> int:
        """
        get the true distance between two cells
        :param start_orientation: orientation of the robot
        :param start: start cell index
        :param end: end cell index
        :return: the true distance
        """
        if start == end:
            return 0
        if end in self.distance_maps:
            distance_map = self.distance_maps[end]
        else:
            distance_map = DistanceMap(end, self.env)
            self.distance_maps[end] = distance_map
        return distance_map.get_distance(self.env, start, start_orientation)

    def space_time_plan(
            self,
            start: int,
            start_direct: int,
            end: int,
            robot_id: int
    ) -> list[tuple[int, int]]:
        """
        finds the shortest path
        :param robot_id: id of the robot
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
            self.visualizer.reset()

        h = self.get_heuristic_value(start, start_direct, end)  # heuristic approximation
        g = 0  # distance traveled
        node_info = (start, start_direct, g, h)
        open_list.put((g + h, h, 0, node_info))
        position_direction_hash = start * 4 + start_direct
        # why start * 4 + start_direct ?
        # because: this results in a unique hash of the postion/orientation (4 orientations -> if orientation changes: at least +1 or +3 at most; if cell changes: at least +4)
        # this is a hash that is used to check if a position/orientation-combination was already looked at
        parent[(position_direction_hash, g)] = None  # safe the parent node

        while not open_list.empty():  # look at all cells in the open list
            if self.VISUALIZE:
                self.visualizer.commit_open_list([(n[3][0], n[3][2]) for n in open_list.queue])
                self.visualizer.new_step()

            node = open_list.get()  # get the node with the lowest f value
            f, h, node_id, current_node_info = node
            position, orientation, g, h = current_node_info
            current_time_step = g  # it is the same, when planning was started in time_step 0
            next_time_step = current_time_step + 1

            if (position * 4 + orientation, g) in all_nodes:
                continue  # skip if this node was already looked at - at the current time step
            all_nodes[(position * 4 + orientation, g)] = current_node_info
            if position == end:
                while True:  # yey, we found a path
                    # todo: instead of returning the path here, we should first check if the path is at least as long as
                    #  the time horizon, otherwise the robot would stop and block the cell it is on until the
                    #  next planning step
                    #  (or worse: the waiting cell is already reserved -> the found route would get canceled))
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

                if self.is_reserved(position, neighbor_location, next_time_step, current_robot_id=robot_id):
                    continue

                neighbor_key = (neighbor_location * 4 + neighbor_direction, next_time_step)

                if neighbor_key in all_nodes:
                    old = all_nodes[neighbor_key]
                    if g + 1 < old[2]:  # todo what exactly is this good for?
                        old = (old[0], old[1], g + 1, old[3], old[4])
                else:
                    next_g = g + 1
                    next_h = self.get_heuristic_value(neighbor_location, neighbor_direction, end)
                    next_node_info = (
                        neighbor_location,
                        neighbor_direction,
                        next_g,
                        next_h,
                    )
                    next_f = next_g + next_h
                    open_list.put(
                        (next_f, next_h, 0, next_node_info)
                    )

                    parent[
                        (neighbor_location * 4 + neighbor_direction, next_g)
                    ] = current_node_info

            if self.VISUALIZE:
                self.visualizer.update_data(self.env, open_list, position, orientation, g)
        if self.VISUALIZE:
            self.visualizer.save_visualizations(self.env, start, end)
        return path

    def is_reserved(self, start: int, end: int, time_step: int, current_robot_id=None) -> bool:
        """
        check if the target cell is already reserved + check if the edge is reserved
        :return: true if move is already reserved
        """
        if end == -1:
            end = start
        if (end, -1, time_step) in self.reservation:
            if current_robot_id is None:
                return True  # the end cell is already reserved
            # only return True if the robot that reserved the cell is not the current robot
            if self.edge_hash_to_robot_id[(end, -1, time_step)] != current_robot_id:
                return True

        if (end, start, time_step) in self.reservation:
            return True  # the edge end --to--> start is already reserved in the next timestep
        return False

    def sample_priority_planner(self, time_limit: int, robot_order=None,
                                try_fix_stuck_robots: Optional[list[int]] = None) -> tuple[
        list[int], int, int, list[int]]:
        """
        get actions for all robots
        :param try_fix_stuck_robots:
        :param time_limit:
        :param robot_order: order in which the robots should be planned (priority)
        :return: actions, path_length_sum, number of waiting_robots,
        collision_groups (list of lists of robot ids that are in a collision group)
        """
        # todo: stop when time_limit is reached?
        # todo: replan (or only plan for specific agents) when some agent reached his goal
        self.reservation = set()
        self.edge_hash_to_robot_id = {}

        # todo implement try_fix_collisions: make the first collider in the group wait one step and make the robot wait the first step he would normally move forward
        #  and then make his moves planned in the previous iteration,
        #  check if this is possible and if the other robot can now move

        path_length_sum = 0
        waiting_robots = 0
        waiting_robot_ids = []
        collision_groups: list[list[int]] = []  # list of lists of robot ids that are in a collision group

        self.next_actions = [[Action.W.value] * len(self.env.curr_states) for _ in range(self.replanning_period)]

        self.prereserve_cells_based_on_robot_positions(try_fix_stuck_robots)

        # plan and reserve path for one robot at a time
        for robot_id in robot_order or range(self.env.num_of_agents):
            # reserve waiting cell for all robots that don't have any goals left
            # todo: instead of waiting,
            #  try to find a path that avoids collisions by setting the robots priority to the lowest
            #  and / or perform depth first search to find a path that avoids collisions
            if not self.env.goal_locations[robot_id]:
                for step in range(self.time_horizon):
                    self.add_reservation(self.env.curr_states[robot_id].location, -1, step, robot_id)
                continue

            path = self.space_time_plan(  # get the shortest possible path
                self.env.curr_states[robot_id].location,
                self.env.curr_states[robot_id].orientation,
                self.env.goal_locations[robot_id][0][0],
                robot_id
            )

            last_loc = self.env.curr_states[robot_id].location
            if path:
                if self.verbose:
                    print(f"{robot_id:03} found a path")
                try:
                    self.reserve_path_if_possible(last_loc, path, robot_id)
                    self.update_next_actions(path, robot_id)
                    path_length_sum += len(path)  # todo decrease sum when a path gets cancelled
                    if self.verbose:
                        print(f"{robot_id:03} reserved path")
                except RuntimeError:
                    # the path could not be reserved (e.g. because the path was shorter than the time horizon
                    # -> the robot would have to wait and the waiting cell is already reserved in some time step)
                    if self.verbose:
                        print(f"{robot_id:03} reserve path error")
                    path = None
            if not path:
                if self.verbose:
                    print(f"{robot_id:03} no path")
                waiting_robots += 1
                waiting_robot_ids.append(robot_id)
                # todo - idea: collect all waiting_robot_ids (not just of the first stopped robots); re-add them to the
                #  queue and see if they can find a path later when paths of other robots were potentially canceled
                # todo - idea: prioritize robots closest to goal
                # todo - idea: make robots in collision group (or all) plan routes as if all other robots would stay
                #  on their current position (delete "waiting"-reservations of a robot once it found another path)
                #  advantage: a robot without a path would not cancel other robots
                #  improvement: after all robots planned, try to plan again and see if there are now better paths
                # todo: make the path finding always return a valid path if possible
                #  (does not have to reach the goal but should avoid collisions)
                #  When using time_horizon this is already the case!
                # there is no path for robot i -> he will wait -> reserve his waiting position BUT:
                # it is possible that the waiting cell is already reserved -> the robot that reserved the cell has to be stopped
                # to prevent a crash
                if robot_id in [781, 689]:
                    print("stop")
                for step in range(self.time_horizon):
                    waiting_position = (last_loc, -1, step + 1)
                    if self.is_reserved(*waiting_position, current_robot_id=robot_id):
                        # check who reserved it and cancel his actions
                        if self.verbose:
                            reserved_by = self.edge_hash_to_robot_id[waiting_position]
                            print(f"{robot_id:03} waiting - cancel actions of {reserved_by:03}")
                        collision_group, stopped_robots_count = self.handle_conflict(*waiting_position)
                        waiting_robots += stopped_robots_count
                        collision_groups.append(collision_group)

                    self.add_reservation(*waiting_position, robot_id)
        return self.next_actions[0], path_length_sum, waiting_robots, waiting_robot_ids

    def prereserve_cells_based_on_robot_positions(self, try_fix_stuck_robots):
        """
        before planning, reserve the cells where the robots cannot leave immediately (e.g. if the robots face a wall)
        :param try_fix_stuck_robots: robots in this list get even more reservations which can help to better
        "run away" from higher priority robots
        """
        robot_position_map = get_robot_position_map(self.env)
        for robot_id in range(self.env.num_of_agents):
            # check if the robot is able to change its position in the next time step
            #  if not -> already reserve the cell the robot is currently in to prevent deadlocks
            position, orientation = self.env.curr_states[robot_id].location, self.env.curr_states[robot_id].orientation
            cell_in_front_of_robot = get_valid_forwards_neighbor_cell(self.env, position, orientation)
            obstacle_in_front_of_robot = cell_in_front_of_robot is None
            if obstacle_in_front_of_robot:
                # there is an obstacle in front of the robot -> reserve the cell the robot is currently in
                self.add_reservation(position, -1, time_step=1, robot_index=robot_id)
                if try_fix_stuck_robots and robot_id in try_fix_stuck_robots:
                    self.add_reservation(position, -1, time_step=2, robot_index=robot_id)
            elif robot_position_map[cell_in_front_of_robot] == 1:
                # there is another robot in front of the robot -> reserve the cell the robot is currently in
                self.add_reservation(position, -1, time_step=1, robot_index=robot_id)
                # todo reserve another step if the other robot is heading in the direction of the robot
                # todo: two is too much: correct: one and only if the robot faces the current robot / two if it faces it and one left or right rotation would not help the other robot to move out of the way
                # todo only make two reservations if the left and right neighbor cells of the robot
                #  are also obstacles or other robots (does not have to be better in every case)
                if try_fix_stuck_robots and robot_id in try_fix_stuck_robots:
                    self.add_reservation(position, -1, time_step=2, robot_index=robot_id)
                    self.add_reservation(position, -1, time_step=3, robot_index=robot_id)

    def update_next_actions(self, path, robot_id):
        # convert the path to actions
        prev_loc = self.env.curr_states[robot_id].location
        prev_ori = self.env.curr_states[robot_id].orientation
        for i in range(min(len(path), self.replanning_period)):
            new_location = path[i][0]
            new_orientation = path[i][1]
            if new_location != prev_loc:
                self.next_actions[i][robot_id] = Action.FW.value
            elif new_orientation != prev_ori:
                incr = new_orientation - prev_ori
                if incr == 1 or incr == -3:
                    self.next_actions[i][robot_id] = Action.CR.value
                elif incr == -1 or incr == 3:
                    self.next_actions[i][robot_id] = Action.CCR.value
            prev_loc = new_location
            prev_ori = new_orientation

    def reserve_path_if_possible(self, last_loc, path, robot_id):
        time_step = 1
        for step in range(self.time_horizon):
            if step < len(path):
                p = path[step]
            else:
                p = path[-1]  # take the last position if path ends before time horizon
            try:
                self.add_reservation(last_loc, p[0], time_step, robot_id, fail_if_already_reserved=True)
            except RuntimeError as e:
                self.revoke_all_reservations_of_robot(robot_id)
                raise e
            last_loc = p[0]
            time_step += 1
        return last_loc

    def add_reservation(self, start: int, end: int, time_step: int, robot_index: int, fail_if_already_reserved=False):
        """
        add a path to the reservation table
        :param start: start cell index
        :param end: end cell index: -1 if same as start
        :param time_step: reservation timestep
        :param robot_index: id of the reserving robot
        :param fail_if_already_reserved: raises an error if the cell is already reserved by another robot
        """
        if end == -1:
            end = start
        cell_hash = (end, -1, time_step)
        if (self.debug_mode or fail_if_already_reserved) and cell_hash in self.reservation and \
                self.edge_hash_to_robot_id[cell_hash] != robot_index:
            raise RuntimeError(f"robot {robot_index} tried to reserve cell {cell_hash}, but it is already reserved "
                               f"by robot {self.edge_hash_to_robot_id[cell_hash]}")
        self.reservation.add(cell_hash)  # reserve the end cell itself
        self.edge_hash_to_robot_id[cell_hash] = robot_index  # to make it easy to lookup which robot reserved which cell
        if start != end:
            edge_hash = (start, end, time_step)
            self.reservation.add(edge_hash)  # reserve the edge
            self.edge_hash_to_robot_id[
                edge_hash] = robot_index  # to make it easy to lookup which robot reserved which edge

    def handle_conflict(self, start: int, end: int, time_step: int, level=0) -> tuple[list[int], int]:
        """
        check who reserved the cell and cancel his actions and reservations - make him wait
        if the stopped robot would collide with another robot -> stop the other robot also recursively
        :param start: start cell
        :param end: end cell
        :param time_step: time step
        :return: list of ids of robots that were stopped
        """
        # todo: check if there is an easy & quick reroute of the colliding robot possible
        colliding_robot_id = self.edge_hash_to_robot_id[(start, end, time_step)]
        collision_group_ids = [colliding_robot_id]
        self.revoke_all_reservations_of_robot(colliding_robot_id)
        stopped_robot_count = 1
        for step in range(self.replanning_period):
            self.next_actions[step][colliding_robot_id] = Action.W.value  # make colliding robot wait
        for step in range(self.time_horizon):
            # if the colliding robot which will now wait would collide with another robot -> stop the other robot also
            stopped_robot_location = self.env.curr_states[colliding_robot_id].location
            wait_cell_hash_of_stopped_robot = (stopped_robot_location, -1, step)
            if self.is_reserved(*wait_cell_hash_of_stopped_robot, current_robot_id=colliding_robot_id):
                cell_was_reserved_by = self.edge_hash_to_robot_id[wait_cell_hash_of_stopped_robot]
                if self.debug_mode and level >= 50:
                    print("recursion limit reached")
                new_collision_group_ids, new_stopped_robot_count = self.handle_conflict(
                    *wait_cell_hash_of_stopped_robot, level=level + 1)
                stopped_robot_count += new_stopped_robot_count
                collision_group_ids.extend(new_collision_group_ids)
            try:
                self.add_reservation(*wait_cell_hash_of_stopped_robot, colliding_robot_id)
            except RuntimeError as e:
                pass
        return collision_group_ids, stopped_robot_count

    def revoke_all_reservations_of_robot(self, robot_id: int):
        """
        remove all reservations of a robot
        :param robot_id: id of the robot
        """
        for edge_hash, r_id in list(self.edge_hash_to_robot_id.items()):
            if r_id == robot_id:
                self.reservation.remove(edge_hash)
                del self.edge_hash_to_robot_id[edge_hash]
