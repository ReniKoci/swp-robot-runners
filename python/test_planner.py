import unittest
from typing import Union

from python.planner_space_time_a_star import SpaceTimeAStarPlanner
from python.util import get_neighbors
from python.models import Env, State, Orientation, Action


def grids_to_env(grid: list[list[Union[int, str]]], goal_grid: list[list[int]], map_name="map") -> Env:
    one_d_map = [cell for row in grid for cell in row]
    one_d_goal_grid = [cell for row in goal_grid for cell in row]

    number_of_robots = len([cell for cell in one_d_map if cell != 0 and cell != 1])

    robot_states = [State(location=0, orientation=0, timestep=-1) for _ in range(number_of_robots)]
    goal_locations: list[[tuple[int, int]]] = [[(-1, -1)] for _ in range(number_of_robots)]
    for i, cell in enumerate(one_d_map):
        if type(cell) == str:
            # it is a robot
            robot_nr: int = int(cell[0]) - 1
            last_char = cell[-1]
            char_to_orientation_map = {
                ">": Orientation.EAST.value,
                "v": Orientation.SOUTH.value,
                "<": Orientation.WEST.value,
                "^": Orientation.NORTH.value
            }
            orientation = char_to_orientation_map.get(last_char)
            robot_states[robot_nr] = State(location=i, orientation=orientation, timestep=0)
            one_d_map[i] = 0

    for i, cell_value in enumerate(one_d_goal_grid):
        if cell_value > 0:
            robot_nr = cell_value - 1
            goal_locations[robot_nr] = [[i, 0]]  # position, timestep when target was revealed

    env = Env(cols=len(grid[0]), rows=len(grid), map=one_d_map, map_name=map_name, num_of_agents=number_of_robots,
              curr_timestep=0, curr_states=robot_states, goal_locations=goal_locations)
    return env


def update_env(env: Env, actions: list[int]) -> Env:
    for i, state in enumerate(env.curr_states):
        action = Action(actions[i])
        if action == Action.W:
            continue
        if action == Action.CR:
            state.orientation += 1
            if state.orientation >= 4:
                state.orientation = 0
        if action == Action.CCR:
            state.orientation -= 1
            if state.orientation <= -1:
                state.orientation = 3
        if action == Action.FW:
            candidates = [
                state.location + 1,
                state.location + env.cols,
                state.location - 1,
                state.location - env.cols,
            ]
            state.location = candidates[state.orientation]
    return env


def print_grid(env: Env):
    orientation_symbols = ['→', '↓', '←', '↑']

    # Create a 2D array to represent the grid
    grid = [['   ' for _ in range(env.cols)] for _ in range(env.rows)]

    # Mark obstacles
    for i in range(env.rows * env.cols):
        if env.map[i] == 1:
            grid[i // env.cols][i % env.cols] = '███'

    # Mark goal locations
    for idx, goals in enumerate(env.goal_locations):
        for goal_cell_index in goals:
            x = goal_cell_index[0] % env.cols
            y = goal_cell_index[0] // env.cols
            grid[y][x] = f'G{idx:02d}'

    # Place robots with their index
    for idx, state in enumerate(env.curr_states):
        x = state.location % env.cols
        y = state.location // env.cols
        grid[y][x] = f'R{idx}{orientation_symbols[state.orientation]}'

    # Print the grid with horizontal and vertical lines
    horizontal_line = '+' + '-----+' * env.cols

    print(horizontal_line)
    for row in grid:
        print('| ' + ' | '.join(row) + ' |')
        print(horizontal_line)


class PlannerTest(unittest.TestCase):
    def get_test_env(self) -> Env:
        grid = [  # 0 - empty; 1 - wall; "<i><o>" i - robot nr (has to be bigger than 0) o - orientation (<>v^)
            [0, 0000, 0, 0],
            [0, "1>", 1, 0],
            [0, 0000, 1, 0],
            [0, 0000, 0, 0],
        ]
        goal_grid = [
            [0, 0, 0, 0],  # 0 - empty; 1-infinity - goal of robot 1 - inf
            [0, 0, 0, 1],  # goal of robot 1 (goals have to be the robot
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
        return grids_to_env(grid, goal_grid, "small_test_map")

    def get_huge_test_env(self):
        grid = [  # 0 - empty; 1 - wall; "<i><o>" i - robot nr (has to be bigger than 0) o - orientation (<>v^)
            [0, 0, 0, 0, 0, 0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, "1>", 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0000, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        ]
        goal_grid = [
            # 0 - empty; 1-infinity - goal of robot 1 - inf
            # goal of robot 1 (goals have to be the robot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ]
        return grids_to_env(grid, goal_grid)

    def test_get_neighbors(self):
        env = self.get_test_env()
        first_robot_state = env.curr_states[0]
        neighbors = get_neighbors(env, first_robot_state.location, first_robot_state.orientation)
        # robot is currently at cell 5 with orientation 0 (east)
        # -> neighbors should be the positions after move fw / turn left / right
        self.assertListEqual(neighbors, [(5, 3), (5, 1)])

    def test_basic_planning_one_step(self):
        planner = SpaceTimeAStarPlanner()
        planner.env = env = self.get_test_env()
        planner.VISUALIZE = True
        print_grid(env)
        actions = planner.plan(None)
        env = update_env(env, actions)
        print_grid(env)
        self.assertEqual(actions[0], Action.CCR.value)

    def test_basic_planning_huge_map(self):
        planner = SpaceTimeAStarPlanner()
        planner.env = env = self.get_huge_test_env()
        planner.VISUALIZE = False
        print_grid(env)
        actions = planner.plan(None)
        env = update_env(env, actions)
        print_grid(env)
        self.assertEqual(actions[0], Action.FW.value)

    def test_basic_planning_two_robots(self):
        grid = [
            [0000, 0, 0000, 0],
            ["1>", 0, "2<", 0],
            [0000, 0, 0000, 0]
        ]
        goal_grid = [
            [0, 0, 0, 0],
            [2, 0, 0, 1],
            [0, 0, 0, 0]
        ]
        env = grids_to_env(grid, goal_grid)
        planner = SpaceTimeAStarPlanner()
        planner.env = env

        print_grid(env)
        actions = planner.plan(None)
        env = update_env(env, actions)
        print_grid(env)
        self.assertEqual(actions[0], Action.FW.value)
        actions = planner.plan(None)
        env = update_env(env, actions)
        print_grid(env)

    def test_avoid_edge_collision(self):
        grid = [
            ["1>", "2<"]
        ]
        goal_grid = [
            [2, 1]
        ]
        env = grids_to_env(grid, goal_grid)
        planner = SpaceTimeAStarPlanner()
        planner.env = env
        actions = planner.plan(None)
        self.assertListEqual(actions, [Action.W.value, Action.W.value])

    def test_avoid_cell_collision(self):
        grid = [
            ["1>", 0, "2<"]
        ]
        goal_grid = [
            [2, 0, 1]
        ]
        env = grids_to_env(grid, goal_grid)
        planner = SpaceTimeAStarPlanner()
        planner.env = env
        actions = planner.plan(None)
        self.assertNotEquals(actions, [Action.W.value, Action.W.value])

    def test_wait_until_blocking_robot_moved(self):
        grid = [
            [0, 0000, 0],
            [0, 0000, "2<"],
            [0, "1^", 0]
        ]
        goal_grid = [
            [0, 1, 0],
            [0, 2, 0],
            [0, 0, 0]
        ]
        env = grids_to_env(grid, goal_grid)
        planner = SpaceTimeAStarPlanner()
        planner.env = env
        actions = planner.plan(None)
        self.assertEquals(Action(actions[1]), Action.W)
