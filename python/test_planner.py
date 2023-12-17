import unittest
from copy import deepcopy
from itertools import cycle
from typing import Union

import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.animation import FuncAnimation

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


def animate_grid(envs: list, filename='grid_animation.gif', interval=200):
    grid_size = (envs[0].rows, envs[0].cols)
    orientation_symbols = ['→', '↓', '←', '↑']
    colors = cycle(['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta', 'yellow'])

    # Initialize a grid
    grid = np.zeros(grid_size)

    # Prepare the figure and axes
    fig, ax = plt.subplots()
    im = ax.imshow(grid)

    # Set plot labels, title, and axis limits
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Grid Animation with Obstacles, Goals, and Robots')
    ax.set_xticks(np.arange(-0.5, grid_size[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid_size[0], 1), minor=True)
    ax.grid(which='minor', color='black', linestyle='-', linewidth=2)
    robot_colors = {}
    for idx, _ in enumerate(envs[0].curr_states):
        robot_colors[idx] = next(colors)

    def update(frame):
        env = envs[frame]
        # Clear previous contents
        for txt in ax.texts:
            txt.remove()
        for patch in ax.patches:
            patch.remove()

        for i in range(env.rows * env.cols):
            x, y = i % env.cols, i // env.cols
            if env.map[i] == 1:
                patch = patches.Rectangle((x - 0.5, y - 0.5), 1, 1, color='white')
            else:
                patch = patches.Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', linewidth=1, edgecolor='white')
            ax.add_patch(patch)

        # Draw goal locations and robots
        for idx, goals in enumerate(env.goal_locations):
            color = robot_colors.get(idx, 'grey')
            for goal_cell_index in goals:
                gx, gy = goal_cell_index[0] % env.cols, goal_cell_index[0] // env.cols
                ax.text(gx, gy, f'G{idx}', ha='center', va='center', color=color, fontsize=16, fontweight='bold')

        for idx, state in enumerate(env.curr_states):
            rx, ry = state.location % env.cols, state.location // env.cols
            color = robot_colors.get(idx, 'grey')
            ax.add_patch(plt.Circle((rx, ry), 0.3, color=color, alpha=0.5))
            ax.text(rx, ry, f'R{idx}{orientation_symbols[state.orientation]}',
                    ha='center', va='center', color='white')

        return [im]

    # Create the animation
    anim = FuncAnimation(fig, update, frames=len(envs), interval=interval, blit=True)

    # Save the animation as a GIF
    anim.save(filename, writer='pillow')
    plt.close()


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
        return grids_to_env(grid, goal_grid, "huge_test_map")

    def test_get_neighbors(self):
        env = self.get_test_env()
        first_robot_state = env.curr_states[0]
        neighbors = get_neighbors(env, first_robot_state.location, first_robot_state.orientation)
        # robot is currently at cell 5 with orientation 0 (east)
        # -> neighbors should be the positions after move fw / turn left / right
        self.assertListEqual(neighbors, [(5, 3), (5, 1)])

    def test_basic_planning_one_step(self):
        planner = SpaceTimeAStarPlanner(visualize=True)
        planner.env = env = self.get_test_env()
        print_grid(env)
        actions = planner.plan(None)
        env = update_env(env, actions)
        print_grid(env)
        self.assertEqual(actions[0], Action.CCR.value)

    def test_basic_planning_huge_map(self):
        planner = SpaceTimeAStarPlanner(visualize=True, animate=False)
        planner.env = env = self.get_huge_test_env()
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
        self.assertNotEqual(actions, [Action.W.value, Action.W.value])

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

    def test_multiple_steps_multiple_robots(self):
        grid = [
            [0, 0000, 1, 0],
            [0, 0000, 1, 0],
            [0, "2>", 1, "1v"],
            [0, 0000, 0, 0]
        ]
        goal_grid = [
            [0, 0, 0, 2],
            [0, 1, 0, 0],
            [0, 0, 0, 0]
        ]
        env = grids_to_env(grid, goal_grid)
        planner = SpaceTimeAStarPlanner()
        planner.env = env
        all_envs = [deepcopy(env)]
        while True:
            actions = planner.plan(None)
            env = update_env(env, actions)
            all_envs.append(deepcopy(env))
            if all(a == Action.W.value for a in actions):
                break
        animate_grid(all_envs)
        for robot_state, goal in zip(env.curr_states, env.goal_locations):
            self.assertEqual(robot_state.location, goal[0][0])



