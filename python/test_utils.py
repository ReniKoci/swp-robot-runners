import json
import os
from itertools import cycle
from typing import Union

import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.animation import FuncAnimation

from python.models import Env, State, Orientation, Action
from python.util import convert_1d_to_2d_coordinate


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


def update_env(env: Env, actions: list[int], tasks: list[int] = None, next_task_index: int = None) -> Union[Env, tuple[Env, int]]:
    robot_map = [None for _ in range(env.cols * env.rows)]
    for i, state in enumerate(env.curr_states):
        if robot_map[state.location] is not None:
            raise ValueError(f"Robot {i} collided with robot {robot_map[state.location]} in previous step")
        robot_map[state.location] = i
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
        # if reached goal, update goal location using next_task_index
        if tasks is not None and state.location == env.goal_locations[i][0][0]:
            env.goal_locations[i][0] = (tasks[next_task_index], env.curr_timestep)
            next_task_index += 1
            if next_task_index >= len(tasks):
                next_task_index = 0
    env.curr_timestep += 1
    robot_map = [None for _ in range(env.cols * env.rows)]
    for i, state in enumerate(env.curr_states):
        if robot_map[state.location] is not None:
            raise ValueError(f"Robot {i} collided with robot {robot_map[state.location]} in location {state.location}")
        robot_map[state.location] = i

    if tasks is not None:
        return env, next_task_index
    return env


def print_grid(env: Env, include_robots=True):
    orientation_symbols = ['→', '↓', '←', '↑']

    # Create a 2D array to represent the grid
    grid = [['   ' for _ in range(env.cols)] for _ in range(env.rows)]

    # Mark obstacles
    for i in range(env.rows * env.cols):
        if env.map[i] == 1:
            x, y = i % env.cols, i // env.cols
            grid[y][x] = '███'
    if include_robots:
        # Mark goal locations
        for idx, goals in enumerate(env.goal_locations):
            for goal_cell_index, _ in goals:
                x, y = convert_1d_to_2d_coordinate(goal_cell_index, env.cols)
                grid[y][x] = f'G{idx:02d}'

        # Place robots with their index
        for idx, state in enumerate(env.curr_states):
            x, y = state.location % env.cols, state.location // env.cols
            grid[y][x] = f'R{idx}{orientation_symbols[state.orientation]}'
    else:
        # add cell indices to cells if cell is no wall (1)
        for i in range(env.rows * env.cols):
            if env.map[i] == 0:
                x, y = i % env.cols, i // env.cols
                grid[y][x] = f'{i:03d}'

    # Print the grid with horizontal and vertical lines
    horizontal_line = '+' + '-----+' * env.cols

    print(horizontal_line)
    for row in grid:
        print('| ' + ' | '.join(row) + ' |')
        print(horizontal_line)


def animate_grid(envs: list, filename='lifelong_animation.gif', interval=200):
    grid_size = (envs[0].rows, envs[0].cols)
    orientation_symbols = ['→', '↓', '←', '↑']
    colors = cycle(['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta', 'yellow'])

    # Initialize a grid
    grid = np.zeros(grid_size)

    # Prepare the figure and axes
    fig, ax = plt.subplots(figsize=(grid_size[1] * 0.5, grid_size[0] * 0.5))
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
    anim.save(f"{envs[0].map_name}_{filename}", writer='pillow')
    plt.close()


def load_map(map_file_path) -> tuple[list[int], int, int]:
    """
    load map from file
    :param map_file_path: path
    :return: 1d map out of 0 (empty) and 1 (obstacle), number of rows, number of columns
    """
    with open(map_file_path, 'r') as file:
        lines = file.readlines()

    # Assuming the map is always square and starts after the line 'map'
    map_start_index = lines.index('map\n') + 1
    map_lines = lines[map_start_index:]
    map_grid = []
    for line in map_lines:
        map_row = [0 if char == '.' else 1 for char in line.strip()]
        map_grid.extend(map_row)

    return map_grid, len(map_lines), len(map_lines[0].strip())


def load_agents(agent_file_path):
    with open(agent_file_path, 'r') as file:
        agents = [int(line.strip()) for line in file.readlines()]
    return agents[1:]


def load_tasks(task_file_path):
    with open(task_file_path, 'r') as file:
        tasks = [int(line.strip()) for line in file.readlines()]
    return tasks[1:]


def get_test_env_and_targets_from_config_file(json_file_path) -> tuple[Env, list[int]]:
    with open(json_file_path, 'r') as file:
        config = json.load(file)

    map_grid, width, height = load_map(os.path.join(os.path.dirname(json_file_path), config['mapFile']))
    agents = load_agents(os.path.join(os.path.dirname(json_file_path), config['agentFile']))
    tasks = load_tasks(os.path.join(os.path.dirname(json_file_path), config['taskFile']))

    # Assuming all agents start with orientation 0 and timestep 0
    agent_states = [State(location=pos_ori_hash, orientation=0, timestep=0) for pos_ori_hash in agents]

    # Assuming task goals are single locations revealed at the same timestep for all agents
    task_goals = [[(task, 0)] for task in tasks]

    environment = Env(
        cols=width,  # Assuming fixed size based on the map file
        rows=height,
        map=map_grid,
        map_name=str(os.path.basename(config["mapFile"])).replace(".map", ""),
        num_of_agents=config['teamSize'],
        curr_timestep=0,
        curr_states=agent_states,
        goal_locations=task_goals[:config['teamSize']]
    )

    return environment, tasks
