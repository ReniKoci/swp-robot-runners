import unittest
from typing import Optional

import numpy as np
from matplotlib import pyplot as plt

from models import Orientation, Env
from test_utils import grids_to_env
from util import DistanceMap, convert_2d_to_1d_coordinate


def plot_distance_map(distances: list[Optional[int]], env: Env):
    grid_size = (env.cols, env.rows)
    grid = [[None for _ in range(grid_size[0])] for _ in range(grid_size[1])]
    # Update the grid with the smallest distances
    for i in range(len(distances)):
        if distances[i] is not None:
            # Determine the cell position (ignoring orientation)
            cell_index = i // 4
            row, col = divmod(cell_index, grid_size[0])

            # Update the grid cell with the smallest distance
            if grid[row][col] is None or distances[i] < grid[row][col]:
                grid[row][col] = distances[i]
    for index, cell in enumerate(env.map):
        if cell == 1:
            row, col = divmod(index, grid_size[0])
            grid[row][col] = -1

    grid_array = np.array([[np.nan if x is None else x for x in row] for row in grid])

    plt.figure(figsize=(8, 8))
    cmap = plt.cm.viridis
    cmap.set_under('black')  # Set color for obstacles
    plt.imshow(grid_array, cmap=cmap, interpolation='nearest', vmin=0)

    # Annotate the cells with the original values
    for i in range(grid_size[1]):
        for j in range(grid_size[0]):
            value = grid[i][j]
            if value == -1:
                continue
            text = '∞' if value is None else int(value)
            plt.text(j, i, text, ha='center', va='center', color='black')

    # Set plot details
    plt.title('Grid of Smallest Distances')
    plt.colorbar(label='Distance (with ∞ for None)')
    plt.xticks(range(grid_size[0]))
    plt.yticks(range(grid_size[1]))
    plt.show()


class UtilTest(unittest.TestCase):
    def test_distance_map(self):
        map = [
            [1, 0, 1, 1, 1, 0, 0],
            [1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 1, 0, 0],
            [1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
        ]
        env = grids_to_env(map, [])
        target = convert_2d_to_1d_coordinate((1, 0), env.cols)
        distance_map = DistanceMap(target, env)
        self.assertEqual(
            distance_map.get_distance(env,
                                      convert_2d_to_1d_coordinate((1, 1), env.cols),
                                      Orientation.NORTH.value),
            1)
        plot_distance_map(distance_map.distance_map, env)

        self.assertEqual(
            distance_map.get_distance(env,
                                      convert_2d_to_1d_coordinate((1, 2), env.cols),
                                      Orientation.EAST.value),
            3)
        plot_distance_map(distance_map.distance_map, env)

        self.assertEqual(
            distance_map.get_distance(env,
                                      convert_2d_to_1d_coordinate((6, 8), env.cols),
                                      Orientation.NORTH.value),
            15)
        plot_distance_map(distance_map.distance_map, env)

    def test_generate_visualization_for_benchmark_tool(self):
        # <config name>, <tasks completed at timestep x>
        data = {
            "config0": [0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 3, 3, 4, 5, 7, 8, 8, 8, 9],
            "config1": [0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 4, 4, 4, 4, 4, 5, 7, 7, 7],
            "config2": [1, 1, 1, 2, 2, 4, 4, 4, 4, 4, 5, 7, 8, 9, 9, 9, 9, 10, 13],
        }
        print("hello")
        # use matplotlib to generate a line chart
        # x - axis: timestep
        # y - axis: tasks completed
        # each line represents a config
        # save the chart as a file


    def test_generate_visualization_for_benchmark_tool_with_range(self):
        # <config name>, <tasks completed at timestep x>
        # one config has multiple runs (because of randomness they sometimes perform better or worse)
        data = {
            "Manhattan, TimeHorizon: 4, Replanning: True": [
                [0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 3, 3, 4, 5, 7, 8, 8, 8, 9],
                [0, 1, 1, 2, 3, 3, 3, 3, 4, 5, 7, 8, 8, 8, 9, 9, 9, 9, 9],
                [0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 3, 3, 4, 5, 7, 8, 8, 8, 9],
            ],
            "Manhattan, TimeHorizon: 8, Replanning: False": [
                [0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 4, 4, 4, 4, 4, 5, 7, 7, 7],
                [0, 0, 0, 0, 1, 1, 1, 2, 2, 4, 4, 4, 4, 4, 5, 7, 7, 7, 8],
                [0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 5, 7, 7, 9],
            ],
            "TrueDistance, TimeHorizon: 6, Replanning: True": [
                [1, 1, 2, 2, 2, 4, 4, 4, 4, 5, 7, 7, 8, 10, 10, 12, 14, 14, 15],
                [1, 1, 1, 2, 2, 4, 4, 4, 4, 4, 5, 7, 8, 9, 9, 9, 9, 10, 13],
                [2, 2, 4, 4, 4, 4, 4, 5, 7, 8, 9, 9, 9, 9, 10, 13, 14, 17, 18],
            ],
        }

        # create new diagram
        plt.figure(figsize=(10, 6))

        # iterate through configs
        for config, runs in data.items():
            # covert to np for easier calculation
            runs_array = np.array(runs)

            # calculate the min, max and avarage
            mean_values = np.mean(runs_array, axis=0)
            min_values = np.min(runs_array, axis=0)
            max_values = np.max(runs_array, axis=0)

            # plt the avarage line
            plt.plot(range(len(mean_values)), mean_values, label=config)

            # fill in the space between min and max
            plt.fill_between(range(len(mean_values)), min_values, max_values, alpha=0.2)

        # description
        plt.xlabel("Timestep")
        plt.ylabel("Tasks Completed")
        plt.title("Benchmark Tool Visualization with Range")
        plt.legend()

        # X,Y Axis Whole Numbers
        plt.xticks(np.arange(0, len(mean_values), step=1))
        plt.yticks(np.arange(0, np.max(max_values) + 1, step=1))

        # save as picture
        #plt.savefig("Benchmarktool_viz_avarage.png")

        # show diagram
        plt.show()
