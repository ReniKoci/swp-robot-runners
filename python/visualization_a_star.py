import datetime
import math

import numpy as np
import matplotlib.colors as mcolors
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation

from python.models import Env


class AStarVisualizer:
    """
    provides callback functions that can be used to collect and then visualize path finding data
    """
    GENERATE_ANIMATIONS = True

    open_list_visualization_data = []  # Data for visualization
    explored_counts = {}
    explored_counts_list = [{}]
    grid_data = []
    grid_data_v2 = []
    current_frame_data = {}
    current_frame_data_v2 = {}
    min_f_values_v2 = {}
    lowest_g_values = {}
    lowest_f_values = {}
    current_f_values_v2 = {}
    current_f_values = {}

    def reset(self):
        self.open_list_visualization_data = []  # Data for visualization
        self.explored_counts = {}
        self.explored_counts_list = [{}]
        self.grid_data = []
        self.grid_data_v2 = []
        self.current_frame_data = {}
        self.current_frame_data_v2 = {}
        self.min_f_values_v2 = {}
        self.lowest_g_values = {}
        self.lowest_f_values = {}
        self.current_f_values_v2 = {}
        self.current_frame_data = {}
        self.current_f_values = {}

    def commit_open_list(self, ol: list):
        self.open_list_visualization_data.append(ol)

    def new_step(self):
        self.current_frame_data = {}
        self.current_f_values = {}
        self.current_f_values_v2 = {}

    def update_data(self, env: Env, open_list, current_pos: int, current_ori: int, g: int):
        self.explored_counts[current_pos] = self.explored_counts.get(current_pos, 0) + 1  # increase by 1
        self.explored_counts_list.append(self.explored_counts.copy())  # save explored counts history
        # Update explored counts and lowest g values
        if current_pos not in self.lowest_g_values or g < self.lowest_g_values[current_pos]:
            self.lowest_g_values[current_pos] = g  # found a lower g value

        # self.current_f_values_v2[(position, orientation)] = [g+h] # todo: do we need this?
        for node in open_list.queue:
            f = node[0]
            pos = node[3][0]
            lowest_f = self.lowest_f_values.get(pos, math.inf)
            if f < lowest_f:
                self.lowest_f_values[pos] = f
            current_lowest_f = self.current_f_values.get(pos, math.inf)
            if f < current_lowest_f:
                self.current_f_values[pos] = f
            ori = node[3][1]
            current_lowest_f = self.min_f_values_v2.get((pos, ori), math.inf)
            if f < current_lowest_f:
                self.min_f_values_v2[(pos, ori)] = f

            l = self.current_f_values_v2.get((pos, ori))
            if l is not None:
                l.append(f)
            else:
                self.current_f_values_v2[(pos, ori)] = [f]

        for pos in range(0, env.cols * env.rows):
            self.current_frame_data[pos] = {
                'f_value': self.current_f_values.get(pos, np.inf),
                'lowest_f_value': self.lowest_f_values.get(pos, np.inf),
                'lowest_g_value': self.lowest_g_values.get(pos, np.inf),
                'visit_count': self.explored_counts.get(pos, 0),
                'current': pos == current_pos
            }
        self.grid_data.append(self.current_frame_data)

        for pos in range(0, env.cols * env.rows):
            orientations_data = []
            for ori in range(4):  # Assuming 4 orientations
                is_current = pos == current_pos and ori == current_ori
                f_val = self.min_f_values_v2.get((pos, ori), np.inf)  # f_value for specific orientation

                orientation_data = {
                    'min_f_value': f_val,
                    'f_value': self.current_f_values_v2.get((pos, ori), [math.inf]),
                    'current': is_current
                }
                orientations_data.append(orientation_data)

            self.current_frame_data_v2[pos] = {'orientations': orientations_data}

        self.grid_data_v2.append(self.current_frame_data_v2.copy())

    def save_visualizations(self, env, start, end):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H_%M_%S")
        visualize_grid_with_lowest_g(self.open_list_visualization_data, env.map,
                                     grid_size=(env.cols, env.rows), filename=f"lowest_g_{timestamp}_{env.map_name}_start_{start}_end_{end}.png")
        visualize_explored_count(self.explored_counts, env.map, grid_size=(env.cols, env.rows), filename=f"explored_count_{timestamp}_{env.map_name}_start_{start}_end_{end}.png")
        if self.GENERATE_ANIMATIONS:
            animate_combined_v2(self.grid_data_v2, env.map, interval=50, grid_size=(env.cols, env.rows),
                                filename=f"{env.map_name}_{timestamp}_start_{start}_end_{end}.gif")


def visualize_grid_with_lowest_g(open_list_data, env_map: list[int], grid_size, filename="lowest_g.png"):
    """
    Visualizes a grid where each cell shows the lowest g value from the last snapshot of open_list_data.
    Also plots obstacles as black cells.
    :param filename: file name
    :param open_list_data: A list of lists, where each inner list contains tuples (position, f value)
    :param env_map: 1D array representing the environment map, where 1 indicates an obstacle
    :param grid_size: Tuple (width, height) representing the size of the grid
    """
    # Extract the last snapshot
    last_snapshot = open_list_data[-1]
    grid_size = (grid_size[1], grid_size[0])
    # Initialize a grid with a high value (assuming lower f is better)
    grid = np.full(grid_size, np.inf)

    # Find the lowest f value for each cell in the last snapshot
    for snapshot in open_list_data:
        for position, f_value in snapshot:
            y, x = np.unravel_index(position,
                                    grid_size)
            grid[y, x] = min(grid[y, x], f_value)

    # Mark obstacles in the grid
    for position, is_obstacle in enumerate(env_map):
        if is_obstacle == 1:
            y, x = np.unravel_index(position, grid_size)
            grid[y, x] = -1  # Marking the obstacle

    # Plotting the grid
    plt.figure()
    cmap = plt.cm.viridis
    cmap.set_under('black')  # Set color for obstacles
    plt.imshow(grid, cmap=cmap, interpolation='nearest', vmin=0)
    plt.colorbar(label='Lowest g Value')
    plt.title('Grid Visualization of Lowest g Values with Obstacles')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Annotating the grid with g values
    for (j, i), g_value in np.ndenumerate(grid):
        if g_value != np.inf and g_value != -1:
            plt.text(i, j, f"{g_value:.0f}", ha='center', va='center', color='white')

    plt.savefig(filename)
    plt.close()




def visualize_explored_count(explored_counts, env_map, grid_size, filename="explored_count.png"):
    """
    Visualizes a grid where each cell shows the number of times it has been explored.
    Obstacles are plotted as black cells.
    :param explored_counts: Dictionary with cell positions as keys and explored counts as values
    :param env_map: 1D array representing the environment map, where 1 indicates an obstacle
    :param grid_size: Tuple (width, height) representing the size of the grid
    """
    # Initialize a grid with zeros (no exploration)
    grid_size = (grid_size[1], grid_size[0])
    grid = np.zeros(grid_size)

    # Populate the grid with exploration counts
    for position, count in explored_counts.items():
        y, x = np.unravel_index(position, grid_size)
        grid[y, x] = count

    # Mark obstacles in the grid
    for position, is_obstacle in enumerate(env_map):
        if is_obstacle == 1:
            y, x = np.unravel_index(position, grid_size)
            grid[y, x] = -1  # Marking the obstacle

    # Plotting the grid
    plt.figure()
    cmap = plt.cm.viridis
    cmap.set_under('black')  # Set color for obstacles
    plt.imshow(grid, cmap=cmap, interpolation='nearest', vmin=0)
    plt.colorbar(label='Exploration Count')
    plt.title('Grid Visualization of Explored Counts with Obstacles')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Annotating the grid with exploration counts
    for (j, i), count in np.ndenumerate(grid):
        if count > 0:
            plt.text(i, j, int(count), ha='center', va='center', color='white')

    plt.savefig(filename)
    plt.close()

def animate_combined_v2(grid_data, env_map, grid_size, filename='combined_animation.gif', interval=200, dpi=100):
    """
    Creates an animation with each cell divided into 4 triangles for orientations.
    Each triangle is colored based on the f value.
    """
    grid_size = (grid_size[1], grid_size[0])
    plt.gca().invert_yaxis()

    # Define the figure and axes
    fig, ax = plt.subplots(figsize=(2 + int(grid_size[1]*1.7), int(grid_size[0]*1.7)))
    ax.set_xlim(-0.5, grid_size[1]-0.5)
    ax.set_ylim(-0.5, grid_size[0]-0.5)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Combined Animation with Orientation')

    # Define the colormap and normalization
    cmap = plt.cm.viridis
    values = [data['orientations'][0]['min_f_value'] for frame in grid_data for data in frame.values() if data['orientations'][0]['min_f_value'] != math.inf]
    maximum = max(values or [0])
    norm = mcolors.Normalize(vmin=0, vmax=maximum)

    # Function to update each frame in the animation
    def update(frame):
        ax.clear()
        ax.set_xlim(-0.5, grid_size[1]-0.5)
        ax.set_ylim(-0.5, grid_size[0]-0.5)

        for position, data in grid_data[frame].items():
            y, x = np.unravel_index(position, grid_size)
            y = grid_size[0]-y-1

            # Define triangles in each grid cell with correct orientation
            triangles = [
                [(x-0.5, y-0.5), (x, y), (x-0.5, y+0.5)],  # West (2)
                [(x-0.5, y+0.5), (x, y), (x+0.5, y+0.5)],  # South (1)
                [(x+0.5, y-0.5), (x, y), (x+0.5, y+0.5)],  # East (0)
                [(x-0.5, y-0.5), (x, y), (x+0.5, y-0.5)],  # North (3)
            ]

            for i, triangle in enumerate(triangles):
                orientation_data = data['orientations'][i]
                f_value = min(orientation_data.get('f_value') or [math.inf])
                color = cmap(norm(f_value)) if f_value != math.inf else 'black'
                if orientation_data['current']:
                    poly = Polygon(triangle, facecolor=color, fill=True, edgecolor='red', lw=3)
                    poly.zorder=99
                else:
                    poly = Polygon(triangle, facecolor=color, fill=True, edgecolor='black', lw=0)
                ax.add_patch(poly)

                # Annotate with orientation-specific information
                mid_point = np.mean(triangle, axis=0)
                annotation = f"f: {orientation_data['f_value']}\nmin f: {orientation_data['min_f_value']}"
                ax.text(mid_point[0], mid_point[1], annotation, ha='center', va='center', color='black', fontsize=6, zorder=100)

        # Update obstacles
        for position, is_obstacle in enumerate(env_map):
            if is_obstacle == 1:
                y, x = np.unravel_index(position, grid_size)
                y = grid_size[0]-y-1
                ax.add_patch(plt.Rectangle((x-0.5, y-0.5), 1, 1, fill=True, color='black'))

        return [ax]

    anim = FuncAnimation(fig, update, frames=len(grid_data), interval=interval)
    print("saving animation...")
    anim.save(filename, writer='pillow', dpi=dpi)
    plt.close()