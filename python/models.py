from dataclasses import dataclass
from enum import Enum


class Action(Enum):
    FW = 0
    CR = 1
    CCR = 2
    W = 3


class Orientation(Enum):
    # 0:east, 1:south, 2:west, 3:north
    EAST = 0
    SOUTH = 1
    WEST = 2
    NORTH = 3

@dataclass
class State:  # state of an agent
    location: int
    orientation: int  # 0:east, 1:south, 2:west, 3:north
    timestep: int


@dataclass
class Env:
    cols: int  # number of columns
    rows: int  # number of rows
    map: list[int]  # 0 - empty; 1 - wall (agents do not affect the status)
    map_name: str
    num_of_agents: int
    curr_timestep: int
    curr_states: list[State]  # position and orientation of each agent
    goal_locations: list[
        [tuple[int, int]]
    ]  # goal of each agent, timestep when the target was revealed
