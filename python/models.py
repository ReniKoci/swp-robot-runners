from abc import abstractmethod
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


class BasePlanner:
    env: Env
    name: str

    def __init__(self, pyenv=None, name="My Planner") -> None:
        self.name = name
        if pyenv is not None:
            self.env: Env = pyenv.env

    @abstractmethod
    def initialize(self, preprocess_time_limit: int):
        """
        :param preprocess_time_limit: limit to preprocess in seconds(?)
        """

    @abstractmethod
    def plan(self, time_limit) -> list[int]:
        """

        :param time_limit: time limit until the next actions have to be returned
        :return: list of actions
        """

