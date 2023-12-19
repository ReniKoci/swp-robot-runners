from python.models import BasePlanner
from python.planner_space_time_a_star import SpaceTimeAStarPlanner
import os


class pyMAPFPlanner:  # this is the wrapper class that will be called by the c++ application
    planner: BasePlanner  # your planner should inherit from BasePlanner

    def __init__(self, pyenv=None, desired_planner="astar") -> None:
        if desired_planner is None:
            desired_planner = os.getenv("PLANNER")  # read the environment variable to determine which planner to use
        if desired_planner == "astar":
            self.planner = SpaceTimeAStarPlanner(pyenv)
        elif desired_planner == "ICTS":
            pass
            # todo: add your planner here: self.planner = ICTSPlanner(pyenv)
        else:
            raise NotImplementedError("No planner was specified.")

    def initialize(self, preprocess_time_limit: int):
        self.planner.initialize(preprocess_time_limit=preprocess_time_limit)

    def plan(self, time_limit):
        self.planner.plan(time_limit=time_limit)
