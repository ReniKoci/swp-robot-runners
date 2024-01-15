from typing import List, Tuple, Set
from queue import PriorityQueue
from models import Action, BasePlanner, Heuristic

class CBSPlanner(BasePlanner):
    debug_mode = True

    def __init__(self, pyenv=None) -> None:
        super().__init__(pyenv, "CBS-Planner")

    def initialize(self, preprocess_time_limit: int):
        return True  # todo: implement preprocessing or optimal pathfinding

    def plan(self, time_limit) -> list[int]:
        if self.last_planning_step + self.replanning_period <= self.env.curr_timestep:
            self.last_planning_step = self.env.curr_timestep
            return self.plan_with_random_restarts(time_limit)
        else:
            return self.next_actions[self.env.curr_timestep - self.last_planning_step]

