from models import Action, BasePlanner


class CBSPlanner(BasePlanner):
    def __init__(self, pyenv=None) -> None:
        super().__init__(pyenv, "CBS-Planner")

    def initialize(self, preprocess_time_limit: int):
        return True

    def plan(self, time_limit) -> list[int]:
        # return wait actions for all the robots
        actions = [Action.W.value] * self.env.num_of_agents
        return actions


