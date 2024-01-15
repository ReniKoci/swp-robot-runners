import unittest

from models import Env, Action
from planner_CBS import CBSPlanner
from test_utils import grids_to_env, print_grid, update_env


class CbsPlannerTest(unittest.TestCase):
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

    def test_basic_planning_one_step(self):
        planner = CBSPlanner()
        planner.env = env = self.get_test_env()
        print_grid(env)

        actions = planner.plan(None)
        print(actions)
        print([Action(a) for a in actions])  # convert actions (ints) to enum values before printing -> more readable
        env = update_env(env, actions)
        print_grid(env)

        self.assertEqual(Action(actions[0]), Action.CCR)
