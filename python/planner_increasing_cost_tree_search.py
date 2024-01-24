from models import BasePlanner, Action
from util import forward, counter_clockwise_rotate, clockwise_rotate, reverse, DistanceMap
from typing import Optional
#from multiprocessing import Pool
#import concurrent.futures
import time

CROSSPRODUCT_SINGLE_PATH = True
TOTAL_MDD_WARNING_TIME = 0.01
SIGNLE_MDD_WARNING_TIME = 0.004
CROSSPRODUCT_WARNING_TIME = 0.075
GENERATE_NEW_ICTS_NODES_WARNING_TIME = 0.01

class IncreasingCostTreeSearchPlanner(BasePlanner):

    def __init__(self, pyenv=None):
        super().__init__(pyenv, "ICTS-Planner")

    def initialize(self, preprocess_time_limit: int):
        self.current_paths = None
        self.init_distance_maps()
        self.init_old_mdd_cache()
        self.init_new_MDD_Cache()
        return True

    def plan(self, time_limit: int) -> list[int]:
        if self.current_paths == None or self.current_paths.subnodes == []:
            self.current_paths = self.increasing_cost_tree_search()
        print(self.current_paths.subnodes[0])
        (next_path, actions) = self.current_paths.subnodes[0]
        self.current_paths = next_path
        #self.update_old_mdd_cache(actions)
        print(actions)
        return actions

    # Distance Maps [goal][current_pos][current_dir]
    def init_distance_maps(self):
        print("Initialize array of distance maps")
        self.distance_maps = [None]*self.env.rows*self.env.cols

    '''def access_distance_maps(self, start_pos: int, start_dir: int, goal: int) -> int:
        if self.distance_maps[goal] == None:
            print("Calculate Distance map for", goal)
            self.distance_maps[goal] = [None]*self.env.rows*self.env.cols
            for i in range(len(self.distance_maps[goal])):
                self.distance_maps[goal][i] = [None]*4

            pos_dir_set = [(goal, 0, 0), (goal, 1, 0), (goal, 2, 0), (goal, 3, 0)]
            for pos, dir, costs in pos_dir_set:
                if self.distance_maps[goal][pos][dir] == None:
                    self.distance_maps[goal][pos][dir] = costs
                    backward = forward(self.env, pos, reverse(dir))
                    if backward >= 0 and (backward, dir) not in [(x[0], x[1]) for x in pos_dir_set]:
                        pos_dir_set.append((backward, dir, costs + 1))
                    ccr = counter_clockwise_rotate(dir)
                    if (pos, ccr) not in [(x[0], x[1]) for x in pos_dir_set]:
                        pos_dir_set.append((pos, ccr, costs + 1))
                    cr = clockwise_rotate(dir)
                    if (pos, cr) not in [(x[0], x[1]) for x in pos_dir_set]:
                        pos_dir_set.append((pos, cr, costs + 1))
        return self.distance_maps[goal][start_pos][start_dir]'''

    def access_distance_maps(self, start_pos: int, start_dir: int, goal: int) -> int:
        if self.distance_maps[goal] == None:
            print("Calculate Distance map for", goal)
            self.distance_maps[goal] = DistanceMap(goal, self.env)
        return self.distance_maps[goal].get_distance(self.env, start_pos, start_dir)

    def increasing_cost_tree_search(self):
        
        starttime = time.perf_counter()
        
        costs_of_root_node = []
        for index_of_agent in range(self.env.num_of_agents):
            print("Calculating ideal costs of Agent", index_of_agent)
            costs_of_root_node.append(self.access_distance_maps(
                start_pos = self.env.curr_states[index_of_agent].location,
                start_dir = self.env.curr_states[index_of_agent].orientation,
                goal = self.env.goal_locations[index_of_agent][0][0]
            ))
            #costs_of_root_node.append(0)
        
        '''all_start_pos = [a.location for a in self.env.curr_states]
        all_start_dir = [a.orientation for a in self.env.curr_states]
        all_goals = [a[0][0] for a in self.env.goal_locations]
        costs_of_root_node = list(map(self.access_distance_maps, all_start_pos, all_start_dir, all_goals))'''

        '''costs_of_root_node = []
        all_start_pos = [a.location for a in self.env.curr_states]
        all_start_dir = [a.orientation for a in self.env.curr_states]
        all_goals = [a[0][0] for a in self.env.goal_locations]
        with Pool(self.env.num_of_agents) as p:
            print("Calculating ideal costs of all Agents")
            costs_of_root_node = p.map(self.access_distance_maps, ([all_start_pos], [all_start_dir], [all_goals]))'''

        '''costs_of_root_node = []
        all_start_pos = [a.location for a in self.env.curr_states]
        all_start_dir = [a.orientation for a in self.env.curr_states]
        all_goals = [a[0][0] for a in self.env.goal_locations]
        with concurrent.futures.ThreadPoolExecutor(self.env.num_of_agents) as executor:
            print("Calculating ideal costs of all Agents")
            results = executor.map(self.access_distance_maps, [all_start_pos], [all_start_dir], [all_goals])
        
        for r in concurrent.futures.as_completed(results):
            try:
                data = r.result()
            except Exception as exc:
                print("Error!")
        print(data)
        costs_of_root_node = data'''

        endtime = time.perf_counter()
        print((endtime-starttime))
        
        nodes = [costs_of_root_node]
        paths = []
        #ICTS_combinations = {}
        #startime = 0
        for costs in nodes:
            #costs_dont_contain_bad_combination = True
            '''min_costs = min([cost for cost in costs])
            for z1 in range(len(costs) - 1):
                for z2 in range(z1 + 1, len(costs)):
                    if (z1, z2, min_costs) not in ICTS_combinations:
                        mdd_z1 = self.access_new_MDD_cache(self.env.curr_states[z1].location, self.env.curr_states[z1].orientation, self.env.goal_locations[z1][0][0], costs[z1])
                        mdd_z2 = self.access_new_MDD_cache(self.env.curr_states[z2].location, self.env.curr_states[z2].orientation, self.env.goal_locations[z2][0][0], costs[z2])
                        crossproduct = self.New_MDD_Crossproduct_Node([mdd_z1, mdd_z2], min_costs)      # Kreuzprodukt berechnet nur bis zu der Tiefe vom minimalen Element aus der Liste an Agent-Kosten
                        ICTS_combinations[(z1, z2, min_costs)] = crossproduct.evaluate()
                    if not ICTS_combinations[(z1, z2, min_costs)]:
                        print("Known bad ICTS combination!", z1, z2, min_costs, end=' ')
                        costs_dont_contain_bad_combination = False
                        break
                else:
                    # Continue if the inner loop wasn't broken.
                    continue
                # Inner loop was broken, break the outer.
                break'''
            '''endtime = time.perf_counter()
            print(endtime-starttime)'''

            paths = self.get_all_paths_for_exact_costs(costs)

            '''if costs_dont_contain_bad_combination:
                starttime2 = time.perf_counter()
                paths = self.get_all_paths_for_exact_costs(costs)
                endtime2 = time.perf_counter()
                print(endtime2-starttime2)
            else:
                print("Skip:", costs)'''
            
            if paths == []:
                # Generate new costs and add them to `nodes`
                # TODO: Find more efficient way to generate new ICTS nodes (one that doesn't try to create duplicates)
                starttime = time.perf_counter()
                for i in range(len(costs)):
                    new_costs = costs.copy()
                    new_costs[i] += 1

                    if new_costs not in nodes:
                        nodes.append(new_costs)
                    #    print("Created new costs", new_costs)
                    #else:
                    #    print("Costs already exist in ICTS", new_costs)
                endtime = time.perf_counter()
                if (endtime-starttime) > GENERATE_NEW_ICTS_NODES_WARNING_TIME:
                    print("Generating new ICTS-nodes took more than", GENERATE_NEW_ICTS_NODES_WARNING_TIME, "second(s)!", (endtime-starttime))
            else:
                nodes.clear()       # All nodes get removed since we no longer need them. This also ends our for-loop.
        
        print("ITCS finished!", paths.to_string_single_path())
        return paths

    # DON'T USE: use access_distance_maps instead
    def calculate_shortest_path_length_ignoring_other_agents(self, start_pos: int, start_dir: int, goal: int) -> int:
        pos_sets = [(start_pos, start_dir, 0)]
        for *pos, costs in pos_sets:
            if pos[0] == goal:
                return costs
            else:
                #print(pos[0], pos[1], goal)
                forward = forward(self.env, pos[0], pos[1])
                if forward >= 0 and (forward, pos[1]) not in [(x[0], x[1]) for x in pos_sets]:
                    pos_sets.append((forward, pos[1], costs + 1))
                ccr = counter_clockwise_rotate(pos[1])
                if (pos[0], ccr) not in [(x[0], x[1]) for x in pos_sets]:
                    pos_sets.append((pos[0], ccr, costs + 1))
                cr = clockwise_rotate(pos[1])
                if (pos[0], cr) not in [(x[0], x[1]) for x in pos_sets]:
                    pos_sets.append((pos[0], cr, costs + 1))
        print("WARNING! Ideal path could not be calculated! Falling back onto Manhatten Distance")
        import time
        time.sleep(3)
        return self.getManhattanDistance(start_pos, goal)
    
    def get_all_paths_for_exact_costs(self, costs: list[int]):
        print("Get paths for costs:", costs)
        mdds = []
        starttime = time.perf_counter()
        for i in range(len(costs)):
            '''mdd = self.access_old_mdd_cache(i, costs[i])
            if not mdd.evaluate():  # sollte im Idealfall (perfekte Schätzung der unteren Kosten) nicht auftreten
                break'''
            
            starttime2 = time.perf_counter()
            mdd = self.access_new_MDD_cache(self.env.curr_states[i].location, self.env.curr_states[i].orientation, self.env.goal_locations[i][0][0], costs[i])
            endtime2 = time.perf_counter()
            if (endtime2-starttime2) > SIGNLE_MDD_WARNING_TIME:
                print("Single MDD took longer than", SIGNLE_MDD_WARNING_TIME, "second(s)!", (endtime2-starttime2))
            if mdd == None:
                break
            mdds.append(mdd)
        endtime = time.perf_counter()
        if (endtime-starttime) > TOTAL_MDD_WARNING_TIME:
            print("Calculating all MDDs took more than", TOTAL_MDD_WARNING_TIME, "second(s)!", (endtime-starttime))
        
        # sinlose mdds um die laufzeit von crossproduct besser optimieren zu können
        # TODO: später entfernen
        '''for i in range(32):
            if self.env.map[i] == 0:
                mdds.append(self.access_new_MDD_cache(i, 0, 400, self.access_distance_maps(i, 0, 400)))'''

        starttime = time.perf_counter()
        #crossproduct = self.Old_MDD_Crossproduct_Node(mdds, None, 0)
        crossproduct = self.New_MDD_Crossproduct_Node(mdds, None)
        endtime = time.perf_counter()
        if (endtime-starttime) > CROSSPRODUCT_WARNING_TIME:
            print("Calculating Crossproduct took more than", CROSSPRODUCT_WARNING_TIME, "second(s)!", (endtime-starttime))
        if crossproduct.evaluate():
            return crossproduct
        else:
            return []

    # Old MDD Cache [agent][length]
    def init_old_mdd_cache(self):
        print("Initialize mdd cache")
        self.mdd_cache = [None]*self.env.num_of_agents
    
    def access_old_mdd_cache(self, agent: int, costs: int):
        if self.mdd_cache[agent] == None:
            self.mdd_cache[agent] = []
        while len(self.mdd_cache[agent]) - 1 < costs:
            self.mdd_cache[agent].append(None)
        if self.mdd_cache[agent][costs] == None:
            import time
            print("Generate MDD for agent", agent, "for costs", costs, "(", self.access_distance_maps(
                self.env.curr_states[agent].location,
                self.env.curr_states[agent].orientation,
                self.env.goal_locations[agent][0][0]
            ) , ")", end='\t')
            starttime = time.perf_counter()
            self.mdd_cache[agent][costs] = self.Old_MDD_Node(self, self.env.curr_states[agent].location, self.env.curr_states[agent].orientation, self.env.goal_locations[agent][0][0], costs, 0, None)
            endtime = time.perf_counter()
            print("Time:", (endtime-starttime))
        return self.mdd_cache[agent][costs]

    def update_old_mdd_cache(self, paths_actions):
        '''print("Update MDD Cache")
        prev_mdd_cache = self.mdd_cache.copy()'''
        self.init_old_mdd_cache()
        '''for a in range(len(paths_actions)):
            for i in range(1, len(prev_mdd_cache[a])):
                if prev_mdd_cache[a][i] != None:
                    temp = next((x for x in prev_mdd_cache[a][i].subnodes if x.prev_action == paths_actions[a]), None)
                    #print("Agent", a, i, prev_mdd_cache[a][i].to_string(), temp.to_string())
                    if temp != None:
                        if self.mdd_cache[a] == None:
                            self.mdd_cache[a] = []
                        while len(self.mdd_cache[a]) < i:
                            self.mdd_cache[a].append(None)
                        self.mdd_cache[a][i-1] = temp'''

    class Old_MDD_Node:
        def __init__(self, other, current_pos: int, current_dir: int, goal: int, max_depth: int, current_depth: int, prev_action: Optional[int]):
            self.current_pos = current_pos
            self.current_dir = current_dir
            self.prev_action = prev_action
            self.max_depth = max_depth
            self.goal = goal
            self.subnodes = []
            
            if (current_depth + IncreasingCostTreeSearchPlanner.access_distance_maps(other, current_pos, current_dir, goal)) <= max_depth and current_depth < max_depth:
                '''with Pool(4) as p:
                    new_nodes = p.starmap(IncreasingCostTreeSearchPlanner.Old_MDD_Node, [
                        (other, current_pos, current_dir, goal, max_depth, (current_depth + 1), Action.W.value),
                        (other, forward(other.env, current_pos, current_dir), current_dir, goal, max_depth, (current_depth + 1), Action.FW.value),
                        (other, current_pos, counter_clockwise_rotate(current_dir), goal, max_depth, (current_depth + 1), Action.CCR.value),
                        (other, current_pos, clockwise_rotate(current_dir), goal, max_depth, (current_depth + 1), Action.CR.value)
                    ])'''
                
                temp = IncreasingCostTreeSearchPlanner.Old_MDD_Node(other, current_pos, current_dir, goal, max_depth, (current_depth + 1), Action.W.value) # wait
                if temp.evaluate():
                    self.subnodes.append(temp)

                forward_pos = forward(other.env, current_pos, current_dir)
                if forward_pos >= 0:
                    temp = IncreasingCostTreeSearchPlanner.Old_MDD_Node(other, forward_pos, current_dir, goal, max_depth, (current_depth + 1), Action.FW.value) # Forward
                    if temp.evaluate():
                        self.subnodes.append(temp)
                
                temp = IncreasingCostTreeSearchPlanner.Old_MDD_Node(other, current_pos, counter_clockwise_rotate(current_dir), goal, max_depth, (current_depth + 1), Action.CCR.value) # CCR
                if temp.evaluate():
                    self.subnodes.append(temp)

                temp = IncreasingCostTreeSearchPlanner.Old_MDD_Node(other, current_pos, clockwise_rotate(current_dir), goal, max_depth, (current_depth + 1), Action.CR.value) # CR
                if temp.evaluate():
                    self.subnodes.append(temp)

        def evaluate(self) -> bool: # Returns if Node leads to goal
            if self.current_pos == self.goal: # TODO: current_depth == max_depth ???
                return True
            else:
                for subnode in self.subnodes:
                    if subnode.evaluate():
                        return True
            return False
        
        def to_string(self) -> str:
            string = str(self.current_pos) + " " + str(self.current_dir) + " ["
            for subnode in self.subnodes:
                string += subnode.to_string()
            string += "]"
            return string

    class Old_MDD_Crossproduct_Node:
        def __init__(self, mdds, max_depth: Optional[int], current_depth: int):
            self.current_pos = []
            self.current_dir = []
            self.prev_action = []
            self.subnodes = []
            self.max_depth = max_depth
            self.current_depth = current_depth

            i = []
            max_amount_of_subnodes = 1

            for mdd in mdds:
                self.current_pos.append(mdd.current_pos)
                self.current_dir.append(mdd.current_dir)
                self.prev_action.append(mdd.prev_action)
                if self.max_depth == None or self.max_depth > mdd.max_depth:
                    self.max_depth = mdd.max_depth
                i.append(0)
                max_amount_of_subnodes *= len(mdd.subnodes)

            for j in range(max_amount_of_subnodes):
                #print(self.current_depth, self.max_depth, j, max_amount_of_subnodes)
                j2 = j
                temp = max_amount_of_subnodes
                sub_MDD_Nodes = []
                for k in range(len(mdds)):
                    temp /= len(mdds[k].subnodes)
                    #print(k, (j2//temp), temp, j2)
                    sub_MDD_Nodes.append(mdds[k].subnodes[int(j2//temp)])
                    j2 -= (j2//temp) * temp

                isValidSubnode = True
                for k1 in range(len(mdds) - 1):
                    for k2 in range(k1 + 1, len(mdds)):
                        if k1 != k2 and isValidSubnode:
                            if self.current_pos[k2] == sub_MDD_Nodes[k1].current_pos and self.current_pos[k1] == sub_MDD_Nodes[k2].current_pos:
                                isValidSubnode = False
                                #print("Conflict: pos switch")
                            elif sub_MDD_Nodes[k1].current_pos == sub_MDD_Nodes[k2].current_pos:
                                isValidSubnode = False
                                #print("Conflict: same pos MDD", k1, ":", sub_MDD_Nodes[k1].to_string(), "MDD", k2, ":", sub_MDD_Nodes[k2].to_string())
                if isValidSubnode:
                    new_subnode = IncreasingCostTreeSearchPlanner.Old_MDD_Crossproduct_Node(sub_MDD_Nodes, self.max_depth, self.current_depth + 1)
                    if new_subnode.evaluate():
                        self.subnodes.append(new_subnode)
                        if CROSSPRODUCT_SINGLE_PATH:
                            break
            #print("Anzahl subnodes", len(self.subnodes))

        def evaluate(self) -> bool:
            if self.current_depth == self.max_depth:
                return True
            else:
                for subnode in self.subnodes:
                    if subnode.evaluate():
                        return True
            return False
        
        def to_string(self) -> str:
            string = str(self.current_pos) + " " + str(self.current_dir) + " <"
            for subnode in self.subnodes:
                string += subnode.to_string()
            string += ">"
            return string

        def to_string_single_path(self) -> str:
            string = str(self.current_pos) + " " + str(self.current_dir) + " <"
            if len(self.subnodes) > 0:
                string += self.subnodes[0].to_string_single_path()
            string += ">"
            return string

    # New MDD Cache [pos][dir][goal][steps_left]
    def init_new_MDD_Cache(self):
        self.new_MDD_Cache = [None]*self.env.rows*self.env.cols

        for i in range(len(self.new_MDD_Cache)):
            self.new_MDD_Cache[i] = [None]*4
            for j in range(len(self.new_MDD_Cache[i])):
                self.new_MDD_Cache[i][j] = [None]*self.env.rows*self.env.cols
                for k in range(len(self.new_MDD_Cache[i][j])):
                    self.new_MDD_Cache[i][j][k] = []

    def access_new_MDD_cache(self, start_pos: int, start_dir: int, goal: int, steps_left: int):
        while len(self.new_MDD_Cache[start_pos][start_dir][goal]) <= steps_left:            # 0 steps ist unnötig
            self.new_MDD_Cache[start_pos][start_dir][goal].append(None)
        if self.new_MDD_Cache[start_pos][start_dir][goal][steps_left] == None:
            self.New_MDD_Node(self, start_pos, start_dir, goal, steps_left)
        if self.new_MDD_Cache[start_pos][start_dir][goal][steps_left] == False and self.new_MDD_Cache[start_pos][start_dir][goal][steps_left] == None:
            self.new_MDD_Cache[start_pos][start_dir][goal][steps_left] = False
            return None
        else:
            return self.new_MDD_Cache[start_pos][start_dir][goal][steps_left]

    class New_MDD_Node:
        def __init__(self, other, current_pos: int, current_dir: int, goal: int, steps_left: int):
            self.current_pos = current_pos
            self.current_dir = current_dir
            self.goal = goal
            self.steps_left = steps_left
            self.subnodes = []
            
            if (IncreasingCostTreeSearchPlanner.access_distance_maps(other, current_pos, current_dir, goal)) <= steps_left and steps_left > 0 :
                temp = other.access_new_MDD_cache(current_pos, current_dir, goal, (steps_left - 1)) # wait
                if temp != None:
                    self.subnodes.append((temp, Action.W.value))

                forward_pos = forward(other.env, current_pos, current_dir)
                if forward_pos >= 0:
                    temp = other.access_new_MDD_cache(forward_pos, current_dir, goal, (steps_left - 1)) # Forward
                    if temp != None:
                        self.subnodes.append((temp, Action.FW.value))

                temp = other.access_new_MDD_cache(current_pos, counter_clockwise_rotate(current_dir), goal, (steps_left - 1)) # CCR
                if temp != None:
                    self.subnodes.append((temp, Action.CCR.value))

                temp = other.access_new_MDD_cache(current_pos, clockwise_rotate(current_dir), goal, (steps_left - 1)) # CR
                if temp != None:
                    self.subnodes.append((temp, Action.CR.value))
            
            if self.evaluate():
                other.new_MDD_Cache[current_pos][current_dir][goal][steps_left] = self
        
        def evaluate(self) -> bool: # Returns if Node leads to goal
            if self.current_pos == self.goal: # TODO: current_depth == max_depth ???
                return True
            else:
                for (subnode, _) in self.subnodes:
                    if subnode.evaluate():
                        return True
            return False

        def to_string(self) -> str:
            string = str(self.current_pos) + " " + str(self.current_dir) + " ["
            for subnode in self.subnodes:
                string += subnode.to_string()
            string += "]"
            return string

    class New_MDD_Crossproduct_Node:
        def __init__(self, mdds, steps_left: Optional[int]):
            self.current_pos = []
            self.current_dir = []
            self.subnodes = []
            if steps_left == None:
                self.steps_left = min([mdd.steps_left for mdd in mdds])
            else:
                self.steps_left = steps_left

            max_amount_of_subnodes = 1

            for mdd in mdds:
                self.current_pos.append(mdd.current_pos)
                self.current_dir.append(mdd.current_dir)
                max_amount_of_subnodes *= len(mdd.subnodes)
            
            global_timer = 0.0

            #bad_combinations_old = []
            bad_combinations = {}
            
            for i in range(max_amount_of_subnodes):
                #starttime = time.perf_counter()
                isValidSubnode = True
                remainder = i
                divisor = max_amount_of_subnodes
                sub_MDD_Nodes = []
                index_of_picked_subnodes = []

                for j1 in range(len(mdds)):
                    divisor /= len(mdds[j1].subnodes)
                    #print(i, (remainder//divisor), divisor, remainder)
                    (quotient, remainder) = map(int, divmod(remainder, divisor))
                    sub_MDD_Nodes.append(mdds[j1].subnodes[quotient])
                    index_of_picked_subnodes.append(quotient)
                    
                    bad_combinations_for_j1 = []
                    if (j1, index_of_picked_subnodes[j1]) in bad_combinations:
                        bad_combinations_for_j1 = bad_combinations[(j1, index_of_picked_subnodes[j1])]

                    for j2 in range(j1):
                        if (j2, index_of_picked_subnodes[j2]) in bad_combinations_for_j1:
                            isValidSubnode = False
                            #print("Bad combination")
                            break
                        elif self.current_pos[j1] == sub_MDD_Nodes[j2][0].current_pos and self.current_pos[j2] == sub_MDD_Nodes[j1][0].current_pos:
                            isValidSubnode = False
                            #bad_combinations_old.append((j1, index_of_picked_subnodes[j1], j2, index_of_picked_subnodes[j2]))
                            if (j1, index_of_picked_subnodes[j1]) not in bad_combinations:
                                bad_combinations[(j1, index_of_picked_subnodes[j1])] = []
                            bad_combinations[(j1, index_of_picked_subnodes[j1])].append((j2, index_of_picked_subnodes[j2]))
                            #print("Conflict: pos switch")
                            break
                        elif sub_MDD_Nodes[j1][0].current_pos == sub_MDD_Nodes[j2][0].current_pos:
                            isValidSubnode = False
                            #bad_combinations_old.append((j1, index_of_picked_subnodes[j1], j2, index_of_picked_subnodes[j2]))
                            if (j1, index_of_picked_subnodes[j1]) not in bad_combinations:
                                bad_combinations[(j1, index_of_picked_subnodes[j1])] = []
                            bad_combinations[(j1, index_of_picked_subnodes[j1])].append((j2, index_of_picked_subnodes[j2]))
                            #print("Conflict: same pos MDD", k1, ":", sub_MDD_Nodes[k1][0].to_string(), "MDD", k2, ":", sub_MDD_Nodes[k2][0].to_string())
                            break
                            
                    else:
                        # Continue if the inner loop wasn't broken.
                        continue
                    # Inner loop was broken, break the outer.
                    break
                   
                '''endtime = time.perf_counter()
                global_timer += endtime - starttime
                print("Timer:", global_timer)'''

                if isValidSubnode:
                    z = list(map(list, zip(*sub_MDD_Nodes)))
                    subnodes = z[0]
                    actions = z[1]

                    new_subnode = IncreasingCostTreeSearchPlanner.New_MDD_Crossproduct_Node(subnodes, (self.steps_left - 1))
                    if new_subnode.evaluate():
                        self.subnodes.append((new_subnode, actions))
                        if CROSSPRODUCT_SINGLE_PATH:
                            break
            '''if steps_left == None:
                print(max_amount_of_subnodes, self.steps_left, end=' ')
                print("Anzahl subnodes", len(self.subnodes))'''

        def evaluate(self) -> bool:
            if self.steps_left == 0:
                return True
            else:
                for subnode in self.subnodes:
                    if subnode[0].evaluate():
                        return True
            return False
        
        def to_string(self) -> str:
            string = str(self.current_pos) + " " + str(self.current_dir) + " <"
            for subnode in self.subnodes:
                string += subnode[0].to_string()
            string += ">"
            return string

        def to_string_single_path(self) -> str:
            string = str(self.current_pos) + " " + str(self.current_dir) + " <"
            if len(self.subnodes) > 0:
                string += self.subnodes[0][0].to_string_single_path()
            string += ">"
            return string
