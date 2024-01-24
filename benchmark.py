import subprocess
import json
import os
import time
import shutil
import argparse
from typing import Optional
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import itertools
import ast

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "Output")

# function to create the full path for files
def create_filepath(filename):
    return os.path.join(os.path.dirname(__file__), filename)


# compilation
def compile_code():
    compile_cmd = "sh compile.sh"
    subprocess.run(compile_cmd, shell=True, check=True)


def run_planviz(cmd):
    subprocess.run(cmd, shell=True, check=True)


def read_configs(args):
    # give default values in case nothing is specified for a configuration
    try:
        # Ensure the config string is a proper dictionary format
        # Use ast.literal_eval for safe evaluation of the string
        config_dict = ast.literal_eval(args.config)
        if not isinstance(config_dict, dict):
            raise ValueError("Config input is not a valid dictionary.")
        return config_dict
    except (SyntaxError, ValueError) as e:
        raise ValueError(f"Invalid config string: {e}")


def combine_config_options(args):
    if args.config is None:
        return [{}]
    configs = read_configs(args)  # assuming this returns a dictionary of configurations
    keys, values = zip(*configs.items())

    config_combinations = [dict(zip(keys, v)) for v in itertools.product(*values)]

    # add runs to the config, the reason is so that the script plots graphs for each run
    runs_combinations = range(1, args.reruns + 1)  # Corrected to use args.reruns

    # Cross product of configurations and runs
    result = []
    for run in runs_combinations:
        for config in config_combinations:
            config['run'] = run
            result.append(dict(config))  # Create a new dictionary for each run

    return result


def set_env_var(dct: dict):
    for key, value in dct.items():
        os.environ[key] = str(value)


# Function to get the cumulative count of finished tasks
def get_cumulative_finished(data):
    finished_counts = defaultdict(int)
    for event_group in data:
        for event in event_group:
            # 'event' is a list within a list, extract the inner list
            for task_id, time_step, event_type in event:
                if event_type == "finished":
                    finished_counts[time_step] += 1

    max_time_step = max(finished_counts.keys(), default=0)
    cumulative_finished = {time_step: sum(finished_counts[key] for key in finished_counts if key <= time_step) for
                           time_step in range(1, max_time_step + 1)}
    return cumulative_finished


def plot_line_range(data: dict[str, list[list[int]]], output_path: str):
    plt.figure(figsize=(10, 6))
    for config, runs in data.items():
        print(config)
        runs_array = np.array(runs)
        mean_values = np.mean(runs_array, axis=0)
        min_values = np.min(runs_array, axis=0)
        max_values = np.max(runs_array, axis=0)

        # plt the average line
        plt.plot(range(len(mean_values)), mean_values, label=config)

        # fill in the space between min and max
        plt.fill_between(range(len(mean_values)), min_values, max_values, alpha=0.2)

    # description
    plt.xlabel("Timestep")
    plt.ylabel("Tasks Completed")
    plt.title("Throughput at step x")
    plt.legend()

    # X,Y Axis Whole Numbers
    plt.xticks(np.arange(0, len(mean_values), step=5))
    plt.yticks(np.arange(0, np.max(max_values) + 1, step=5))

    # save as picture
    plt.savefig(output_path)


def plot_results(run_dir, configs):
    filename_tasks_finished = create_filepath(f"{run_dir}/tasks_finished.json")
    with open(filename_tasks_finished, "r") as file:
        tasks_finished = json.load(file)
    plot_line_range(tasks_finished, f"{run_dir}/tasks_finished.png")


# executing algorithm for each map
def run_iteration(input_file, iterations=None):
    map_file_location = "./example_problems/" + input_file
    run_cmd = create_filepath(f"build/lifelong --inputFile {map_file_location} -o test.json")

    if iterations:
        run_cmd += f" --simulationTime {iterations}"
    print(run_cmd)
    # start execution time
    start_time = time.time()
    subprocess.run(run_cmd, shell=True, check=True)
    execution_time = time.time() - start_time
    # end execution time

    # get the output file and read data
    with open(create_filepath("test.json"), "r") as output_file:
        output_data = json.load(output_file)

    # get number of tasks finished
    num_tasks_finished = output_data.get("numTaskFinished", "Tasks not found")
    # get the events of robots (assigned task, finished task)
    events = output_data.get("events", "Events not found")

    return num_tasks_finished, execution_time, events


def generate_config_str(config):
    # Exclude 'run' key from the configuration dictionary
    return ", ".join(f"{key}: {str(value)}" for key, value in config.items() if key != "run")


def generate_filename():
    return time.strftime("%Y%m%d_%H%M%S")


def run_code(input_file, output_file_folder, args):
    # get number of iterations if specified
    iterations = args.iterations

    # get all configs, if a parameter is not specfified, the list will be empty
    configs = combine_config_options(args)
    tasks_finished = {}
    # create a list to store results
    results = []
    # run code for each map and configuration
    run_dir_path = os.path.join(OUTPUT_DIR, output_file_folder)
    os.makedirs(create_filepath(run_dir_path), exist_ok=True)

    for config in configs:
        # save configurations as env variables
        set_env_var(config)

        num_task_finished, execution_time, events = run_iteration(input_file, iterations)
        result_entry = {"file": input_file,
                        "timesteps_taken": iterations,
                        "tasks_finished": num_task_finished,
                        "execution_time": execution_time,
                        "events": events
                        }

        result_entry.update(config)
        results.append(result_entry)

        # update dict for when same config called multiple times
        mul_key, mul_value = generate_config_str(config), get_cumulative_finished([events])
        if mul_key in tasks_finished:
            tasks_finished[mul_key].append(list(mul_value.values()))
        else:
            tasks_finished[mul_key] = [list(mul_value.values())]

    # save the detailed file
    file_name = create_filepath(f"{run_dir_path}/{output_file_folder}.json")
    with open(file_name, "a") as output_file:
        json.dump(results, output_file, indent=4)

    # save the number of tasks finished file
    file_name = create_filepath(f"{run_dir_path}/tasks_finished.json")
    with open(file_name, "a") as output_file:
        json.dump(tasks_finished, output_file, indent=4)

    # plot the charts
    plot_results(run_dir_path, configs)


def get_total_tasks_finished(file_name):
    with open(file_name, "r") as file:
        data = json.load(file)

    total_tasks_finished = sum(entry.get("tasks_finished", 0) for entry in data)
    return total_tasks_finished


def compare_and_update_best_benchmark(file_name, iterations=None):
    # if the best benchmark file does not exist, create it
    if not os.path.isfile("best_benchmark.json"):
        shutil.copy(file_name, "best_benchmark.json")
        return

    # get the current number of tasks finished
    current_benchmark_tasks_finished = get_total_tasks_finished(file_name)
    best_benchmark_tasks_finished = get_total_tasks_finished("best_benchmark.json")

    if current_benchmark_tasks_finished > best_benchmark_tasks_finished:
        shutil.copy(file_name, "best_benchmark.json")
        print("Better benchmark found!")


def main(args):
    # check if user wants to rebuild lifelong file
    if args.rebuild:
        # compilation
        compile_code()

    set_env_var({"PLANNER": args.planner})

    timestamp_filename = generate_filename()
    output_file_folder = f"benchmark_{timestamp_filename}"

    # execution
    run_code(args.file, output_file_folder, args)

    # run planviz if specified
    if args.viz:
        run_planviz(args.viz)

    # check if we found a better algorithm
    # do this only when the iterations are not specified
    # this means that the code is fully running instead of testing it out with a limited number of steps
    if not args.iterations:
        file_name = f"Output/{output_file_folder}/{output_file_folder}.json"
        compare_and_update_best_benchmark(file_name)


if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--rebuild", action="store_true", help="Use when you want to rebuild the program")

    argParser.add_argument("--planner", type=str, default="astar", help="name of the desired planner")
    argParser.add_argument("--iterations", type=int, nargs="?", default=None,
                           help="Specify the number of iterations(steps)")
    argParser.add_argument("--viz", type=str, nargs="?",
                           const="python3 ../../PlanViz/script/plan_viz.py --map "
                                 "../example_problems/random.domain/maps/random-32-32-20.map --plan ./test.json "
                                 "--grid --aid --static --ca",
                           help="Specify the amount of times to run one configuration steps")
    argParser.add_argument("--reruns", type=int, nargs="?", default=1,
                           help="Specify the amount of times to run one configuration")

    argParser.add_argument("--config", type=str, default=None,
                           help='Configuration for algorithm, heuristic, timeHorizon, replanningPeriod etc.')

    argParser.add_argument("--file", type=str, default="random.domain/random_20.json",
                           help="Location of the map to run the algorithm on (e.g. random.domain/random_20.json)")

    args = argParser.parse_args()

    main(args)
