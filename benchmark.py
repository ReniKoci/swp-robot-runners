import subprocess
import json
import os
import time
import shutil
import argparse
from typing import Optional

import matplotlib.pyplot as plt
from collections import defaultdict
import seaborn as sns
import pandas as pd
import itertools
import ast


# compilation
def compile_code():
    compile_cmd = "sh compile.sh"
    subprocess.run(compile_cmd, shell=True, check=True)


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
    return [dict(zip(keys, v)) for v in itertools.product(*values)]


def set_env_var(dct: dict):
    for key, value in dct.items():
        os.environ[key] = str(value)


def plot_results(folder, configs):
    json_file = f"Output/{folder}/{folder}.json"
    output_dir = f"Output/{folder}"
    with open(json_file, "r") as file:
        data = json.load(file)

    # Function to get the cumulative count of finished tasks
    def get_cumulative_finished(data):
        finished_counts = defaultdict(int)
        for event_group in data:
            for event in event_group:
                # 'event' is a list within a list, extract the inner list
                for task_id, time_step, event_type in event:
                    if event_type == "finished":
                        finished_counts[time_step] += 1
        cumulative = 0
        cumulative_finished = {}
        for time_step in sorted(finished_counts):
            cumulative += finished_counts[time_step]
            cumulative_finished[time_step] = cumulative
        return cumulative_finished

    # Plotting function
    def plot_data(cumulative_finished, title, file_suffix):
        plt.figure()
        time_steps = list(cumulative_finished.keys())
        finished_tasks = list(cumulative_finished.values())
        plt.plot(time_steps, finished_tasks, marker='o')
        plt.title(title)
        plt.xlabel('Time Step')
        plt.ylabel('Cumulative Finished Tasks')
        plt.grid(True)
        plt.savefig(f"{output_dir}/plot_{file_suffix}.png", bbox_inches='tight')
        plt.close()

    for config in configs:
        config_key = tuple(config.items())
        filtered_data = [entry['events'] for entry in data if all(entry[k] == v for k, v in config.items())]
        cumulative_finished = get_cumulative_finished(filtered_data)
        plot_data(cumulative_finished, f'Finished Tasks over Time for Config {config_key}', f'combined_{config_key}')


# executing algorithm for each map
def run_iteration(input_file, iterations=None):
    map_file_location = "./example_problems/" + input_file

    run_cmd = f"./build/lifelong --inputFile {map_file_location} -o test.json"

    if iterations:
        run_cmd += f" --simulationTime {iterations}"

    # start execution time
    start_time = time.time()
    subprocess.run(run_cmd, shell=True, check=True)
    execution_time = time.time() - start_time
    # end execution time

    # get the output file and read data
    with open("test.json", "r") as output_file:
        output_data = json.load(output_file)

    # get number of tasks finished
    num_tasks_finished = output_data.get("numTaskFinished", "Tasks not found")
    # get the events of robots (assigned task, finished task)
    events = output_data.get("events", "Events not found")

    return num_tasks_finished, execution_time, events


def generate_filename():
    return time.strftime("%Y%m%d_%H%M%S")


def run_code(input_files, output_file_folder, args):
    # get number of iterations if specified
    iterations = args.iterations

    # get all configs, if a parameter is not specfified, the list will be empty
    configs = combine_config_options(args)

    # create a list to store results
    results = []

    # run code for each map and configuration
    for config in configs:
        # save configurations as env variables
        set_env_var(config)
        for input_file in input_files:
            num_task_finished, execution_time, events = run_iteration(input_file, iterations)
            result_entry = {"file": input_file,
                            "timesteps_taken": iterations,
                            "tasks_finished": num_task_finished,
                            "execution_time": execution_time,
                            "events": events
                            }
            result_entry.update(config)
            results.append(result_entry)

    # create the directory if it doesn't exist
    os.makedirs("Output", exist_ok=True)
    os.makedirs(f"Output/{output_file_folder}", exist_ok=True)

    file_name = f"Output/{output_file_folder}/{output_file_folder}.json"
    with open(file_name, "a") as output_file:
        json.dump(results, output_file, indent=4)

    # plot the charts
    plot_results(output_file_folder, configs)


def get_total_tasks_finished(file_name):
    with open(file_name, "r") as file:
        data = json.load(file)

    total_tasks_finished = sum(entry.get("tasks_finished", 0) for entry in data)
    return total_tasks_finished


def get_timesteps_taken(file_name):
    with open(file_name, "r") as file:
        data = json.load(file)

    timesteps_taken = sum(entry.get("timesteps_taken", 0) for entry in data)
    return timesteps_taken


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


def get_timesteps():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulationTime", type=int)
    args = parser.parse_args()
    return args.simulationTime


def main(args):
    # running using different maps
    input_files = [
        "random.domain/random_20.json",
        # "random.domain/random_50.json",
        # "random_100.json",
        # "random_200.json"
    ]

    if args.rebuild:
        # compilation
        compile_code()

    set_env_var({"PLANNER": args.planner})

    timestamp_filename = generate_filename()
    output_file_folder = f"benchmark_{timestamp_filename}"

    # execution
    run_code(input_files, output_file_folder, args)

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
    argParser.add_argument("--config", type=str, default=None,
                           help='Configurations to try out. Each possible combination of configs will be set as os '
                                'environment variables and tested. - Use this to test out different parameters of your '
                                'planner.')

    args = argParser.parse_args()
    main(args)
