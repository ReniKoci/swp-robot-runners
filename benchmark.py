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
    return [dict(zip(keys, v)) for v in itertools.product(*values)]


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


def plot_line(cumulative_finished, title, file_suffix, output_dir):
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


def plot_results(folder, configs):
    json_file = f"Output/{folder}/{folder}.json"
    output_dir = f"Output/{folder}"
    with open(json_file, "r") as file:
        data = json.load(file)

    for config in configs:
        config_key = tuple(config.items())
        filtered_data = [entry['events'] for entry in data if all(entry[k] == v for k, v in config.items())]
        cumulative_finished = get_cumulative_finished(filtered_data)
        plot_line(cumulative_finished, f'Finished Tasks over Time for Config {config_key}', f'combined_{config_key}',
                  output_dir)


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


def generate_config_str(config):
    return ', '.join(str(value) for value in config.values())


def generate_filename():
    return time.strftime("%Y%m%d_%H%M%S")


def run_code(input_files, output_file_folder, args):
    # get number of iterations if specified
    iterations = args.iterations

    # get all configs, if a parameter is not specfified, the list will be empty
    configs = combine_config_options(args)

    tasks_finished = {}
    # create a list to store results
    results = []

    runs = args.reruns
    for run in range(runs):
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
                                "events": events,
                                "run": run
                                }

                result_entry.update(config)
                results.append(result_entry)

                # update dict for when same config called multiple times
                mul_key, mul_value = generate_config_str(config), get_cumulative_finished([events])
                if mul_key in tasks_finished:
                    tasks_finished[mul_key].append(list(mul_value.values()))
                else:
                    tasks_finished[mul_key] = [list(mul_value.values())]

    # create the directory if it doesn't exist
    os.makedirs("Output", exist_ok=True)
    os.makedirs(f"Output/{output_file_folder}", exist_ok=True)

    # save the detailed file
    file_name = f"Output/{output_file_folder}/{output_file_folder}.json"
    with open(file_name, "a") as output_file:
        json.dump(results, output_file, indent=4)

    # save the number of tasks finished file
    file_name = f"Output/{output_file_folder}/tasks_finished.json"
    with open(file_name, "a") as output_file:
        json.dump(tasks_finished, output_file, indent=4)

    # plot the charts
    plot_results(output_file_folder, configs)


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

    argParser.add_argument("--config", type=str, default={},
                           help='Configuration for algorithm, heuristic, timeHorizon, replanningPeriod etc.')

    args = argParser.parse_args()
    main(args)
