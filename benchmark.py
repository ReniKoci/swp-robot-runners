import subprocess
import json
import os
import time
import shutil
import argparse
import pandas as pd
from math import log10
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import itertools
import ast
import seaborn as sns

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "Output")


# function to create the full path for files
def create_filepath(filename):
    return os.path.join(os.path.dirname(__file__), filename)


# compilation
def compile_code():
    compile_cmd = "sh compile.sh"
    subprocess.run(compile_cmd, shell=True, check=True)


def get_map_name(input_file):
    file_name = "./example_problems/" + input_file
    with open(file_name, "r") as file:
        data = json.load(file)
    return data.get("mapFile")


def get_map_path(input_file):
    map_file = get_map_name(input_file)
    domain_name = input_file[:input_file.find("/")]
    return create_filepath("example_problems/" + domain_name + "/" + map_file)


def run_planviz(args, json_path: str):
    if args.viz == "no-path-provided":
        path = (f"../../PlanViz/script/plan_viz.py --map {get_map_path(args.file)}"
                f" --plan {json_path} --grid --aid --end {args.iterations-1} "
                "--tid --ca")
    else:
        # if path provided (wthout the arguments)
        # example -> ./../PlanViz/script/plan_viz.py
        path = (f"{args.viz} --map {get_map_path(args.file)}"
                f" --plan {json_path} --grid --aid --end {args.iterations-1} "
                "--tid --ca")

    cmd = f"python3 {path}"
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
def get_cumulative_finished(data, max_time_step=None):
    finished_counts = defaultdict(int)
    for event_group in data:
        for event in event_group:
            # 'event' is a list within a list, extract the inner list
            for task_id, time_step, event_type in event:
                if event_type == "finished":
                    finished_counts[time_step] += 1

    max_time_step = max_time_step or max(finished_counts.keys(), default=0)
    cumulative_finished = {time_step: sum(finished_counts[key] for key in finished_counts if key <= time_step) for
                           time_step in range(1, max_time_step + 1)}
    return cumulative_finished


def calculate_xtick_step(x_min, x_max, target_ticks=20):
    """
    Calculate a smart x-tick step size based on the range of x values and a target number of ticks.

    Parameters:
    - x_min: The minimum value of the x-axis.
    - x_max: The maximum value of the x-axis.
    - target_ticks: The desired number of x-ticks (default: 10).

    Returns:
    - A suitable step size for the x-ticks.
    """
    if x_min == x_max:
        return 1
    # Calculate the range of x values
    x_range = x_max - x_min

    # Initial step size based on the target number of ticks
    raw_step = x_range / target_ticks

    # Round the step size to a "nice" value for better readability
    order_of_magnitude = 10 ** int(log10(raw_step))
    nice_factors = [1, 2, 5, 10]  # Common factors to adjust steps to nice values
    try:
        nice_step = min([factor * order_of_magnitude for factor in nice_factors if factor * order_of_magnitude <= raw_step], key=lambda x: abs(x - raw_step))
    except Exception as e:
        nice_step = min([factor * order_of_magnitude for factor in nice_factors], key=lambda x: abs(x - raw_step))
    return nice_step


def plot_line_range(data: dict[str, list[list[int]]], output_path: str, map_name=None, agent_count=None):
    plt.figure(figsize=(10, 6))
    max_througput = 0
    for config, runs in data.items():
        print(config)
        runs_array = np.array(runs)
        mean_values = np.mean(runs_array, axis=0)
        min_values = np.min(runs_array, axis=0)
        max_values = np.max(runs_array, axis=0)
        max_througput = max(max_througput, np.max(max_values))

        # plt the average line
        plt.plot(range(len(mean_values)), mean_values, label=config)
        last_x, last_y = len(mean_values) - 1, mean_values[-1]
        plt.annotate(f'{int(last_y)}', (last_x, last_y), textcoords="offset points", xytext=(15, -5), ha='center')

        # fill in the space between min and max
        plt.fill_between(range(len(mean_values)), min_values, max_values, alpha=0.2)

    # description
    plt.xlabel("Iteration step")
    plt.ylabel("Tasks Completed")
    plt.title("Throughput at step x" " (map: " + map_name + ", agents: " + agent_count + ")" if map_name else "")
    plt.legend()

    plt.xticks(get_ticks(len(mean_values)-1), rotation=90)
    plt.yticks(get_ticks(max_througput, 15))

    # save as picture
    plt.savefig(output_path)

def get_ticks(max: int, count=20) -> list[int]:
    step = calculate_xtick_step(0, max, count)
    ticks = list(np.arange(0, max+1, step=step))
    if max not in ticks:
        ticks = ticks[:-1]
        ticks.append(max)
    return ticks


def plot_proc_times(tasks_finished, output_path: str, map_name=None, agent_count=None):
    # Convert JSON data to a pandas DataFrame
    df_list = []
    for key, values in tasks_finished.items():
        for idx, timestamps in enumerate(values):
            for timestamp, value in enumerate(timestamps):
                df_list.append({'Config': key, 'Timestamp': timestamp, 'Value': value})

    df = pd.DataFrame(df_list)

    # Calculate the mean for each timestamp and key
    mean_df = df.groupby(['Timestamp', 'Config']).mean().reset_index()

    # Plot using seaborn
    sns.set(style="whitegrid")
    plt.figure(figsize=(12, 8))
    sns.barplot(x='Timestamp', y='Value', hue='Config', data=mean_df, palette="Set1")

    number_of_steps = len(mean_df['Timestamp'].unique())
    plt.xticks(get_ticks(number_of_steps-1))
    plt.title(
        'Processing time by configurations' + " (map: " + map_name + ", agents: " + agent_count + ")" if map_name else "")
    plt.xlabel('Iteration step')
    plt.ylabel('Processing time (seconds)')
    plt.legend(title='Config')

    # save as picture
    plt.savefig(output_path)


def plot_results(run_dir, configs, input_config_file_name):
    print(f"input_config_file_name: {input_config_file_name}")
    map_name = input_config_file_name.split(".")[0]
    agent_count = input_config_file_name.split("_")[-1].replace(".json", "")
    # number of tasks finished per config
    filename_tasks_finished = create_filepath(f"{run_dir}/tasks_finished.json")
    with open(filename_tasks_finished, "r") as file:
        tasks_finished = json.load(file)
    plot_line_range(tasks_finished, f"{run_dir}/tasks_finished_{map_name}_{agent_count}.png", map_name, agent_count)

    # processing time taken for each step for each config
    filename_tasks_finished = create_filepath(f"{run_dir}/processing_times.json")
    with open(filename_tasks_finished, "r") as file:
        processing_times = json.load(file)
    plot_proc_times(processing_times, f"{run_dir}/processing_times_{map_name}_{agent_count}.png", map_name, agent_count)


completed_config_counts_dict = defaultdict(int)


def config_to_file_name(config: dict, input_file_name: str, iterations="x") -> str:
    global completed_config_counts_dict
    # Exclude 'run' key from the configuration dictionary
    map_name = input_file_name.split(".")[0]
    agent_count = input_file_name.split("_")[-1].replace(".json", "")
    # todo add iterations, run_index, map_name, agent_count
    name = "_".join(f"{map_name}_{agent_count}_{iterations}_{key}_{str(value)}" for key, value in config.items() if key != "run")
    run_index = completed_config_counts_dict[name]
    completed_config_counts_dict[name] += 1
    name += f"_{run_index}"
    return name

def run_iteration(input_file, iterations=None, run_folder_path="", config: dict = None):
    file_name = config_to_file_name(config, input_file, iterations)
    output_file_path = run_folder_path + "/" + file_name + ".json"

    execution_time = 0
    if not os.path.exists(output_file_path):
        print(f"{output_file_path} not found, running the algorithm...")
        map_file_location = "./example_problems/" + input_file
        run_cmd = create_filepath(f"build/lifelong --planTimeLimit {args.planTimeLimit} --inputFile {map_file_location} "
                                  f"--output {output_file_path}")

        if iterations:
            run_cmd += f" --simulationTime {iterations}"

        print(run_cmd)
        # start execution time
        start_time = time.time()
        subprocess.run(run_cmd, shell=True, check=True)
        execution_time = time.time() - start_time
        # end execution time
    else:
        print(f"File {output_file_path} already exists, skipping execution")

    # get the output file and read data
    with open(output_file_path, "r") as output_file:
        output_data = json.load(output_file)

    # get number of tasks finished
    num_tasks_finished = output_data.get("numTaskFinished", "Tasks not found")
    # get the events of robots (assigned task, finished task)
    events = output_data.get("events", "Events not found")
    # get the processing times for each step
    planner_times = output_data.get("plannerTimes")

    # times are missing when the planner did not finish planning -> fill in with planTimeLimit
    steps_with_timeout = []
    for _, _, step, message in output_data.get("errors", []):
        if message == "incorrect vector size":
            steps_with_timeout.append(step)
    for step in steps_with_timeout:
        # insert it at the correct index
        planner_times.insert(step, args.planTimeLimit)

    return num_tasks_finished, execution_time, events, planner_times, output_file_path


def generate_config_str(config) -> str:
    return ", ".join(f"{key}: {str(value)}" for key, value in config.items() if key != "run")


def get_time_stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def run_code(input_file, output_file_folder, args) -> str:
    print(f"input file: {input_file}")
    # get number of iterations if specified
    iterations = args.iterations
    # dictionaries to store the number of tasks and processing time for each config
    tasks_finished = {}
    processing_times = {}
    # get all configs, if a parameter is not specified, the list will be empty
    configs = combine_config_options(args)
    # create a list to store results
    results = []
    # run code for each map and configuration
    run_dir_path = os.path.join(OUTPUT_DIR, output_file_folder)
    os.makedirs(create_filepath(run_dir_path), exist_ok=True)

    for config in configs:
        # save configurations as env variables
        set_env_var(config)

        num_task_finished, execution_time, events, planner_times, output_file_path = run_iteration(input_file,
                                                                                                   iterations,
                                                                                                   run_dir_path, config)
        result_entry = {"file": input_file,
                        "timesteps_taken": iterations,
                        "tasks_finished": num_task_finished,
                        "execution_time": execution_time,
                        "events": events
                        }

        result_entry.update(config)
        results.append(result_entry)

        # update dict for when same config called multiple times
        mul_key, mul_value = generate_config_str(config), get_cumulative_finished([events], max_time_step=iterations)
        if mul_key in tasks_finished:
            tasks_finished[mul_key].append(list(mul_value.values()))
        else:
            tasks_finished[mul_key] = [list(mul_value.values())]

        # create a dict for planning times
        if mul_key in processing_times:
            processing_times[mul_key].append(list(planner_times))
        else:
            processing_times[mul_key] = [list(planner_times)]

    # save the detailed file
    result_json_path = create_filepath(f"{run_dir_path}/{output_file_folder}.json")
    with open(result_json_path, "a") as output_file:
        json.dump(results, output_file, indent=4)

    # save the number of tasks finished file
    file_name = create_filepath(f"{run_dir_path}/tasks_finished.json")
    with open(file_name, "a") as output_file:
        json.dump(tasks_finished, output_file, indent=4)

    # save the number of processing time for each step for each config
    file_name = create_filepath(f"{run_dir_path}/processing_times.json")
    with open(file_name, "a") as output_file:
        json.dump(processing_times, output_file, indent=4)

    # plot the charts
    plot_results(run_dir_path, configs, input_file)
    return output_file_path


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


def main(args, files):
    # check if user wants to rebuild lifelong file
    if args.rebuild:
        # compilation
        compile_code()
    for file in files:
        set_env_var({"PLANNER": args.planner})

        output_file_folder = args.resume or f"benchmark_{get_time_stamp()}"
        if args.resume:
            # delete "processing_times.json" and "tasks_finished.json" to avoid appending to the file
            for f in ["processing_times.json", "tasks_finished.json"]:
                file_path = create_filepath(f"{OUTPUT_DIR}/{output_file_folder}/{f}")
                if os.path.exists(file_path):
                    os.remove(file_path)

        # execution
        result_json_path = run_code(file, output_file_folder, args)

        # run planviz if specified
        if args.viz:
            run_planviz(args, result_json_path)

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

    argParser.add_argument("--iterations", type=int, nargs="?", default=10,
                           help="Specify the number of iterations(steps)")

    argParser.add_argument("--viz", type=str, nargs="?",
                           const="no-path-provided",
                           help="If specified, the last call of the algorithm will run in planviz")

    argParser.add_argument("--reruns", type=int, nargs="?", default=1,
                           help="Specify the amount of times to run one configuration")

    argParser.add_argument("--resume", type=str, default=None,
                           help="Specify the benchmark result folder name to continue a previous run")

    argParser.add_argument("--planTimeLimit", type=int, nargs="?", default=2147483647,
                           help="Specify the amount of allowed processing time for one step (in seconds)")

    argParser.add_argument("--config", type=str, default=None,
                           help='Configuration for algorithm, heuristic, timeHorizon, replanningPeriod etc.')

    argParser.add_argument("--file", type=str, default="random.domain/random_20.json",
                           help="Location of the map to run the algorithm on (e.g. random.domain/random_20.json)")

    args = argParser.parse_args()

    all_files = ["random.domain/random_20.json", "random.domain/random_50.json", "random.domain/random_200.json",
                 "game.domain/brc202d_200.json", "game.domain/brc202d_500.json", "game.domain/brc202d_1000.json",
                 "city.domain/paris_200.json", "city.domain/paris_500.json", "city.domain/paris_1000.json"]

    main(args, all_files if args.file == "all" else [args.file])
