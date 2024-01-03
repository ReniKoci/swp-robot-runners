import subprocess
import json
import os
import time
import shutil
import argparse
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd


# compilation
def compile_code():
    compile_cmd = "sh compile.sh"
    subprocess.run(compile_cmd, shell=True, check=True)


def plot_results(folder):
    json_file = f"Output/{folder}/{folder}.json"
    output_dir = f"Output/{folder}"
    with open(json_file, "r") as file:
        data = json.load(file)

    # Create a DataFrame from the JSON data
    df = pd.DataFrame(data)

    # Create a pivot table for tasks_finished
    pivot_tasks_finished = df.pivot_table(index='heuristic', columns=['timeHorizon', 'replanningPeriod'],
                                          values='tasks_finished')

    plt.figure(figsize=(12, 8))
    sns.heatmap(pivot_tasks_finished, cmap='YlGnBu', annot=True, fmt=".1f", cbar=True)
    plt.xlabel("Time Horizon - Replanning Period")
    plt.ylabel("Heuristic")
    plt.title("Tasks Finished Heatmap")
    plt.savefig(f"{output_dir}/tasks_finished_heatmap", bbox_inches='tight')
    plt.close()

    # Create a bar plot for execution_time
    plt.figure(figsize=(12, 8))
    sns.barplot(x='heuristic', y='execution_time', hue='timeHorizon', data=df)
    plt.xlabel("Heuristic")
    plt.ylabel("Execution Time (seconds)")
    plt.title("Execution Time vs. Heuristic (Time Horizon as Hue)")
    plt.legend(title="Time Horizon")
    plt.xticks(rotation=45)
    plt.savefig(f"{output_dir}/time_horizon_exe_time", bbox_inches='tight')
    plt.close()

    # Create a bar plot for tasks_finished
    plt.figure(figsize=(12, 8))
    sns.barplot(x='heuristic', y='tasks_finished', hue='timeHorizon', data=df)
    plt.xlabel("Heuristic")
    plt.ylabel("Tasks Finished")
    plt.title("Tasks Finished vs. Heuristic (Time Horizon as Hue)")
    plt.legend(title="Time Horizon")
    plt.xticks(rotation=45)
    plt.savefig(f"{output_dir}/time_horizon_tasks_finished", bbox_inches='tight')
    plt.close()


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

    num_tasks_finished = output_data.get("numTaskFinished", "Data not found")  # get number of tasks finished

    return num_tasks_finished, execution_time


def generate_filename():
    return time.strftime("%Y%m%d_%H%M%S")


def run_code(input_files, output_file_folder, args):
    # get number of iterations if specified
    iterations = args.iterations

    # get all configs, if a parameter is not specfified, the list will be empty
    if args.config is not None:
        heuristics = args.config.get('heuristic', [])
        time_horizon = args.config.get('timeHorizon', [])
        replanning_period = args.config.get('replanningPeriod', [])

    # create a list to store results
    results = []

    # run code for each map
    for input_file in input_files:
        num_task_finished, execution_time = run_iteration(input_file, iterations)
        results.append({"file": input_file, "timesteps_taken": iterations, "tasks_finished": num_task_finished,
                        "execution_time": execution_time})

    # create the directory if it doesn't exist
    os.makedirs("Output", exist_ok=True)
    os.makedirs(f"Output/{output_file_folder}", exist_ok=True)

    file_name = f"Output/{output_file_folder}/{output_file_folder}.json"
    with open(file_name, "a") as output_file:
        json.dump(results, output_file, indent=4)

    # plot the charts
    if args.config is not None:
        plot_results(output_file_folder)


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
    argParser.add_argument("--iterations", type=int, nargs="?", default=None, help="Specify the number of iterations("
                                                                                   "steps")
    argParser.add_argument("--config", type=json.loads, default=None,
                           help='Configuration for algorithm, heuristic, timeHorizon, '
                                'replanningPeriod etc.')

    args = argParser.parse_args()
    main(args)
