import subprocess
import json
import os
import time
import shutil


# compilation
def compile_code():
    compile_cmd = "sh compile.sh"
    subprocess.run(compile_cmd, shell=True, check=True)


# executing algorithm for each map
def run_iteration(input_file):
    map_file_location = "./example_problems/" + input_file

    # start execution time
    start_time = time.time()
    run_cmd = f"./build/lifelong --inputFile {map_file_location} -o test.json"
    subprocess.run(run_cmd, shell=True, check=True)
    # end execution time
    execution_time = time.time() - start_time

    # get the output file and read data
    with open("test.json", "r") as output_file:
        output_data = json.load(output_file)

    num_tasks_finished = output_data.get("numTaskFinished", "Data not found")
    return num_tasks_finished, execution_time


def generate_filename():
    return time.strftime("%Y%m%d_%H%M%S")


def run_code(input_files, output_file_path):
    # create the directory if it doesnt exist
    output_directory = os.path.dirname(output_file_path)
    os.makedirs(output_directory, exist_ok=True)

    # create a list to store results
    results = []

    # run code for each map
    for input_file in input_files:
        num_task_finished, execution_time = run_iteration(input_file)
        results.append({"file": input_file, "tasks_finished": num_task_finished, "execution_time": execution_time})

    with open(output_file_path, "a") as output_file:
        json.dump(results, output_file, indent=4)


def get_total_tasks_finished(file_name):
    with open(file_name, "r") as file:
        data = json.load(file)

    total_tasks_finished = sum(entry.get("tasks_finished", 0) for entry in data)
    return total_tasks_finished


def compare_and_update_best_benchmark(file_name):
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


def main():
    # running using different maps
    input_files = [
        "random.domain/random_20.json",
        "random.domain/random_50.json",
        # "random_100.json",
        # "random_200.json"
    ]

    # compilation
    compile_code()

    timestamp_filename = generate_filename()
    output_file_path = f"Output/benchmark_{timestamp_filename}.json"

    # execution
    run_code(input_files, output_file_path)

    # check if we found a better algorithm
    compare_and_update_best_benchmark(output_file_path)


if __name__ == "__main__":
    main()
