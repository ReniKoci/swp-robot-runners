import subprocess
import json
import time

# compilation
compile_cmd = "sh compile.sh"
subprocess.run(compile_cmd, shell=True, check=True)

# running using different maps
input_files = [
    "random.domain/random_20.json",
    "random.domain/random_50.json",
    # "random_100.json",
    # "random_200.json"
]


def run_code(input_file):
    map_file_location = "./example_problems/" + input_file

    # start exeuction time
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


# create a list to store results
results = []

# run code for each map
for input_file in input_files:
    num_task_finished, execution_time = run_code(input_file)
    results.append({"file": input_file, "tasks_finished": num_task_finished, "execution_time": execution_time})


output_file_path = "benchmark_results.json"
with open(output_file_path, "w") as output_file:
    json.dump(results, output_file, indent=4)
