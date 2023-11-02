# Important commands
## Run

### creating lifelong 
```
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true
make -j
```

### in ./build:
```
./lifelong --inputFile ../example_problems/random.domain/random_20.json -o test.json
```

## Visualize
- clone PlanViz into the same directory as swp-robot-runners
in ./build:
```
python3 ../../PlanViz/script/plan_viz.py --map ../example_problems/random.domain/maps/random-32-32-20.map --plan ./test.json --grid --aid --static --ca
```

# Plan for the next weeks
1. Create a MVP:
    - one robot walks from emitter to delivery point
    - act as a baseline
1. Extend the MVP from step 1 to work with multiple robots 
    - does not have to be good as long as the robots don't crash
1. Create a more fine garin project plan (1 page)
1. Create the project plan presentation
1. Write a script that 
    - write a script that automatically runs lifelong with multiple configs (maps,...) and meassures and logs the performance
    - --> this will enable easy evaluation 
3. Split into smaller groups
    - each group implements a different approach
    - this is not a competition -> maybe we can combine different approaches at the end to reach the best result
...
...
4. Final presentation and report: **14. February 2024**
each step except the last one can be processed simultaniously by 1 to 2 persons


# Start-Kit

## Join the competition

Log in to the [competition website](http://www.leagueofrobotrunners.org/) with a GitHub account, and we will automatically create a private GitHub submission repo for you.
The repo will be the place where you submit codes. In the `My Submission` page, you can click "My Repo" to open your GitHub submission repo page.

## Clone your submission repo

Clone your submission repo to your local machine. The repo contains starter codes to help you prepare your submission.

```
$ git clone git@github.com:your_submission_repo_address
$ cd your_submission_repo
```

## Compile the start-kit

### Denpendencies

- [cmake >= 3.16](https://cmake.org/)
- [libboost >= 1.49.0](https://www.boost.org/)
- Python3 and [pybind11](https://pybind11.readthedocs.io/en/stable/) (for python interface user)

Install dependencies on Ubuntu or Debian Linux:
```shell
sudo apt-get update
sudo apt-get install build-essential libboost-all-dev python3-dev python3-pybind11 
```

[Homebrew](https://brew.sh/) is recomanded for installing dependencies on Mac OS.

### Compiling

Using `compile.sh`:
```shell
./compile.sh
```

Using cmake: 
```shell
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j
```

## Run the start kit

Running the start-kit using commands: 
```shell
./lifelong --inputFile the_input_file_name -o output_file_location
```

for example:
```shell
./lifelong --inputFile ../example_problems/random.domain/random_20.json -o test.json
```

more info on help:
```shell
./lifelong --help
```

## Windows users
If you are a Windows user, the most straightforward method to utilize our start-kits is by employing the WSL (Windows Subsystem for Linux) subsystem. Follow these steps:
1. Install WSL, please refer to [https://learn.microsoft.com/en-us/windows/wsl/install](https://learn.microsoft.com/en-us/windows/wsl/install)
2. Open a shell in WSL and execute the following commands to install the necessary tools (CMake, GCC, Boost, pip, Pybind11):
```shell
sudo apt-get update
sudo apt-get install cmake g++ libboost-all-dev python3-pip python3-pybind11 
```
3. Employ the commands provided above to compile the start-kit.

While it's technically possible to use our start-kit with Cygwin, Mingw, and MSVC, doing so would be more complex compared to using WSL. You would likely need to configure the environment yourself.

## Upgrade Your Start-Kit

If your private start-kit copy repo was created before a start-kit upgrade, you could run the `./upgrade_start_kit.sh` to upgrade your start-kit to the latest version.

You can check `version.txt` to know the current version of your start-kit.

The `upgrade_start_kit.sh` will check which file is marked as an upgrade needed and pull those files from the start-kit.

The upgrade may overwrite some of your changes to `CMakeLists.txt` and `apt.txt`, you could compare the difference using `git diff` and decide whether to revert some modifications on these files.

The upgrade script will not touch any participants' created file, `python/pyMAPFPlanner.py`, `inc/MAPFPlanner.h` and `src/MAPFPlanner.cpp`. So that participants' implementations should not be influenced by the start-kit upgrade.

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Planner.md](./Prepare_Your_Planner.md).

## Visualisation
We provide a visualisation tool written in Python: [https://github.com/MAPF-Competition/MAPF_analysis](https://github.com/MAPF-Competition/MAPF_analysis).

It is able to visualise the output of the start-kit program and help participants to debug the implementations.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).



