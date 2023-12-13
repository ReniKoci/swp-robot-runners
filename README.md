# How to actually set up robot runners
## Setup IDE
1. Download Visual Studio Code or VSCodium
    - if VSCode runs inside of a Flatpak you need to add the following into `settings.json` under `terminal.integrated.profiles.linux`:
    ```json
    "bash (outside flatpak)": {
        "path": "/usr/bin/flatpak-spawn",
        "icon": "terminal-bash",
        "args": [
            "--host",
            "--env=TERM=xterm-256color",
            "bash"
        ]
    },
    ```
2. Clone repository

## Python setup
1. create a venv with Python 3.10
2. activate the venv
3. `python -m pip install -r requirements.txt`
4. `pre-commit install` to activate git hook that runs auto formatting and linting on commit

## Install dependencies
Install dependencies on Ubuntu 22.04+ or Debian 11+:
```shell
sudo apt-get update
sudo apt-get install cmake build-essential libboost-all-dev python3-dev python3-pybind11
```
Install dependencies on Fedora 38+:
```shell
sudo dnf install cmake boost-devel python3.10-devel pybind11-devel
```
For all other Linux environments it's recommended to use Distrobox.

### Compiling
1. activate the python environment
2. Using `compile.sh`:
```shell
./compile.sh
```
If compiling doesn't work try deleting the build directory.

# Important commands
## Run
### in ./build:
```
./lifelong --inputFile ../example_problems/random.domain/random_20.json -o test.json
```

## Visualize
- clone PlanViz into the same directory as swp-robot-runners
- visualize the run with PlanViz by providing the map and the generated "log" file `test.json`:
    - in ./build:
```
python3 ../../PlanViz/script/plan_viz.py --map ../example_problems/random.domain/maps/random-32-32-20.map --plan ./test.json --grid --aid --static --ca
```
## Debug
It is not that easy to debug our Python MapfPlanner since it is called each second by the compiled c++ scirpt. Hence we need a remote-debugging server.
### VS Code
`import debugpy`

Add the following lines:
```python
# to wait for debugger connection:
debugpy.listen(('localhost', 5678))
print("Waiting for debugger attach")
debugpy.wait_for_client()
```
1. call `python3 benchmark.py` or any other thing that calls our planner.
1. In VsCode (while having pyMAPFPlanner.py open):
    1. Click on the Debug-Button
    1. Click "Run and Debug"
    1. Click "Remote Attach"
    1. Enter localhost as IP
    1. Enter 5678 as Port
1. start debugging :)

### IntelliJ
- Follow these steps from the IntellJ documentation, ignore all the other steps:
  - [Create a run/debug configuration](https://www.jetbrains.com/help/idea/remote-debugging-with-product.html#create-remote-debug-config)
  - [Launch the Debug Server](https://www.jetbrains.com/help/idea/remote-debugging-with-product.html#launch-debug-server)
#### TLDR:
- `import pydevd_pycharm`
- Add line: `pydevd_pycharm.settrace('localhost', port=12345, stdoutToServer=True, stderrToServer=True)`
---

---
Original Readme:
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

### Dependencies

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
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j
```

## Run the start kit

Running the start-kit using commands: 
```shell
./build/lifelong --inputFile the_input_file_name -o output_file_location
```

for example:
```shell
./build/lifelong --inputFile ./example_problems/random.domain/random_20.json -o test.json
```

more info on help:
```shell
./build/lifelong --help
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

The `upgrade_start_kit.sh` will check which file is marked as an upgrade needed and pull those files from the start-kit. It will pull and stage the files, but not commit them. This allows you to review the changes before committing them. 

For files stated as unmodifiable in [Parepare_Your_Planner.md](./Prepare_Your_Planner.md), you always commit their changes.

The upgrade may overwrite some of your changes to `CMakeLists.txt`, `compile.sh`, and `apt.txt`, you could compare the difference using `git diff` and decide whether to revert some modifications or partially accept changes on these files.

The upgrade script will not touch any participants' created file, `python/pyMAPFPlanner.py`, `inc/MAPFPlanner.h` and `src/MAPFPlanner.cpp`. So that participants' implementations should not be influenced by the start-kit upgrade.

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Planner.md](./Prepare_Your_Planner.md).

## Debug and Visualise Your Planner
We provide a visualisation tool written in Python: [https://github.com/MAPF-Competition/PlanViz](https://github.com/MAPF-Competition/PlanViz).
It is able to visualise the output of the start-kit program and help participants debug the implementations. 

Please refer to the project website for more information. Also the document [Debug_and_Visualise_Your_Planner](./Debug_and_Visualise_Your_Planner.md) which provides helpful hints for interpreting and diagnosing planner output.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).



