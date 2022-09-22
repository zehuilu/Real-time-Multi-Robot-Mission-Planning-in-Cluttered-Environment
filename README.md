# DrMaMP-Distributed-Real-time-Multi-agent-Mission-Planning-Algorithm
This is DrMaMP: Distributed Real-time Multi-agent Mission Planning in Cluttered Environment.

We have already submitted our paper "DrMaMP: Distributed Real-time Multi-agent Mission Planning in Cluttered Environment" to IEEE Robotics and Automation Letters.


This repo has been tested with:
* GCC 10.2.0, CMake 3.16.3, Ubuntu 20.04.2 LTS
* GCC 9.3.0, CMake 3.16.3, Ubuntu 20.04.1 LTS
* Clang 12.0.0, CMake 3.18.3, macOS 10.15.7
* Clang 13.0.0, CMake 3.22.0, macOS 11.4


An example with 8 agents and 40 tasks in a 50 * 50 grid map with 250 obstacles is shown below. The computational time is 59.96 ms.

![Example](doc/example_after.png?raw=true "Example")


This repo is used for hardware experiments. A video about real-time mission planning for two Parrot Mambo with static/dynamic obstacles and tasks is shown in [https://youtu.be/il3YxhXgGac](https://youtu.be/il3YxhXgGac).

![Experiment](doc/experiment_screenshoot.jpeg?raw=true "Experiment")

Dependencies
============
* [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding)
  - [pybind11](https://github.com/pybind/pybind11)
  - [numpy](https://numpy.org/)
  - [matplotlib](https://matplotlib.org/)
  - [SciPy](https://www.scipy.org/)

* [OR-Tools](https://developers.google.com/optimization)


For experiments **ONLY**:

This repo uses two Parrot Mambo quadrotors to do the experiments. To control these quadrotors based on results from DrMaMP, you need to download and install the following repository by the instructions.
* [Mambo-Tracking-Interface](https://github.com/zehuilu/Mambo-Tracking-Interface)

For comparisons **ONLY**:
* [CBBA-Python](https://github.com/zehuilu/CBBA-Python.git)
  - Python >= 3.7
  - [numpy](https://numpy.org/)
  - [matplotlib](https://matplotlib.org/)

For C++ K-means clustering algorithm comparisons **ONLY** (you don't need the following packages if not running `test/k_means_comparison.py`):
  - [scikit-learn](https://scikit-learn.org/stable/)

Build
=====

This repo uses the binary package of [OR-Tools](https://developers.google.com/optimization), so you don't have to compile it by yourself. You just need to make [OR-Tools](https://developers.google.com/optimization) as a library of my codes in CMakeLists.txt (which I've done) and compile the solver.
To install (download) [OR-Tools](https://developers.google.com/optimization) for C++ from Binary on Linux or Macos, follow the instructions [here](https://developers.google.com/optimization/install/cpp/linux) or [here](https://developers.google.com/optimization/install/cpp/mac). After you download the binary package and make sure [OR-Tools](https://developers.google.com/optimization) is working (see how to test it in Section **Run**), do the following to build C++ codes. See more details [here](https://github.com/jwdinius/ortools-with-cmake) and [here](https://github.com/google/or-tools/issues/1440) to build C++ codes by CMake with OR-Tools Binary Distribution.


Since [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding) is a submodule of this repo, follow the instructions below to build [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding) and this repo respectively.


To download this repo and build [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding),
```
$ sudo apt install build-essential zlib1g-dev # For macOS: xcode-select --install && brew install cmake zlib
$ sudo apt install libgflags-dev libgoogle-glog-dev # For macOS: brew install gflags glog
$ apt install python3-pybind11 # For macOS: brew install pybind11
$ pip3 install numpy matplotlib scipy
$ git clone https://github.com/zehuilu/DrMaMP-Distributed-Real-time-Multi-agent-Mission-Planning-Algorithm.git
$ cd <MAIN_DIRECTORY>
$ git submodule update --init --recursive
$ cd externals/Lazy-Theta-with-optimization-any-angle-pathfinding
$ mkdir build
$ cd build
$ cmake ..
$ make
```


To build this repo,
```
$ cd <MAIN_DIRECTORY>
$ mkdir build
$ cd build
$ # download OR-Tools Binary and extract it
$ cmake -DORTOOLS_ROOT=<path-to-ortools-root-dir> ..

# For example (macOS): cmake -DORTOOLS_ROOT=/Users/zehui/or-tools_MacOsX-11.2.1_v8.2.8710 ..
# For example (Linux): cmake -DORTOOLS_ROOT=/home/aims-zehui/or-tools_Ubuntu-20.04-64bit_v8.0.8283 ..
# For example (Linux): cmake -DORTOOLS_ROOT=/home/zehui/or-tools_Ubuntu-20.04-64bit_v8.2.8710 ..

$ make
```


Test
====

* To test if [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding) is working correctly:
```
$ cd <MAIN_DIRECTORY>/externals/Lazy-Theta-with-optimization-any-angle-pathfinding
$ build/main_single_path
```

* To test if [OR-Tools](https://developers.google.com/optimization) is installed properly and works correctly:
```
$ cd <MAIN_DIRECTORY>/build
$ ./test_or_tools
```

* To test if [CBBA-Python](https://github.com/zehuilu/CBBA-Python.git) is working correctly:
```
$ cd <MAIN_DIRECTORY>/
$ python3 example/run_cbba.py
```

* To test if [CBBA-Python](https://github.com/zehuilu/CBBA-Python.git) + path finding is working correctly:
```
$ cd <MAIN_DIRECTORY>/
$ python3 example/run_cbba_and_find_path.py
```


Run
===

<!-- * To run the solver in C++:
```
$ cd <MAIN_DIRECTORY>/build
$ ./test_solve_cpp
``` -->

* To run Mission Planner once:
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_MissionPlanning.py
```

* To run Mission Planner iteratively:
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_MissionPlanning_online.py
```

* To run Mission Planner for one agent once:
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_SolveOneAgent.py
```

* To run Mission Planner for one agent iteratively:
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_SolveOneAgent_online.py
```

* To run Mission Planner by CBBA + path finding (the existing method):
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_CBBA_Path_Finding.py
```

* To run the proposed method and the existing method for the same scenario:
```
$ cd <MAIN_DIRECTORY>
$ python3 comparison/single_compare_CBBA_many_agents.py
```


Experiments
===========

First, you need to download [Mambo-Tracking-Interface](https://github.com/zehuilu/Mambo-Tracking-Interface) and follow the instructions to install.


1. Run one Mambo with Qualisys Motion Capture System and Offline Planner once.

* Create a directory for csv trajectories
```
$ cd <Mambo-Tracking-Interface>/scripts_aimslab/
$ mkdir traj_csv_files
$ mkdir traj_csv_files/mambo_01
```

* Run Mocap Qualisys for Mambo tracking controller:
```
$ cd <Mambo-Tracking-Interface>
$ python3 scripts_aimslab/run_mocap_qualisys.py <mambo_id>
```

* Run Mambo tracking controller:
```
$ cd <Mambo-Tracking-Interface>
$ python3 scripts_aimslab/run_mambo.py <mambo_id>
```

* Run Offline Planner once:
```
$ cd <MAIN_DIRECTORY>
$ python3 experiment/scripts/run_planner_once.py <mambo_id>
# example: $ python3 experiment/scripts/run_planner_once.py 1
```


2. Run two Mambo with Qualisys Motion Capture System and Online Planner.

* Create a directory for csv trajectories
```
$ cd <Mambo-Tracking-Interface>/scripts_aimslab/
$ mkdir traj_csv_files
$ mkdir traj_csv_files/mambo_01
$ mkdir traj_csv_files/mambo_02
$ mkdir traj_csv_files/mambo_03
$ chmod +x launch_mambo.sh
```

* Run mambo_01 Mocap Qualisys for {Online Planner, Mambo tracking controller}, and run Mambo tracking controller:
```
$ cd <Mambo-Tracking-Interface>
$ ./scripts_aimslab/launch_mambo.sh 1 true
```

* Run mambo_03 Mocap Qualisys for {Online Planner, Mambo tracking controller}, and run Mambo tracking controller:
```
$ cd <Mambo-Tracking-Interface>
$ ./scripts_aimslab/launch_mambo.sh 3 true
```

* Run Online Planner:
```
$ cd <MAIN_DIRECTORY>
$ python3 experiment/scripts/run_planner_online_multi_agent.py
```


<!-- 3. Run one Mambo with Qualisys Motion Capture System and Online Planner.

**NOTE**: I revised `AgentFSMExp.py` to make the hardware experiment with multiple agents work. But I haven't revised the scripts about single agent case. I will do that later (Feb. 21, 2022). But any scripts related to multiple agents DO work.

* Create a directory for csv trajectories
```
$ cd <Mambo-Tracking-Interface>/scripts_aimslab/
$ mkdir traj_csv_files
$ mkdir traj_csv_files/mambo_01
$ chmod +x launch_mambo.sh
```

* Run Mocap Qualisys for {Online Planner, Mambo tracking controller}, and run Mambo tracking controller:
```
$ cd <Mambo-Tracking-Interface>
$ ./scripts_aimslab/launch_mambo.sh <mambo_id> <run_mambo_flag>
# example: $ ./scripts_aimslab/launch_mambo.sh 1 true
```

* Run Online Planner:
```
$ cd <MAIN_DIRECTORY>
$ python3 experiment/scripts/run_planner_online.py <mambo_id>
# example: $ python3 experiment/scripts/run_planner_online.py
# example: $ python3 experiment/scripts/run_planner_online.py 1
# example: $ python3 experiment/scripts/run_planner_online.py 2
``` -->
