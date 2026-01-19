# MeshA* for Efficient Path Planning with Motion Primitives

## Overview
This project consists of a core component written in C++ (implementation of search algorithms) and a visualization component developed in Python. The development was carried out on a Linux environment using the GCC compiler, and the Python code requires the libraries NumPy and Matplotlib.


## Project Structure
The project is organized into the following directories:

- **data/**: Contains files with control set and corresponding information about the configurations of primitives for searching on the mesh graph.
- **maps/**: Includes maps from the MovingAI benchmark along with their scenarios.
- **include/**: Header files for С++.
- **src/**: Source code files for C++.
- **res/**: Directory where the results of the algorithm's execution are stored.
- **visualization/**: Directory containing Python code for visualizing the generated paths.
- **Makefile**: A file used to build the project C++.


## Building the Project
To compile the project, navigate to the project directory and run the following command:

```bash
make mesh_astar
```

## Running the Algorithm
After building the project, you can execute the algorithm with the following command:

```bash
./mesh_astar
```

The algorithm will run for up to 1 minute and will generate the following files in the `res/` directory:

- `1.000000_AR0304SR_control_set_primitive_configurations_0_0.txt`: This file contains the results of comparative testing of the MeshA*, LBA*, Mesh/ParallA*, Mesh/PruningA*, and LazyLBA* algorithms on the first 10 tests of the AR0304SR map.
- `*_result.txt`: These files contain the results of each algorithm's execution on test number 4019 of the AR0304SR map.

By default, the weight \( w \) is set to 1 across all configurations.

## Configuration Options
You can modify the settings as follows:

1. In the file `include/common.hpp`, the parameter `MAX_TESTS` controls the number of tests to be executed (default is 10).
2. In the file `src/KC_testing.cpp`, at the bottom of the `main` function, you can adjust the parameters:
   - `i`: Selects the map for testing (ranging from 0 to 3, default is 2).
   - `w`: Sets the weight of the heuristic (default is 1.0).
3. In the same file, within the `solve_test` function, you can change the parameter `ind` (default is 4099) to select a different test case.

## Visualization
After the `mesh_astar` algorithm has completed and generated the `*_result.txt` files, you can visualize the maps with the found trajectories. To do this, navigate to the `visualization/` directory and run the following command:

```bash
python3 show_map_and_path.py
```

By default, this script will generate an image for the path found by the LBA* algorithm, but it can be easily modified to visualize paths from other algorithms. 

It is important to note that the AR0304SR map consists largely of obstacles, so the visualization process may take several minutes to complete. 