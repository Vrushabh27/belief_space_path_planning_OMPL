# IG-RRT* and RI-PRM* Path Planning

This repository contains C++ implementations of the IG-RRT* and RI-PRM* path planning algorithms, along with Python scripts for visualizing the results. The code generates random obstacles, plans paths avoiding them, and visualizes the paths along with uncertainty ellipsoids.

This README provides detailed instructions to set up the environment, compile and run the code, and understand the tunable parameters. 

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Update and Upgrade System](#1-update-and-upgrade-system)
  - [2. Install Required Packages](#2-install-required-packages)
  - [3. Install Eigen Library](#3-install-eigen-library)
  - [4. Install Boost Libraries](#4-install-boost-libraries)
  - [5. Install OMPL Library](#5-install-ompl-library)
- [Compiling the C++ Code](#compiling-the-c-code)
  - [1. Download the Source Code](#1-download-the-source-code)
  - [2. Compile RRT* Code](#2-compile-rrt-code)
  - [3. Compile PRM* Code](#3-compile-prm-code)
- [Running the C++ Code](#running-the-c-code)
  - [1. Running the RRT* Planner](#1-running-the-rrt-planner)
  - [2. Running the PRM* Planner](#2-running-the-prm-planner)
- [Running the Python Plotting Scripts](#running-the-python-plotting-scripts)
  - [1. Install Python Dependencies](#1-install-python-dependencies)
  - [2. Running the RRT* Plotting Script](#2-running-the-rrt-plotting-script)
  - [3. Running the PRM* Plotting Script](#3-running-the-prm-plotting-script)
- [Understanding the Tunable Parameters](#understanding-the-tunable-parameters)
  - [1. C++ Code Parameters](#1-c-code-parameters)
    - [a. `max_number`](#a-max_number)
    - [b. `alpha`](#b-alpha)
    - [c. Obstacle Parameters](#c-obstacle-parameters)
  - [2. Python Plotting Code Parameters](#2-python-plotting-code-parameters)
- [Additional Notes](#additional-notes)
  - [1. Common Issues and Solutions](#1-common-issues-and-solutions)
  - [2. Tips for Customization](#2-tips-for-customization)
- [Conclusion](#conclusion)

---

## Prerequisites

- **Ubuntu Machine**: This guide assumes you are using an Ubuntu 18.04 LTS or later.
- **Basic Knowledge**: Familiarity with terminal commands, compiling C++ code, and running Python scripts.
- **Dependencies**: The code requires the following libraries:
  - **OMPL** (Open Motion Planning Library)
  - **Eigen** (C++ template library for linear algebra)
  - **Boost** (Collection of portable C++ source libraries)

---

## Installation

### 1. Update and Upgrade System

Open a terminal and update your system packages:

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. Install Required Packages

Install essential build tools and dependencies:

```bash
sudo apt install -y build-essential cmake git pkg-config
```

### 3. Install Eigen Library

#### Option 1: Install from Ubuntu Repositories

```bash
sudo apt install -y libeigen3-dev
```

#### Option 2: Install from Source (Recommended for Latest Version)

```bash
cd ~
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build && cd build
cmake ..
sudo make install
```

### 4. Install Boost Libraries

```bash
sudo apt install -y libboost-all-dev
```

### 5. Install OMPL Library

#### Option 1: Install from Ubuntu Repositories

**Note**: This may not install the latest version.

```bash
sudo apt install -y libompl-dev
```

#### Option 2: Install from Source (Recommended)

```bash
cd ~
git clone https://github.com/ompl/ompl.git
cd ompl
mkdir -p build/Release
cd build/Release
cmake ../..
make -j$(nproc)
sudo make install
```

#### Verify OMPL Installation

Check if OMPL is installed correctly:

```bash
pkg-config --modversion ompl
```

---

## Compiling the C++ Code

### 1. Download the Source Code

Clone this repository or download the source code files (`rrt_star_with_custom_cost.cpp`, `prm_star_with_custom_cost.cpp`, and the Python scripts) into a directory.

```bash
cd ~
mkdir path_planning
cd path_planning
```

Place the C++ and Python files in this directory.

### 2. Compile RRT* Code

```bash
g++ -std=c++11 rrt_star_with_custom_cost.cpp -o rrt_star \
    -I/usr/include/eigen3 \
    -I/usr/include/boost \
    $(pkg-config --cflags --libs ompl) \
    -lboost_system \
    -lboost_filesystem \
    -lpthread
```

**Explanation**:

- `-std=c++11`: Use C++11 standard.
- `-o rrt_star`: Output executable named `rrt_star`.
- `-I`: Include directories for Eigen and Boost.
- `$(pkg-config --cflags --libs ompl)`: Automatically includes the necessary flags for OMPL.
- `-lboost_system -lboost_filesystem -lpthread`: Link against Boost and pthread libraries.

### 3. Compile PRM* Code

```bash
g++ -std=c++11 prm_star_with_custom_cost.cpp -o prm_star \
    -I/usr/include/eigen3 \
    -I/usr/include/boost \
    $(pkg-config --cflags --libs ompl) \
    -lboost_system \
    -lboost_filesystem \
    -lpthread
```

---

## Running the C++ Code

### 1. Running the RRT* Planner

```bash
./rrt_star
```

**Expected Output**:

- The program will display messages indicating the progress of each run.
- It will generate obstacle files (`obstacles_1.csv`, `obstacles_2.csv`, etc.) and path data files (`path_data_rrt_1.csv`, `path_data_rrt_2.csv`, etc.).

### 2. Running the PRM* Planner

```bash
./prm_star
```

**Expected Output**:

- The program will display messages about the planning process.
- It will generate `obstacles_prm.csv` and `path_data_prm.csv`.

---

## Running the Python Plotting Scripts

### 1. Install Python Dependencies

Ensure you have Python 3 and `pip` installed:

```bash
sudo apt install -y python3 python3-pip
```

Install required Python packages:

```bash
pip3 install matplotlib numpy
```

### 2. Running the RRT* Plotting Script

Save the following script as `plot_paths.py`:

```python
# plot_paths.py

import matplotlib.pyplot as plt
import csv
import matplotlib.patches as patches
import numpy as np
import os

def read_obstacles(filename):
    obstacles = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)  # Skip header
        for row in reader:
            type_ = row[0]
            num_vertices = int(row[1])
            vertices = []
            verts = row[2:]
            for i in range(0, len(verts), 2):
                x = float(verts[i])
                y = float(verts[i+1])
                vertices.append((x, y))
            obstacles.append((type_, vertices))
    return obstacles

def plot_obstacles(obstacles, ax):
    for obs in obstacles:
        type_, vertices = obs
        polygon = patches.Polygon(vertices, closed=True, fill=True, edgecolor='black', facecolor='gray', alpha=0.5)
        ax.add_patch(polygon)

def read_path(filename):
    path = []
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            P11 = float(row['P11'])
            P12 = float(row['P12'])
            P22 = float(row['P22'])
            path.append((x, y, P11, P12, P22))
    return path

def plot_path(path, ax):
    xs = [state[0] for state in path]
    ys = [state[1] for state in path]
    ax.plot(xs, ys, '-o', color='blue', label='RRT* Path')

    # Plot uncertainty ellipsoids
    for idx, state in enumerate(path):
        x, y, P11, P12, P22 = state
        cov = np.array([[P11, P12],
                        [P12, P22]])

        # Ensure covariance matrix is symmetric
        cov = (cov + cov.T) / 2

        # Eigenvalues and eigenvectors for the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Handle negative eigenvalues
        if np.any(eigenvalues < 0):
            print(f"Negative eigenvalues at index {idx}, position ({x}, {y}): {eigenvalues}")
            # Adjust small negative eigenvalues
            eigenvalues[eigenvalues < 0] = 0.0
            # If eigenvalues are significantly negative, skip plotting
            if np.any(eigenvalues < -1e-6):
                print(f"Significant negative eigenvalues encountered, skipping ellipsoid at index {idx}.")
                continue

        # Proceed with plotting
        order = eigenvalues.argsort()[::-1]
        eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]
        angle = np.degrees(np.arctan2(*eigenvectors[:,0][::-1]))
        width, height = 2 * np.sqrt(eigenvalues)  # 1-sigma ellipse

        # Check for NaN or infinite values
        if not np.isfinite(width) or not np.isfinite(height):
            print(f"Non-finite ellipse dimensions at index {idx}, skipping.")
            continue

        ellip = patches.Ellipse((x, y), width, height, angle,
                                edgecolor='red', facecolor='none', linestyle='--', alpha=0.5)
        ax.add_patch(ellip)

def main():
    max_number = 5  # Should match the max_number in the C++ code
    for run_number in range(1, max_number + 1):
        obstacles_filename = f'obstacles_{run_number}.csv'
        path_filename = f'path_data_rrt_{run_number}.csv'
        if not os.path.exists(obstacles_filename) or not os.path.exists(path_filename):
            print(f"Files for run {run_number} not found.")
            continue

        obstacles = read_obstacles(obstacles_filename)
        path = read_path(path_filename)

        fig, ax = plt.subplots(figsize=(10,10))
        plot_obstacles(obstacles, ax)
        plot_path(path, ax)

        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(f'RRT* Path with Uncertainty Ellipsoids (Run {run_number})')
        ax.legend()
        ax.set_aspect('equal')
        plt.grid(True)
        plt.savefig(f'rrt_path_{run_number}.png')
        plt.close(fig)
        print(f"Figure for run {run_number} saved as 'rrt_path_{run_number}.png'.")

if __name__ == "__main__":
    main()
```

Run the script:

```bash
python3 plot_paths.py
```

### 3. Running the PRM* Plotting Script

Save the following script as `plot_prm_path.py`:

```python
# plot_prm_path.py

import matplotlib.pyplot as plt
import csv
import matplotlib.patches as patches
import numpy as np
import os

def read_obstacles(filename):
    obstacles = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            type_ = row[0]
            num_vertices = int(row[1])
            vertices = []
            verts = row[2:]
            for i in range(0, len(verts), 2):
                x = float(verts[i])
                y = float(verts[i+1])
                vertices.append((x, y))
            obstacles.append((type_, vertices))
    return obstacles

def plot_obstacles(obstacles, ax):
    for obs in obstacles:
        type_, vertices = obs
        polygon = patches.Polygon(vertices, closed=True, fill=True, edgecolor='black', facecolor='gray', alpha=0.5)
        ax.add_patch(polygon)

def read_path(filename):
    path = []
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            P11 = float(row['P11'])
            P12 = float(row['P12'])
            P22 = float(row['P22'])
            path.append((x, y, P11, P12, P22))
    return path

def plot_path(path, ax):
    xs = [state[0] for state in path]
    ys = [state[1] for state in path]
    ax.plot(xs, ys, '-o', color='blue', label='PRM* Path')

    # Plot uncertainty ellipsoids
    for idx, state in enumerate(path):
        x, y, P11, P12, P22 = state
        cov = np.array([[P11, P12],
                        [P12, P22]])

        # Ensure covariance matrix is symmetric
        cov = (cov + cov.T) / 2

        # Eigenvalues and eigenvectors for the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Handle negative eigenvalues
        if np.any(eigenvalues < 0):
            print(f"Negative eigenvalues at index {idx}, position ({x}, {y}): {eigenvalues}")
            # Adjust small negative eigenvalues
            eigenvalues[eigenvalues < 0] = 0.0
            # If eigenvalues are significantly negative, skip plotting
            if np.any(eigenvalues < -1e-6):
                print(f"Significant negative eigenvalues encountered, skipping ellipsoid at index {idx}.")
                continue

        # Proceed with plotting
        order = eigenvalues.argsort()[::-1]
        eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]
        angle = np.degrees(np.arctan2(*eigenvectors[:,0][::-1]))
        width, height = 2 * np.sqrt(eigenvalues)  # 1-sigma ellipse

        # Check for NaN or infinite values
        if not np.isfinite(width) or not np.isfinite(height):
            print(f"Non-finite ellipse dimensions at index {idx}, skipping.")
            continue

        ellip = patches.Ellipse((x, y), width, height, angle,
                                edgecolor='red', facecolor='none', linestyle='--', alpha=0.5)
        ax.add_patch(ellip)

def main():
    obstacles_filename = 'obstacles_prm.csv'
    path_filename = 'path_data_prm.csv'
    if not os.path.exists(obstacles_filename) or not os.path.exists(path_filename):
        print("Required files not found.")
        return

    obstacles = read_obstacles(obstacles_filename)
    path = read_path(path_filename)

    fig, ax = plt.subplots(figsize=(10,10))
    plot_obstacles(obstacles, ax)
    plot_path(path, ax)

    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('PRM* Path with Uncertainty Ellipsoids')
    ax.legend()
    ax.set_aspect('equal')
    plt.grid(True)
    plt.savefig('prm_path.png')
    plt.close(fig)
    print("Figure saved as 'prm_path.png'.")

if __name__ == "__main__":
    main()
```

Run the script:

```bash
python3 plot_prm_path.py
```

---

## Understanding the Tunable Parameters

### 1. C++ Code Parameters

#### a. `max_number` (RRT* Code)

- **Location**: Near the beginning of `rrt_star_with_custom_cost.cpp`.
- **Description**: Determines the number of times the RRT* planner runs.
- **Default Value**: `int max_number = 5;`
- **Usage**: Increase or decrease to run the planner more or fewer times.

#### b. `alpha` (Both RRT* and PRM* Codes)

- **Location**: In the `main()` function of both C++ files.
- **Description**: Weighting factor for the information cost in the custom optimization objective.
- **Default Value**: `double alpha = 0.2;` (RRT*), `double alpha = 0.2;` (PRM*)
- **Usage**: Adjust to give more or less importance to the information cost relative to the travel distance.

#### c. Obstacle Parameters

- **Variables**: `m1`, `m2`, `m3`, `min_obstacle_size`, `max_obstacle_size`
- **Description**:
  - `m1`, `m2`, `m3`: Number of triangles, squares, and pentagons, respectively.
  - `min_obstacle_size`, `max_obstacle_size`: Minimum and maximum sizes of the obstacles.
- **Usage**: Modify to change the complexity of the environment.

#### d. Workspace Bounds

- **Variables**: `workspace_min`, `workspace_max`
- **Description**: Define the boundaries of the workspace.
- **Default Values**: `-5.0` to `5.0`
- **Usage**: Adjust to change the size of the environment.

### 2. Python Plotting Code Parameters

#### a. `max_number` (RRT* Plotting Script)

- **Location**: In `plot_paths.py`.
- **Description**: Should match the `max_number` in the C++ RRT* code.
- **Usage**: Ensure consistency between the number of runs in C++ and the plots generated.

#### b. Plot Aesthetics

- **Variables**: `figsize`, axis limits, colors, labels.
- **Usage**: Customize the appearance of the plots as desired.

---

## Additional Notes

### 1. Common Issues and Solutions

#### a. OMPL Not Found During Compilation

- **Error**: `fatal error: ompl/...: No such file or directory`
- **Solution**: Ensure OMPL is correctly installed and the include paths are set. Use `pkg-config` to include OMPL flags.

#### b. Boost Library Errors

- **Error**: Linking errors related to Boost libraries.
- **Solution**: Ensure Boost is installed (`sudo apt install libboost-all-dev`) and link against necessary Boost libraries during compilation.

#### c. Negative Eigenvalues in Python Scripts

- **Issue**: Runtime warnings or errors due to negative eigenvalues when plotting ellipsoids.
- **Solution**: The scripts handle small negative eigenvalues by setting them to zero. Ensure the C++ code generates valid covariance matrices.

### 2. Tips for Customization

- **Obstacle Shapes and Sizes**: Modify the obstacle generation code to include different shapes or sizes.
- **Planner Parameters**: Adjust the planner settings (e.g., planning time, number of samples) to see different results.
- **Visualization Enhancements**: Add more details to the plots, such as start and goal positions, or save plots in different formats.

---




