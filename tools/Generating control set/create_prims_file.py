"""
This module contains functions for saving generated primitives to a file.
The resulting file can be loaded by the C++ planner (via ControlSet::load_primitives)
and used for search.

It also handles the generation of symmetrical primitives (rotations by 90 degrees).
"""

from typing import List, Tuple, Optional
import copy  # copy.deepcopy allows deep copying of class instances including internal structures
import matplotlib
import sys
import numpy as np

# Adjust path to point to tools/common/
sys.path.append("../common/")
from KC_structs import *
from KC_graphics import *

def get_prim_collision(traj: ShortTrajectory) -> List[Tuple[int, int]]:
    """
    Calculates the collision footprint of the trajectory (collision trace).
    Returns a list of grid cell coordinates (i, j) that the trajectory passes through.
    
    The cells are listed in the order they are traversed by the trajectory 
    (sorted by their occurrence time along the curve).
    """
    
    # We use a dictionary to store (i,j) pairs. 
    # Dict keys are unique, and crucially, modern Python preserves insertion order.
    ij_coord = {}  
    
    # Sample points along the trajectory with a very fine step to ensure no cell is missed
    for x, y in zip(traj.sample_x(ds=0.01), traj.sample_y(ds=0.01)):
        i = int(round(y, 0))  # Round to nearest integer to get grid row index
        j = int(round(x, 0))  # Round to nearest integer to get grid column index
        ij_coord[(i, j)] = (i, j)  # Add to footprint (duplicates are automatically handled)
        
    return ij_coord.values() 

def save_primitive(file: str, prim: ShortTrajectory, theta_discrete: Theta) -> None: 
    """
    Appends a single primitive to a text file in the specific format required by the planner.
    """
    
    # Initialize static counter if it doesn't exist
    if not hasattr(save_primitive, "prim_id"):
        save_primitive.prim_id = 0
    else:
        save_primitive.prim_id += 1
    
    # Open in APPEND mode ('a') to avoid overwriting previous primitives
    with open(file, "a") as f:  
        f.write(f"===== prim description: =====\n")

        # 1. Start Heading (Discrete index)
        # We don't write coordinates because ControlSet primitives always start at (0,0).
        f.write(f"start heading (number): {theta_discrete.num_angle(prim.start.theta)}\n") 
        
        # 2. Goal State (Discrete indices)
        # Convert continuous coordinates to grid indices (i, j). 
        # Note: i corresponds to y (row), j corresponds to x (col).
        f.write(f"goal state (i, j, heading num): {int(prim.goal.y)} {int(prim.goal.x)} {theta_discrete.num_angle(prim.goal.theta)}\n")

        # 3. Metrics
        f.write(f"length is: {prim.length}\n")
        f.write(f"turning on: {theta_discrete.dist(prim.start.theta, prim.goal.theta)}\n") # Discrete heading difference
        f.write(f"total heading change: {prim.total_heading_change()}\n")
        f.write(f"prim ID is: {save_primitive.prim_id}\n")
        
        # 4. Trajectory geometry (x, y)
        # Used for visualization or drawing. Sampled at a coarser step (e.g., 0.1).
        f.write(f"trajectory is:\n")  
        for x, y in zip(prim.sample_x(ds=0.1), prim.sample_y(ds=0.1)):
            f.write(f"{x} {y}\n")  
        f.write(f"---\n")

        # 5. Collision footprint (i, j)
        f.write(f"collision is:\n")
        for i, j in get_prim_collision(prim):
            f.write(f"{i} {j}\n")
        f.write(f"---\n")

        f.write("prim end\n")
        
        
def save_and_show(file: str, prim: ShortTrajectory, theta_discrete: Theta, 
                  central_sym: bool = True, show: bool = True, ax: Optional[matplotlib.axes.Axes] = None):
    """
    Wrapper function to save a primitive and optionally its symmetrical variations.
    Can also visualize the primitives on a plot.

    Args:
        file: Output file path.
        prim: The base primitive (ShortTrajectory).
        theta_discrete: Discretization logic.
        central_sym: If True, generates and saves 3 additional versions rotated by 
                     +90, +180, and +270 degrees (rotational symmetry).
        show: If True, plots the primitives using KC_graphics.
        ax: Matplotlib axes for plotting.
        
    Note: If show=True and central_sym=True, 4 primitives will be drawn in 
    Red (base), Green (+90), Blue (+180), Yellow (+270).
    """
    
    # Cache goal parameters
    x, y, theta, k = prim.goal.x, prim.goal.y, prim.goal.theta, prim.goal.k
    
    # Work with a deep copy to preserve the original object
    current_prim = copy.deepcopy(prim)
    
    # 1. Save Base Primitive (0 degrees)
    save_primitive(file, current_prim, theta_discrete)
    if show: show_trajectory(current_prim, col="r", ax=ax)
        
    if central_sym:
        # 2. Rotate +90 degrees (Red -> Green)
        # Rotate start heading
        current_prim.start.theta += np.pi/2
        # Rotate goal state: (x, y) -> (-y, x)
        current_prim.goal = State(-y, x, theta+np.pi/2, k)
        
        # Note: Internal parameters (length, curvature profile) do not change with rotation.
        # Only start/goal states change. The sample_x/sample_y functions will respect the new start angle.
        save_primitive(file, current_prim, theta_discrete)
        if show: show_trajectory(current_prim, col="g", ax=ax)
            
        # 3. Rotate +180 degrees (Green -> Blue)
        current_prim.start.theta += np.pi/2 
        current_prim.goal = State(-x, -y, theta+2*np.pi/2, k) # (x, y) -> (-x, -y)
        save_primitive(file, current_prim, theta_discrete)
        if show: show_trajectory(current_prim, col="b", ax=ax)
            
        # 4. Rotate +270 degrees (Blue -> Yellow)
        current_prim.start.theta += np.pi/2
        current_prim.goal = State(y, -x, theta+3*np.pi/2, k) # (x, y) -> (y, -x)
        save_primitive(file, current_prim, theta_discrete)
        if show: show_trajectory(current_prim, col="y", ax=ax)
