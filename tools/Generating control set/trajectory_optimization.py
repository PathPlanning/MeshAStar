"""
This module implements the multidimensional Newton's method for motion primitive generation 
(trajectory parameter optimization).

General logic:
We instantiate a ShortTrajectory class with fixed desired 'start' and 'goal' states.
For these fixed states, the trajectory is determined by exactly three parameters: 
k1, k2, and log_length.

These parameters need to be optimized. We define a 'get_residual' function that returns 
three numerical values (a 3D vector) representing the difference between the components 
(x, y, theta) of the desired 'goal' state and the current 'final_state' of the trajectory 
generated with the current k1, k2, log_length.

The objective is to drive these residual values to zero (the closer to zero, the more 
accurately the trajectory reaches the goal). Since we have three parameters (k1, k2, log_length) 
determining three residual values, we can use the multidimensional Newton's method. 
It iteratively adjusts k1, k2, and log_length to minimize the residual vector.

Thus, we generate a valid short trajectory between fixed start and goal states.
"""

import numpy as np
import sys
# Adjust path to point to tools/common/
sys.path.append("../common/") 
from KC_structs import *

def get_residual(traj: ShortTrajectory) -> np.ndarray:
    """
    Calculates the residual function: the component-wise difference between the desired 
    'goal' state and the current 'final_state' of the trajectory.
    """
    
    final = traj.final_state()  # Current end state of the trajectory
    return np.array([traj.goal.x - final.x,
                     traj.goal.y - final.y,
                     traj.goal.theta - final.theta])   # Component-wise difference

def calc_Jacobian_matrix(traj: ShortTrajectory, params: np.ndarray, dk: float = 0.001, dl: float = 0.001) -> np.ndarray:
    """
    Calculates the 3x3 Jacobian matrix for the get_residual function, considered as a function 
    of the trajectory parameters (k1, k2, log_length).
    """
    
    k1, k2, log_length = params  # Current parameters
    # Create a copy of the trajectory (preserving start/goal) to avoid corrupting the original instance
    traj = ShortTrajectory(traj.start, traj.goal)  
    
    # --- Partial derivatives w.r.t k1 ---
    dF_p = get_residual(traj.set_curve_params(k1+dk, k2, log_length))  # Residual with shifted k1
    dF_m = get_residual(traj.set_curve_params(k1-dk, k2, log_length))
    # Finite difference: f'(x) ~= (f(x+h) - f(x-h)) / 2h
    grad_k1 = (dF_p - dF_m) / (2 * dk)  

    # --- Partial derivatives w.r.t k2 ---
    dF_p = get_residual(traj.set_curve_params(k1, k2+dk, log_length))
    dF_m = get_residual(traj.set_curve_params(k1, k2-dk, log_length))
    grad_k2 = (dF_p - dF_m) / (2 * dk)

    # --- Partial derivatives w.r.t log_length (S) ---
    dF_p = get_residual(traj.set_curve_params(k1, k2, log_length+dl))
    dF_m = get_residual(traj.set_curve_params(k1, k2, log_length-dl))
    grad_S = (dF_p - dF_m) / (2 * dl)

    # Stack column vectors to form the matrix
    return np.hstack((grad_k1.reshape(-1, 1), grad_k2.reshape(-1, 1), grad_S.reshape(-1, 1)))

def optimization_Newton(start: State, goal: State, iters: int = 2000, eps: float = 1e-2, lr: float = 0.03, redraw_trajectory = None) -> ShortTrajectory:
    """
    Multidimensional Newton's method loop to find optimal trajectory parameters.

    Args:
        start, goal: The states to connect with a short feasible trajectory.
        iters: Maximum number of iterations (N_{max}).
        eps: Tolerance threshold for the residual norm. If reached, the trajectory 
             is considered to have reached the goal sufficiently well.
        lr: Learning rate.
        redraw_trajectory: Optional callback function for online visualization of the generation process.

    Returns:
        tuple: (number of steps taken, resulting ShortTrajectory object) or None if failed.
    """
    
    traj = ShortTrajectory(start, goal)   # Initialize trajectory
    params = np.array([0.0, 0.0, 0.0])    # Initial guess for parameters: k1=0, k2=0, log_length=0

    steps = 0
    for i in range(iters):
        steps += 1
        # Calculate current residual by setting parameters to the trajectory
        curr_diff = get_residual(traj.set_curve_params(*params))
        
        # Calculate Jacobian at current point
        J = calc_Jacobian_matrix(traj, params)
        
        # Newton step: params = params - lr * J^(-1) * residual
        # We aim to zero out 'curr_diff'
        try:
            params -= lr * np.linalg.inv(J) @ curr_diff
        except np.linalg.LinAlgError:
            print("Singular matrix encountered in Newton method.")
            return None

        # Check convergence
        if np.sum(curr_diff ** 2) ** 0.5 <= eps:
            break
            
        if redraw_trajectory:
            redraw_trajectory(traj, i)  # Visualize current state

    else:
        print("Could not find trajectory! Newton's method did not converge.")
        return None

    return steps, traj.set_curve_params(*params)  # Return success
