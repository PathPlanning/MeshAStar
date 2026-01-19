"""
This file defines the core data structures required for kinematically consistent trajectory planning:

    State:           Continuous state of the mobile agent (x, y, theta, k).
    DiscreteState:   Discrete state of the agent (= node in the State Lattice graph).
    ShortTrajectory: Class for generating and storing a short feasible trajectory (primitive candidates).
    Primitive:       A generated motion primitive (fixed trajectory between two states).
    Theta:           Discrete heading angle logic and discretization.
    ControlSet:      A collection of generated primitives starting from (0,0, theta_l).
"""

import numpy as np
from scipy.integrate import quad
import copy

# Type hinting imports to ensure code clarity and reduce errors.
from typing import List, Tuple, Optional

# 'Self' type is available in typing module since Python 3.11. 
# For older versions, we use typing_extensions.
try:
    from typing import Self
except ImportError:
    from typing_extensions import Self 


class State:
    """
    Represents the 4-dimensional continuous state of a mobile agent: 
    coordinates (x, y), heading angle (theta), and curvature (k).
    """
    
    def __init__(self, x: float, y: float, theta: float, k: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.k = k


class DiscreteState:
    """
    Represents a discrete state: two integer coordinates (i, j) and a discrete heading index (theta).
    Discrete states correspond to nodes in the State Lattice graph and necessarily have zero curvature 
    (by definition of the discretization used in this work).
    """
    
    def __init__(self, i: int, j: int, theta: int) -> None:
        self.i = i
        self.j = j
        self.theta = theta
        
    def __eq__(self, other: 'DiscreteState') -> bool:
        """
        Equality check function. This is essential for using DiscreteState as a key or element 
        in the CLOSED set (implemented as a hash set or dict).
        
        Args:
            other: Another DiscreteState instance to compare with.
            
        Returns:
            bool: True if all components (i, j, theta) are identical.
        """
        return (self.i == other.i) and (self.j == other.j) and (self.theta == other.theta)
       
    def __hash__(self) -> int:
        """
        Hashing function. Returns an integer used for addressing the DiscreteState in a hash table 
        (e.g., the CLOSED set).
        
        Distinct states (where __eq__ is False) should ideally have distinct hashes to minimize collisions 
        and maximize hash table performance. Equal states must have identical hashes.
        """
        return hash((self.i, self.j, self.theta))
        

class ShortTrajectory:
    """
    Class for storing and generating a short feasible trajectory between two states.
    
    A short trajectory is defined by a fixed 'start' state and a set of internal parameters.
    (Two parametrizations are used, as described in the thesis).
    We also store a 'goal' state, which is the ideal target. The optimization process (Newton's method)
    adjusts the parameters so that the actual 'final_state' of the trajectory matches the 'goal'.
    """
    
    def __init__(self, start: State, goal: State) -> None:
        """
        Initialization.
        
        Args:
            start: The initial state from which the trajectory originates.
            goal: The target state the trajectory aims to reach.
        """
        # Fixed endpoints
        self.start = start
        self.goal = goal
        self.k0 = self.start.k  # Initial curvature is fixed by the start state
        
        # Parametrization 1: Polynomial curvature coefficients (a, b, c) and length
        self.a = None
        self.b = None
        self.c = None
        self.length = None

        # Parametrization 2: Curvature values at specific points and log_length
        self.log_length = None
        self.k1 = None
        self.k2 = None
        self.kf = None

        # Vectorize coordinate functions to allow batch processing of 's' (arc length) values
        self.vect_x = np.vectorize(self.x)
        self.vect_y = np.vectorize(self.y)
        

    def set_coef_params(self, a: float, b: float, c: float, length: float) -> Self:
        """
        Sets trajectory parameters using the First (1) Parametrization.
        
        Args:
            a, b, c: Polynomial coefficients determining curvature profile.
            length: Total length of the trajectory.
        """
        assert length >= 0, "Length cannot be negative!"
        
        # Set params for parametrization 1
        self.length = length  
        self.a = a
        self.b = b
        self.c = c
        
        # Calculate params for parametrization 2 (derived from 1)
        self.log_length = np.log(length)
        self.k1 = self.k(1/3 * length)
        self.k2 = self.k(2/3 * length)
        self.kf = self.goal.k  # Convenience of param 2: final curvature is explicitly the goal curvature

        return self
        

    def set_curve_params(self, k1: float, k2: float, log_length: float) -> Self:
        """
        Sets trajectory parameters using the Second (2) Parametrization.
        This parametrization is used by the Newton optimizer.
        
        Args:
            k1, k2: Curvature values at 1/3 and 2/3 of the length.
            log_length: Natural logarithm of the total length.
        """
        # Set params for parametrization 2
        self.log_length = log_length
        self.k1 = k1
        self.k2 = k2
        self.kf = self.goal.k
        
        # Calculate params for parametrization 1 (Polynomial coefficients)
        # We solve a linear system to find coefficients a, b, c that satisfy curvatures k0, k1, k2, kf.
        self.length = np.exp(log_length)
        vect_s = np.array([0, 1/3 * self.length, 2/3 * self.length, self.length]).reshape(-1, 1)
        # Construct the Vandermonde-like matrix for the cubic polynomial
        mat_s = np.hstack((vect_s ** 0, vect_s ** 1, vect_s ** 2, vect_s ** 3)) 
        
        # Solve for [k0, a, b, c]
        params = np.linalg.inv(mat_s) @ np.array([self.k0, self.k1, self.k2, self.kf])
        
        k0_check, a, b, c = params
        self.a, self.b, self.c = a, b, c
        
        # Sanity check: the calculated k0 must match the fixed start curvature
        assert abs(k0_check - self.k0) < 1e-4, "Error in parametrization conversion: calculated k0 mismatch."
                                                                                               
        return self

    """
    Given a fixed start state and fixed parameters (First Parametrization), the trajectory functions 
    are uniquely defined. All functions depend on 's' (arc length from start):
        Curvature k(s): Cubic polynomial.
        Heading theta(s): Quartic polynomial (integral of curvature).
        Coordinates x(s), y(s): Integrals of cos(theta) and sin(theta).
    """
    
    def k(self, s: float) -> float:
        """Returns curvature at arc length s."""
        assert s >= 0, "Parameter s must be non-negative!"
        return self.k0 + self.a * s + self.b * s**2 + self.c * s**3
    
    def theta(self, s: float) -> float:
        """Returns heading angle at arc length s."""
        assert s >= 0, "Parameter s must be non-negative!"
        # Derived by integrating k(s)
        theta0, k0 = self.start.theta, self.k0
        a, b, c = self.a, self.b, self.c
        return theta0 + k0 * s + a/2 * s**2 + b/3 * s**3 + c/4 * s**4

    def x(self, s: float) -> float:
        """Returns x-coordinate at arc length s (via numerical integration)."""
        assert s >= 0, "Parameter s must be non-negative!"
        x0 = self.start.x
        return x0 + quad(lambda x: np.cos(self.theta(x)), 0, s, limit=200, limlst=10)[0]
    
    def y(self, s: float) -> float:
        """Returns y-coordinate at arc length s (via numerical integration)."""
        assert s >= 0, "Parameter s must be non-negative!"
        y0 = self.start.y
        return y0 + quad(lambda x: np.sin(self.theta(x)), 0, s, limit=200, limlst=10)[0]

    def total_heading_change(self) -> float:
        """
        Calculates the total accumulated change in heading angle along the trajectory.
        Dividing this value by the length yields the AOL (Angle-Over-Length) metric, 
        used to evaluate path "wobbliness" (see Bench-MR: https://eric-heiden.com/publication/2021-benchmr-ral-icra/2021-benchmr-ral-icra.pdf).
        
        The change in angle at point s is |theta(s+ds) - theta(s)| approx |theta'(s)| ds.
        The total change is the integral: Integral(|theta'(s)| ds).
        Since theta'(s) = k(s), we compute: Integral(|k(s)|, 0, length).
        """ 
        return quad(lambda s: abs(self.k(s)), 0, self.length, limit=200, limlst=10)[0]
    
    # Sampling functions for visualization
    def sample_x(self, ds: float = 0.02) -> np.ndarray:
        num = int(self.length / ds)
        return self.vect_x(np.linspace(0, self.length, num=num, endpoint=True))
    
    def sample_y(self, ds: float = 0.02) -> np.ndarray:
        num = int(self.length / ds)
        return self.vect_y(np.linspace(0, self.length, num=num, endpoint=True))

    def state(self, s: float) -> State:
        """
        Returns the full agent State at arc length s.
        The trajectory is effectively a function mapping arc length s to the state space.
        """
        return State(self.x(s), self.y(s), self.theta(s), self.k(s))

    def final_state(self) -> State:
        """
        Returns the state at the end of the trajectory.
        The goal of the optimization is to make final_state() == self.goal.
        """
        return self.state(self.length)
    
    
class Primitive:
    """
    Class representing a generated motion primitive.
    A primitive is a fixed, feasible short trajectory connecting two discrete states.
    """
    
    def __init__(self, start: DiscreteState, goal: DiscreteState) -> None:
        """
        Args:
            start: Discrete start state.
            goal: Discrete goal state.
        """
        self.start = start
        self.goal = goal
        
    def set_description(self, x_coords: np.ndarray, y_coords: np.ndarray, length: float, 
                              i_coords: np.ndarray, j_coords: np.ndarray, turning: int) -> Self:
        """
        Stores detailed information about the primitive.
        
        Args:
            x_coords, y_coords: Sampled points along the curve (for visualization).
            length: Arc length.
            i_coords, j_coords: Grid cells (collision footprint) traversed by the primitive.
            turning: Discrete heading change (number of angular steps between start and goal headings).
            
        Note on Coordinate Systems:
        The primitive generation assumes Y-axis points UP. Thus, i-coordinates in collision arrays
        increase upwards. When mapping to a matrix-based grid (where row index increases downwards),
        primitives may need to be flipped or coordinates adjusted accordingly.
        """
        assert x_coords.shape == y_coords.shape, "Mismatch in coordinate array shapes."
        assert i_coords.shape == j_coords.shape, "Mismatch in collision array shapes."
        assert (i_coords[0] == self.start.i) and (j_coords[0] == self.start.j), "Collision trace must start at the primitive's origin."
        
        self.x_coords = x_coords
        self.y_coords = y_coords
        self.length = length
        self.collision_in_i = i_coords
        self.collision_in_j = j_coords
        self.turning = turning
        self.U = len(i_coords)
        assert self.U >= 2, "Primitive collision trace must contain at least 2 cells."
        
        return self
    
    def set_id(self, id: int) -> None:
        """Sets a unique numeric identifier for the primitive."""
        self.id = id
        
    def __eq__(self, other: 'Primitive') -> bool:
        """
        Equality check. We assume there is a unique optimal primitive between any two specific discrete states.
        Therefore, two primitives are equal if and only if their start and goal states are identical.
        """
        return (self.start == other.start) and (self.goal == other.goal)
    
    def __hash__(self) -> int:
        """
        Hash function based on start and goal configuration.
        Allows using Primitive as a dictionary key.
        """
        return hash((self.start.i, self.start.j, self.start.theta, self.goal.i, self.goal.j, self.goal.theta))
    

class ControlSet:
    """
    Manages a collection of generated primitives (the Control Set).
    
    Concept:
    All primitives in this set are normalized to start at the discrete state (0, 0, theta_l).
    To use a primitive in the actual search graph (State Lattice), we find the corresponding 
    template here and 'translate' (shift) it to the actual start coordinates (i, j).
    """
    
    def __init__(self, theta_discretization: 'Theta', max_id=1000) -> None:
        """
        Args:
            theta_discretization: The Theta instance defining angular resolution.
            max_id: Size of the lookup table for accessing primitives by ID.
        """
        self.theta = theta_discretization
        self.theta_amount = theta_discretization.theta_amount
        self.control_set = []   # List of lists. control_set[theta] holds primitives starting at angle theta.
        self.list_all_prims = [None] * max_id

        for i in range(self.theta_amount):
            self.control_set.append([])

    def get_prims_heading(self, heading: int) -> List[Primitive]:
        """Returns the list of primitives originating from a specific discrete heading."""
        heading %= self.theta_amount
        return self.control_set[heading]
    
    def get_prim_between_states(self, start: DiscreteState, goal: DiscreteState) -> Primitive:
        """
        Retrieves (constructs via translation) the primitive connecting two specific states.
        This effectively returns an edge in the State Lattice graph.
        
        Logic:
        1. Calculate relative offset (di, dj) = goal - start.
        2. Find a template primitive in control_set starting at start.theta and ending at (di, dj, goal.theta).
        3. Create a copy of that template and shift its coordinates by (start.i, start.j).
        """
        
        # Relative coordinates if start was at (0,0)
        di, dj = goal.i - start.i, goal.j - start.j
        
        # Iterate over candidates starting with the correct heading
        for prim in self.get_prims_heading(start.theta):
            # Check if this primitive reaches the relative goal state
            if (self.theta.num_dist(goal.theta, prim.goal.theta) == 0 and
                prim.goal.i == di and
                prim.goal.j == dj):
                
                # Create the specific primitive instance
                find_prim = Primitive(start, goal)
                
                # Apply translation (shift) to geometry and collision footprint
                # Note: i-axis corresponds to y-axis in this generation scheme
                x_coords = copy.deepcopy(prim.x_coords) + start.j
                y_coords = copy.deepcopy(prim.y_coords) + start.i            
                
                collision_in_i, collision_in_j = [], []
                for i, j in zip(prim.collision_in_i, prim.collision_in_j):
                    collision_in_i.append(i + start.i)
                    collision_in_j.append(j + start.j)
                
                find_prim.set_description(x_coords, y_coords, prim.length,
                                          np.array(collision_in_i), np.array(collision_in_j), prim.turning)
                return find_prim
        
        raise Exception(f"Primitive not found! Request: {start.i, start.j, start.theta} -> {goal.i, goal.j, goal.theta}")
    

    def load_primitives(self, file: str) -> Self:    
        """
        Parses primitives from a text file and populates the control set.
        The file format must match the output of primitive_saver.py.
        """  
        with open(file, "r") as f:
            curr_prim_x = []
            curr_prim_y = []
            i_collisions = []
            j_collisions = []
            theta = None
            goal = None
            length = None
            turning = None
            id = None
            
            while True:
                line = f.readline()
                if line == "":
                    break
                
                if "===== prim description: =====" in line:
                    curr_prim_x.clear()
                    curr_prim_y.clear()
                    i_collisions.clear()
                    j_collisions.clear()
                    continue

                if "start heading (number):" in line:
                    theta = int(line.split(": ")[1])
                    
                if "goal state (i, j, heading num):" in line:
                    p = list(map(int, (line.split(": ")[1]).split(" ")))
                    goal = DiscreteState(*p)

                if "length is:" in line:
                    length = float(line.split(": ")[1])

                if "turning on:" in line:
                    turning = int(line.split(": ")[1])

                if "trajectory is:" in line:
                    while True:
                        line = f.readline()
                        if "---" in line:
                            break
                        curr_prim_x.append(float(line.split()[0]))
                        curr_prim_y.append(float(line.split()[1]))

                if "collision is:" in line:
                    while True:
                        line = f.readline()
                        if "---" in line:
                            break
                        i_collisions.append(int(line.split()[0]))
                        j_collisions.append(int(line.split()[1]))
                        
                if "prim ID is:" in line:
                    id = int(line.split(": ")[1])
                        
                if "prim end" in line:
                    # Create the control set template (starts at 0,0)
                    prim = Primitive(DiscreteState(0, 0, theta), goal)
                    prim.set_description(np.array(curr_prim_x), np.array(curr_prim_y), length,
                                         np.array(i_collisions), np.array(j_collisions), turning)
                    prim.set_id(id)
                    
                    self.control_set[theta].append(prim)
                    if id < len(self.list_all_prims):
                        self.list_all_prims[id] = prim
                    else:
                        # Dynamic resize if max_id was too small
                        self.list_all_prims.extend([None] * (id - len(self.list_all_prims) + 1))
                        self.list_all_prims[id] = prim

        return self
    
        
class Theta:
    """
    Handles heading angle discretization (specifically 16 directions).
    Includes logic for non-uniform spacing (e.g., arctan(0.5) for grid transitions).
    """
    
    def __init__(self) -> None:
        self.theta_amount = 16
        self.theta_16 = np.zeros(16)
        
        # Standard 45-degree increments
        for i in range(0, 16, 2):
            self.theta_16[i] = np.deg2rad(i / 16 * 360)
            
        # Specific angles for connecting (0,0) to (1,2) and (2,1) neighbors
        self.theta_16[1] = np.arctan(1 / 2)
        self.theta_16[3] = np.arctan(2)
        
        # Symmetries
        self.theta_16[5] = self.theta_16[1] + np.pi / 2
        self.theta_16[7] = self.theta_16[3] + np.pi / 2
        self.theta_16[9] = self.theta_16[1] + np.pi
        self.theta_16[11] = self.theta_16[3] + np.pi
        self.theta_16[13] = self.theta_16[5] + np.pi
        self.theta_16[15] = self.theta_16[7] + np.pi 
        
        # Normalize all to [-pi, pi)
        for i in range(16):
            self.theta_16[i] = self.correct_angle(self.theta_16[i])

        self.angles = self.theta_16
         
    def correct_angle(self, angle: float) -> float:
        """
        Normalizes an angle to the interval [-pi, pi).
        """
        angle %= (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        return float(angle)
    
    def __getitem__(self, ind: int) -> float:
        """Allows array-like access: theta_obj[i] returns the i-th discrete angle."""
        ind = ind % 16
        return self.theta_16[ind]

    def num_angle(self, angle: float) -> int:
        """
        Maps a continuous angle to the nearest discrete heading index (0-15).
        Raises an exception if the angle does not match any discrete value (within tolerance).
        """
        EPS = 1e-6
        angle = self.correct_angle(angle)
        
        for i in range(16): 
            if (angle - EPS <= self[i] <= angle + EPS):      
                return i
        else:
            raise Exception(f"Angle {angle} is not a valid discrete heading!")
        
    def dist(self, angle1: float, angle2: float) -> int:
        """
        Calculates the shortest distance (in number of steps) between two continuous angles 
        that correspond to discrete headings.
        """
        l1 = self.num_angle(angle1)
        l2 = self.num_angle(angle2)
        return min(abs(l1 - l2), 16 - abs(l1 - l2))
    
    def num_dist(self, l1: int, l2: int) -> int:
        """
        Calculates the shortest distance (in number of steps) between two discrete heading indices.
        """
        l1 %= 16
        l2 %= 16
        return min(abs(l1 - l2), 16 - abs(l1 - l2))
