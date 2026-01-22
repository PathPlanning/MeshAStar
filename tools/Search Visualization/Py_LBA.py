import sys
import os
import heapq
import numpy as np
from typing import Set, Tuple, List, Optional

sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))
from KC_structs import DiscreteState, ControlSet, Theta
from KC_searching import Map, SearchNode

class VisualizableLBA:
    """
    Standard Lattice-Based A* (LBA*) implementation with integrated visualization hooks.
    
    This class implements a state-lattice planner that searches a graph where edges 
    correspond to kinodynamically feasible motion primitives. It serves as a baseline 
    algorithm for comparing performance and search behavior against MeshA*.
    
    Key Features:
    - State Space: (x, y, theta) discrete configurations.
    - Edges: Pre-computed motion primitives connecting discrete states.
    - Visualization: Real-time rendering of the OPEN/CLOSED sets and expanded primitives.
    """
    
    def __init__(self, task_map: Map, control_set: ControlSet, theta_config: Theta, R: float = 0, A: int = 0):
        """
        Initialize the LBA* planner.
        
        Args:
            task_map: The grid-based environment map.
            control_set: The set of pre-computed motion primitives.
            theta_config: Discretization configuration for heading angles.
            R: Goal region radius (Euclidean distance).
            A: Goal region angular tolerance (discrete steps).
        """
        self.map = task_map
        self.cs = control_set
        self.theta = theta_config
        self.R = R
        self.A = A

    def heuristic(self, state: DiscreteState, goal: DiscreteState) -> float:
        """
        Computes the admissible heuristic h(n).
        
        Uses Euclidean distance to the goal position (ignoring heading).
        This is a standard admissible heuristic for 2D kinematic planning.
        """
        return np.hypot(state.i - goal.i, state.j - goal.j)

    def is_goal(self, state: DiscreteState, goal: DiscreteState) -> bool:
        """
        Checks if a state satisfies the goal condition.
        
        The goal is defined as a region within radius R and angular tolerance A 
        from the target configuration.
        """
        dist_sq = (state.i - goal.i)**2 + (state.j - goal.j)**2
        a_dist = self.theta.num_dist(state.theta, goal.theta)
        return (dist_sq <= self.R**2) and (a_dist <= self.A)

    def run(self, start_node: DiscreteState, goal_node: DiscreteState, 
            visualizer=None, update_freq: int = 10) -> bool:
        """
        Executes the A* search on the lattice graph.
        
        Args:
            start_node: The initial configuration.
            goal_node: The target configuration.
            visualizer: An instance of LiveVisualizer for real-time rendering.
            update_freq: Frequency of visualization updates (every N expansions).
            
        Returns:
            True if a path is found, False otherwise.
        """
        # Initialize search node with f = g + h
        start_wrapper = SearchNode(start_node, g=0.0, h=self.heuristic(start_node, goal_node))
        
        # Priority Queue for OPEN set: stores (f-score, SearchNode)
        open_set = []
        heapq.heappush(open_set, (start_wrapper.f, start_wrapper))
        
        closed_set = set()
        g_scores = {start_node: 0.0}
        
        # Path reconstruction map: State -> (Parent State, Primitive Geometry)
        came_from = {}
        
        # Visualization Containers
        # vis_open/closed: Sets of (i, j) coordinates for highlighting grid cells.
        # vis_segments: List of primitive geometries (valid) to be drawn (Blue lines).
        # vis_invalid: List of primitive geometries (collision) to be drawn (Dashed Grey lines).
        vis_open = set([(start_node.i, start_node.j)])
        vis_closed = set()
        vis_segments = []
        vis_invalid = [] 
        
        exp_cnt = 0
        
        while open_set:
            f, node = heapq.heappop(open_set)
            curr = node.vertex
            
            # Update visualization sets
            if (curr.i, curr.j) in vis_open: vis_open.remove((curr.i, curr.j))
            vis_closed.add((curr.i, curr.j))
            
            if curr in closed_set: continue
            
            # Goal Check
            if self.is_goal(curr, goal_node):
                if visualizer:
                    # Final update to show the complete search state
                    # Note: LBA* does not use 'initial_cells' (Magenta), so we pass None.
                    visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid, None)
                    
                    # Reconstruct and animate the optimal path
                    path = self._reconstruct(curr, came_from)
                    visualizer.animate_final_path(path)
                return True
                
            closed_set.add(curr)
            exp_cnt += 1
            
            # Expansion: Iterate over all applicable motion primitives
            for prim in self.cs.get_prims_heading(curr.theta):
                di, dj = prim.goal.i, prim.goal.j
                new_state = DiscreteState(curr.i + di, curr.j + dj, prim.goal.theta)
                
                # Compute absolute geometry of the primitive for visualization/collision
                geom = (prim.x_coords + curr.j, prim.y_coords + curr.i)

                # 1. Bounds Check
                if not self.map.in_bounds(new_state.i, new_state.j):
                    vis_invalid.append(geom)
                    continue                
                
                # 2. Collision Check (Swathe)
                # Check all swept cells along the primitive
                ci = prim.collision_in_i + curr.i
                cj = prim.collision_in_j + curr.j
                if self._check_coll(ci, cj):
                    vis_invalid.append(geom)
                    continue
                
                # If valid, add to visualization buffer (Blue lines)
                vis_segments.append(geom)
                
                # 3. Path Relaxation (G-score update)
                tent_g = g_scores[curr] + prim.length
                if new_state not in g_scores or tent_g < g_scores[new_state]:
                    g_scores[new_state] = tent_g
                    new_node = SearchNode(new_state, g=tent_g, h=self.heuristic(new_state, goal_node))
                    heapq.heappush(open_set, (new_node.f, new_node))
                    came_from[new_state] = (curr, geom)
                    
                    vis_open.add((new_state.i, new_state.j))
                    if (new_state.i, new_state.j) in vis_closed:
                        vis_closed.remove((new_state.i, new_state.j))

            # Live Visualization Update
            if visualizer and (exp_cnt % update_freq == 0):
                # Update the plotter with the accumulated valid/invalid segments and cell states
                visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid, None)
                # Clear buffers after rendering to avoid memory growth/duplicate drawing
                vis_segments = []
                vis_invalid = []

        return False

    def _check_coll(self, ci: np.ndarray, cj: np.ndarray) -> bool:
        """
        Checks collision for a sequence of grid cells.
        Returns True if ANY cell in the sequence is blocked.
        """
        for i, j in zip(ci, cj):
            if not self.map.traversable(i, j): return True
        return False
        
    def _reconstruct(self, curr: DiscreteState, came_from: dict) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Reconstructs the path geometry from start to goal.
        """
        path = []
        while curr in came_from:
            prev, geom = came_from[curr]
            path.append(geom)
            curr = prev
        path.reverse()
        return path
