import sys
import os
import heapq
import numpy as np
from typing import Set, Tuple, List

sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))
from KC_structs import DiscreteState, ControlSet, Theta
from KC_searching import Map, SearchNode

class VisualizableLBA:
    """
    Standard Lattice-Based A* implementation for visualization comparison.
    """
    def __init__(self, task_map: Map, control_set: ControlSet, theta_config: Theta, R=0, A=0):
        self.map = task_map
        self.cs = control_set
        self.theta = theta_config
        self.R = R
        self.A = A

    def heuristic(self, state, goal):
        return np.hypot(state.i - goal.i, state.j - goal.j)

    def is_goal(self, state, goal):
        dist_sq = (state.i - goal.i)**2 + (state.j - goal.j)**2
        a_dist = self.theta.num_dist(state.theta, goal.theta)
        return (dist_sq <= self.R**2) and (a_dist <= self.A)

    def run(self, start_node, goal_node, visualizer=None, update_freq=10):
        start_wrapper = SearchNode(start_node, g=0.0, h=self.heuristic(start_node, goal_node))
        open_set = []
        heapq.heappush(open_set, (start_wrapper.f, start_wrapper))
        
        closed_set = set()
        g_scores = {start_node: 0.0}
        came_from = {}
        
        # Viz Containers
        vis_open = set([(start_node.i, start_node.j)])
        vis_closed = set()
        vis_segments = []
        vis_invalid = [] # Added for consistency with MeshA*
        
        exp_cnt = 0
        
        while open_set:
            f, node = heapq.heappop(open_set)
            curr = node.vertex
            
            if (curr.i, curr.j) in vis_open: vis_open.remove((curr.i, curr.j))
            vis_closed.add((curr.i, curr.j))
            
            if curr in closed_set: continue
            
            if self.is_goal(curr, goal_node):
                if visualizer:
                    # Pass None for initial_cells
                    visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid, None)
                    path = self._reconstruct(curr, came_from)
                    visualizer.animate_final_path(path)
                return True
                
            closed_set.add(curr)
            exp_cnt += 1
            
            for prim in self.cs.get_prims_heading(curr.theta):
                di, dj = prim.goal.i, prim.goal.j
                new_state = DiscreteState(curr.i + di, curr.j + dj, prim.goal.theta)
                
                # Bounds check
                if not self.map.in_bounds(new_state.i, new_state.j): continue
                
                # Geometry
                geom = (prim.x_coords + curr.j, prim.y_coords + curr.i)
                
                # Collision Check
                ci = prim.collision_in_i + curr.i
                cj = prim.collision_in_j + curr.j
                if self._check_coll(ci, cj):
                    # Record invalid for visualization if desired (optional for LBA)
                    vis_invalid.append(geom)
                    continue
                
                vis_segments.append(geom)
                
                tent_g = g_scores[curr] + prim.length
                if new_state not in g_scores or tent_g < g_scores[new_state]:
                    g_scores[new_state] = tent_g
                    new_node = SearchNode(new_state, g=tent_g, h=self.heuristic(new_state, goal_node))
                    heapq.heappush(open_set, (new_node.f, new_node))
                    came_from[new_state] = (curr, geom)
                    
                    vis_open.add((new_state.i, new_state.j))
                    if (new_state.i, new_state.j) in vis_closed:
                        vis_closed.remove((new_state.i, new_state.j))

            if visualizer and (exp_cnt % update_freq == 0):
                visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid, None)
                vis_segments = []
                vis_invalid = []

        return False

    def _check_coll(self, ci, cj):
        for i, j in zip(ci, cj):
            if not self.map.traversable(i, j): return True
        return False
        
    def _reconstruct(self, curr, came_from):
        path = []
        while curr in came_from:
            prev, geom = came_from[curr]
            path.append(geom)
            curr = prev
        path.reverse()
        return path
