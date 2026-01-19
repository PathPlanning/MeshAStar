import sys
import os
import heapq
import numpy as np
from typing import List, Tuple, Dict, Set, Optional

sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))
from KC_structs import DiscreteState, ControlSet, Theta
from KC_searching import Map

class MeshInfo:
    """
    Parses and stores the topological structure of the Mesh Graph.
    
    The Mesh Graph defines the connectivity between Extended Cells u = (i, j, Psi).
    This class loads the transition table, heuristic lookups (Finals), and 
    configuration mappings from the pre-computed file.
    """
    def __init__(self, filepath: str, control_set: ControlSet):
        self.filepath = filepath
        self.control_set = control_set
        
        # Mapping: Discrete Heading (0..15) <-> Initial Configuration ID
        self.initial_config_by_theta: Dict[int, int] = {}
        self.theta_by_initial_config: Dict[int, int] = {}
        
        # Transition Table: ConfigID -> List of Successors
        self.successors: Dict[int, List[Tuple[int, int, int, int]]] = {}
        
        # Primitives within Configuration: ConfigID -> List of Primitives (for Heuristic/Pruning)
        self.primitives_in_config: Dict[int, List[Tuple[int, int, int, int, int]]] = {}
        
        self._parse()

    def _parse(self):
        if not os.path.exists(self.filepath):
            raise FileNotFoundError(f"Mesh info file not found: {self.filepath}")
            
        with open(self.filepath, 'r') as f:
            lines = [l.strip() for l in f.readlines()]
            
        idx = 0
        while idx < len(lines):
            line = lines[idx]
            
            # Parse Section 1: Start Configurations
            if "control-set-start with theta" in line:
                p = line.split()
                theta, tid = int(p[3]), int(p[6])
                self.initial_config_by_theta[theta] = tid
                self.theta_by_initial_config[tid] = theta
                
            # Parse Section 3: Transition Table
            elif "start type is:" in line:
                tid = int(line.split(": ")[1])
                self.successors[tid] = []
                # Ensure reverse mapping exists (default -1 for non-initial)
                if tid not in self.theta_by_initial_config:
                    self.theta_by_initial_config[tid] = -1
                
                idx += 1
                while idx < len(lines) and "---" not in lines[idx]:
                    # Format: di dj neighbor_config_id primitive_id
                    p = lines[idx].split()
                    self.successors[tid].append((int(p[0]), int(p[1]), int(p[2]), int(p[3])))
                    idx += 1
            
            # Parse Section 4: Heuristic Data (Finals)
            elif "final_thetas_of_type" in line:
                tid = int(line.split()[1])
                self.primitives_in_config[tid] = []
                idx += 1
                while idx < len(lines) and "---" not in lines[idx]:
                    # Format: final_theta di dj step_k primitive_id
                    p = lines[idx].split()
                    self.primitives_in_config[tid].append((
                        int(p[0]), int(p[1]), int(p[2]), int(p[3]), int(p[4])
                    ))
                    idx += 1
            idx += 1

class VisualizableMesh:
    """
    Python implementation of the MeshA* algorithm for visualization.
    
    Key Features:
    - Search Space: Extended Cells (i, j, ConfigurationID).
    - Heuristic: Admissible heuristic based on lookahead over 'Finals'.
    - Pruning: Detects dead-ends by checking reachability of Initial Cells.
    """
    
    def __init__(self, task_map: Map, mesh_info: MeshInfo, control_set: ControlSet, 
                 theta_config: Theta, R: float = 0.0, A: int = 0):
        self.map = task_map
        self.mesh_info = mesh_info
        self.cs = control_set
        self.theta_struct = theta_config
        self.R = R
        self.A = A

    def heuristic(self, i: int, j: int, config_id: int, goal: DiscreteState) -> float:
        """
        Calculates the admissible heuristic h(u).
        
        Case 1 (Initial Cell): Euclidean distance to goal.
        Case 2 (Intermediate Cell): min( h(endpoint) + cost_to_endpoint ) over all valid primitives.
        """
        # Case 1: Initial Extended Cell
        if self.mesh_info.theta_by_initial_config.get(config_id, -1) >= 0:
            return np.hypot(i - goal.i, j - goal.j)
        
        # Case 2: Intermediate Cell
        min_h = 1e9
        finals = self.mesh_info.primitives_in_config.get(config_id, [])
        
        for (ft, di, dj, k, prim_id) in finals:
            next_i = i + di
            next_j = j + dj
            
            cost_to_endpoint = self.cs.list_all_prims[prim_id].length
            
            # Speculative check (ignore closed list for heuristic speed)
            if self.map.in_bounds(next_i, next_j) and self.map.traversable(next_i, next_j):
                endpoint_h = np.hypot(next_i - goal.i, next_j - goal.j)
                if cost_to_endpoint + endpoint_h < min_h:
                    min_h = cost_to_endpoint + endpoint_h
                    
        return min_h

    def should_prune(self, i: int, j: int, config_id: int, closed_set: Set) -> bool:
        """
        Checks if the current Extended Cell leads only to blocked or closed states.
        If so, it can be safely pruned (Definition 5 in the paper).
        """
        # Never prune Initial Cells here (handled by regular expansion)
        if self.mesh_info.theta_by_initial_config.get(config_id, -1) >= 0:
            return False
            
        finals = self.mesh_info.primitives_in_config.get(config_id, [])
        for (ft, di, dj, k, prim_id) in finals:
            next_i = i + di
            next_j = j + dj
            next_config = self.mesh_info.initial_config_by_theta.get(ft, -1)
            
            if self.map.in_bounds(next_i, next_j) and self.map.traversable(next_i, next_j):
                # If at least one endpoint is NOT in CLOSED, this cell is viable
                if (next_config, next_i, next_j) not in closed_set:
                    return False 
        
        return True # All paths blocked or closed

    def run(self, start_node: DiscreteState, goal_node: DiscreteState, 
            visualizer=None, update_freq: int = 10) -> bool:
        """
        Executes the MeshA* search.
        """
        if start_node.theta not in self.mesh_info.initial_config_by_theta:
            print(f"Error: No Mesh configuration found for start theta {start_node.theta}")
            return False
            
        start_type = self.mesh_info.initial_config_by_theta[start_node.theta]
        h0 = self.heuristic(start_node.i, start_node.j, start_type, goal_node)
        
        # Priority Queue: (f, g, type_id, i, j)
        open_set = []
        heapq.heappush(open_set, (h0, 0.0, start_type, start_node.i, start_node.j))
        
        closed_set = set()
        g_scores = {(start_type, start_node.i, start_node.j): 0.0}
        came_from = {}
        
        # Visualization Containers
        vis_open = set([(start_node.i, start_node.j)])
        vis_closed = set()
        vis_initials = set() # NEW: Tracks Initial Extended Cells (Magenta)
        
        vis_segments = []       
        vis_invalid_segments = [] 
        
        expansion_count = 0
        
        while open_set:
            f, g, u_type, u_i, u_j = heapq.heappop(open_set)
            state_key = (u_type, u_i, u_j)
            
            # Sync Visual Sets
            if (u_i, u_j) in vis_open: vis_open.remove((u_i, u_j))
            vis_closed.add((u_i, u_j))
            
            if state_key in closed_set: continue

            # === INITIAL CELL HANDLING (Pivot States) ===
            theta_curr = self.mesh_info.theta_by_initial_config.get(u_type, -1)
            
            if theta_curr >= 0:
                # Mark as Initial Cell for Visualization (Magenta Layer)
                vis_initials.add((u_i, u_j))
                
                # Goal Check (Only valid at Initial Cells per Theorem 1)
                dist_sq = (u_i - goal_node.i)**2 + (u_j - goal_node.j)**2
                a_dist = self.theta_struct.num_dist(theta_curr, goal_node.theta)
                
                if (dist_sq <= self.R**2) and (a_dist <= self.A):
                    if visualizer:
                        # Final update including Initial Cells
                        visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid_segments, vis_initials)
                        path = self._reconstruct_path(state_key, came_from)
                        visualizer.animate_final_path(path)
                    return True
                
                # Visualize Bundles (Outgoing Primitives)
                # This demonstrates how MeshA* covers the local space
                for prim in self.cs.get_prims_heading(theta_curr):
                    xs = prim.x_coords + u_j
                    ys = prim.y_coords + u_i
                    
                    # Check validity just for coloring (blue vs dashed grey)
                    ci = prim.collision_in_i + u_i
                    cj = prim.collision_in_j + u_j
                    is_valid = True
                    for k in range(len(ci)):
                        if not self.map.in_bounds(ci[k], cj[k]) or not self.map.traversable(ci[k], cj[k]):
                            is_valid = False
                            break
                    
                    if is_valid:
                        vis_segments.append((xs, ys))
                    else:
                        vis_invalid_segments.append((xs, ys))

            closed_set.add(state_key)
            expansion_count += 1
            
            # === PRUNING ===
            if self.should_prune(u_i, u_j, u_type, closed_set):
                continue

            # === EXPANSION (Transitions) ===
            if u_type in self.mesh_info.successors:
                for (di, dj, v_type, prim_id) in self.mesh_info.successors[u_type]:
                    v_i, v_j = u_i + di, u_j + dj
                    
                    # Basic bounds check
                    if not self.map.in_bounds(v_i, v_j) or not self.map.traversable(v_i, v_j):
                        continue
                    
                    # Edge Cost: 0 for Regular successors, L(pi) for Initial successors
                    step_cost = 0.0
                    if prim_id != -1:
                        step_cost = self.cs.list_all_prims[prim_id].length
                    
                    v_g = g + step_cost
                    v_key = (v_type, v_i, v_j)
                    
                    if v_key not in g_scores or v_g < g_scores[v_key]:
                        g_scores[v_key] = v_g
                        v_h = self.heuristic(v_i, v_j, v_type, goal_node)
                        heapq.heappush(open_set, (v_g + v_h, v_g, v_type, v_i, v_j))
                        came_from[v_key] = state_key
                        
                        vis_open.add((v_i, v_j))
                        if (v_i, v_j) in vis_closed: vis_closed.remove((v_i, v_j))

            # === LIVE UPDATE ===
            if visualizer and (expansion_count % update_freq == 0):
                visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid_segments, vis_initials)
                vis_segments = []
                vis_invalid_segments = []

        return False

    def _reconstruct_path(self, current_key, came_from):
        """
        Reconstructs the trajectory by backtracking through Initial Extended Cells.
        Algorithm 3 from the paper.
        """
        # 1. Backtrack state sequence
        full_trace = []
        curr = current_key
        while curr in came_from:
            full_trace.append(curr)
            curr = came_from[curr]
        full_trace.append(curr)
        full_trace.reverse()
        
        # 2. Extract Initial Cells
        initials = []
        for state in full_trace:
            tid, _, _ = state
            if self.mesh_info.theta_by_initial_config.get(tid, -1) >= 0:
                initials.append(state)
                
        # 3. Retrieve Geometry via Control Set
        path_geometry = []
        for t in range(len(initials) - 1):
            u = initials[t]
            v = initials[t+1]
            
            st = self.mesh_info.theta_by_initial_config[u[0]]
            ft = self.mesh_info.theta_by_initial_config[v[0]]
            di = v[1] - u[1]
            dj = v[2] - u[2]
            
            # Lookup the unique primitive connecting these lattice states
            prim = self.cs.get_prim_between_states(
                DiscreteState(0, 0, st), 
                DiscreteState(di, dj, ft)
            )
            
            # Translate geometry to absolute coordinates
            path_geometry.append((prim.x_coords + u[2], prim.y_coords + u[1]))
                
        return path_geometry
