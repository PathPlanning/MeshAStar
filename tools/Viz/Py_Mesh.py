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
    Data structure representing the precomputed Mesh Graph topology.
    
    The Mesh Graph is an abstraction where nodes correspond to 'Extended Cells' 
    (spatial grid coordinates + discrete configuration ID) and edges correspond 
    to valid transitions via motion primitives.
    
    This class handles the parsing of the precomputed lookup table, which defines:
    1. Mapping between discrete headings and Initial Configuration IDs (Pivots).
    2. Adjacency lists (Successors) for graph traversal.
    3. Internal structure of primitive bundles (Pruning/Heuristic lookups).
    """
    def __init__(self, filepath: str, control_set: ControlSet):
        """
        Initializes the Mesh topology from a precomputed file.
        
        Args:
            filepath: Path to the generated mesh structure file (.txt).
            control_set: Reference to the motion primitives used for generation.
        """
        self.filepath = filepath
        self.control_set = control_set
        
        # Mapping: Discrete Theta Index <-> Initial Configuration ID (Pivot States)
        # Used to identify if a configuration corresponds to the start of a new primitive bundle.
        self.initial_config_by_theta: Dict[int, int] = {}
        self.theta_by_initial_config: Dict[int, int] = {}
        
        # Transition Table: ConfigID -> List of (delta_i, delta_j, neighbor_config_id, prim_id)
        # Defines the graph edges: how to move from one extended cell to another.
        # prim_id is -1 for intermediate steps, and a valid ID for completed motions (Initial->Initial).
        self.successors: Dict[int, List[Tuple[int, int, int, int]]] = {}
        
        # Bundle Geometry: ConfigID -> List of (final_theta, di, dj, step_k, prim_id)
        # Stores the set of all primitives ('Finals') that are geometrically valid within a specific configuration.
        # Crucial for computing the admissible heuristic and performing reachability pruning.
        self.primitives_in_config: Dict[int, List[Tuple[int, int, int, int, int]]] = {}
        
        self._parse()

    def _parse(self):
        """Parses the specific text format of the precomputed Mesh file."""
        if not os.path.exists(self.filepath):
            raise FileNotFoundError(f"Mesh info file not found: {self.filepath}")
            
        with open(self.filepath, 'r') as f:
            lines = [l.strip() for l in f.readlines()]
            
        idx = 0
        while idx < len(lines):
            line = lines[idx]
            # Section 1: Pivot State Definitions
            if "control-set-start with theta" in line:
                p = line.split()
                theta, tid = int(p[3]), int(p[6])
                self.initial_config_by_theta[theta] = tid
                self.theta_by_initial_config[tid] = theta
            
            # Section 2: Adjacency / Successors
            elif "start type is:" in line:
                tid = int(line.split(": ")[1])
                self.successors[tid] = []
                if tid not in self.theta_by_initial_config:
                    self.theta_by_initial_config[tid] = -1
                idx += 1
                while idx < len(lines) and "---" not in lines[idx]:
                    p = lines[idx].split()
                    self.successors[tid].append((int(p[0]), int(p[1]), int(p[2]), int(p[3])))
                    idx += 1
            
            # Section 3: Bundle Composition (Heuristic Data)
            elif "final_thetas_of_type" in line:
                tid = int(line.split()[1])
                self.primitives_in_config[tid] = []
                idx += 1
                while idx < len(lines) and "---" not in lines[idx]:
                    p = lines[idx].split()
                    self.primitives_in_config[tid].append((
                        int(p[0]), int(p[1]), int(p[2]), int(p[3]), int(p[4])
                    ))
                    idx += 1
            idx += 1

class VisualizableMesh:
    """
    MeshA* Algorithm Implementation with Real-Time Visualization.
    
    Performs A* search over the implicitly defined Mesh Graph. This planner 
    differs from standard Lattice A* by expanding 'bundles' of trajectories 
    simultaneously and pruning those that are geometrically blocked or dominated.
    
    Visualization Semantics:
    - Magenta Cells: Initial Extended Cells (Pivot points where bundles originate).
    - Orange Cells: OPEN Set (The search frontier).
    - Grey Cells: CLOSED Set (Explored states).
    - Blue Lines (During Search): Valid motion primitives explored so far.
    - Dark Grey Lines (Post-Processing): The 'history' of exploration, dimmed for contrast.
    - Bright Blue Lines (Post-Processing): The Essential Spanning Tree, showing the 
      effective branching factor and actual logical paths considered.
    """
    def __init__(self, task_map: Map, mesh_info: MeshInfo, control_set: ControlSet, 
                 theta_config: Theta, R: float = 0.0, A: int = 0):
        self.map = task_map
        self.mesh_info = mesh_info
        self.cs = control_set
        self.theta_struct = theta_config
        self.R = R # Goal radius tolerance
        self.A = A # Goal angular tolerance

    def heuristic(self, i: int, j: int, config_id: int, goal: DiscreteState) -> float:
        """
        Computes the admissible heuristic h(u) for a given Extended Cell.
        
        Logic:
        1. If u is an Initial Cell (Pivot): h(u) = Euclidean distance to goal.
        2. If u is an Intermediate Cell: h(u) = min( cost_remaining + h(endpoint) )
           over all primitives surviving in the current bundle configuration.
           This effectively performs a 'lookahead' to the end of the primitive.
        """
        # Case 1: Initial Cell
        if self.mesh_info.theta_by_initial_config.get(config_id, -1) >= 0:
            return np.hypot(i - goal.i, j - goal.j)
        
        # Case 2: Intermediate Cell (Lookahead)
        min_h = 1e9
        finals = self.mesh_info.primitives_in_config.get(config_id, [])
        
        for (ft, di, dj, k, prim_id) in finals:
            next_i = i + di
            next_j = j + dj
            cost_to_endpoint = self.cs.list_all_prims[prim_id].length
            
            # Speculative check (ignores Closed list for heuristic speed)
            if self.map.in_bounds(next_i, next_j) and self.map.traversable(next_i, next_j):
                endpoint_h = np.hypot(next_i - goal.i, next_j - goal.j)
                if cost_to_endpoint + endpoint_h < min_h:
                    min_h = cost_to_endpoint + endpoint_h
        return min_h

    def should_prune(self, i: int, j: int, config_id: int, closed_set: Set) -> bool:
        """
        Implements the MeshA* Domination Rule (Pruning).
        
        A state u = (i, j, config) can be pruned if ALL trajectories it carries
        lead to states that are either blocked by obstacles or have already been 
        visited (are in the CLOSED set) via a cheaper or equal path.
        
        Returns:
            True if the state provides no new information and should be discarded.
        """
        # Never prune Initial Cells here; they are handled by standard A* duplicate detection.
        if self.mesh_info.theta_by_initial_config.get(config_id, -1) >= 0:
            return False
            
        finals = self.mesh_info.primitives_in_config.get(config_id, [])
        for (ft, di, dj, k, prim_id) in finals:
            next_i = i + di
            next_j = j + dj
            next_config = self.mesh_info.initial_config_by_theta.get(ft, -1)
            
            if self.map.in_bounds(next_i, next_j) and self.map.traversable(next_i, next_j):
                # If at least one endpoint leads to a valid, unvisited state, we must keep this cell.
                if (next_config, next_i, next_j) not in closed_set:
                    return False 
        
        return True # All downstream paths are blocked or closed

    def run(self, start_node: DiscreteState, goal_node: DiscreteState, 
            visualizer=None, update_freq: int = 10) -> bool:
        """
        Executes the MeshA* search loop.
        
        Args:
            start_node: Start configuration (i, j, theta).
            goal_node: Goal configuration.
            visualizer: Callback object for live rendering.
            update_freq: Visual update rate (every N expansions).
        """
        if start_node.theta not in self.mesh_info.initial_config_by_theta:
            print(f"Error: No Mesh configuration found for start theta {start_node.theta}")
            return False
            
        start_type = self.mesh_info.initial_config_by_theta[start_node.theta]
        h0 = self.heuristic(start_node.i, start_node.j, start_type, goal_node)
        
        # Priority Queue: Stores (f-score, g-score, type, i, j)
        open_set = []
        heapq.heappush(open_set, (h0, 0.0, start_type, start_node.i, start_node.j))
        
        closed_set = set()
        g_scores = {(start_type, start_node.i, start_node.j): 0.0}
        
        # Path reconstruction map: State -> Previous Initial State (Pivot)
        # We track pivots to easily reconstruct valid primitive sequences later.
        came_from = {}
        
        # Visualization Containers
        vis_open = set([(start_node.i, start_node.j)])
        vis_closed = set()
        vis_initials = set() # Special tracking for Pivots
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

            # === INITIAL CELL HANDLING (Pivot Points) ===
            theta_curr = self.mesh_info.theta_by_initial_config.get(u_type, -1)
            
            if theta_curr >= 0:
                # Mark as Initial Cell for Visualization (Magenta Layer)
                vis_initials.add((u_i, u_j))
                
                # Goal Check (Valid only at Initial Cells per Theorem 1 of the paper)
                dist_sq = (u_i - goal_node.i)**2 + (u_j - goal_node.j)**2
                a_dist = self.theta_struct.num_dist(theta_curr, goal_node.theta)
                
                if (dist_sq <= self.R**2) and (a_dist <= self.A):
                    if visualizer:
                        # 1. Finalize Frontier Update
                        visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid_segments, vis_initials)
                        
                        # 2. Reconstruct and Animate Solution Trajectory (Red Line)
                        path = self._reconstruct_path(state_key, came_from)
                        visualizer.animate_final_path(path)

                        # 3. POST-PROCESSING: Isolate Essential Search Tree
                        # Fade out the exploration history (Dark Grey) to highlight the 
                        # actual effective branching factor (Blue).
                        visualizer.dim_history() 
                        
                        tree_segments = self._collect_essential_tree(came_from, open_set, vis_initials)
                        visualizer.draw_essential_tree(tree_segments, duration_frames=20)
                    return True
                
                # Expand Bundles for Visualization (Show Potential Reachability)
                for prim in self.cs.get_prims_heading(theta_curr):
                    xs = prim.x_coords + u_j
                    ys = prim.y_coords + u_i
                    
                    is_valid = True
                    ci = prim.collision_in_i + u_i
                    cj = prim.collision_in_j + u_j
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
            
            # === PRUNING STEP ===
            if self.should_prune(u_i, u_j, u_type, closed_set):
                continue

            # === EXPANSION STEP ===
            if u_type in self.mesh_info.successors:
                for (di, dj, v_type, prim_id) in self.mesh_info.successors[u_type]:
                    v_i, v_j = u_i + di, u_j + dj
                    
                    if not self.map.in_bounds(v_i, v_j) or not self.map.traversable(v_i, v_j):
                        continue
                    
                    # Edge Cost: 0 for Regular successors (internal bundle step), 
                    # Length(pi) for Initial successors (completed primitive).
                    step_cost = 0.0
                    if prim_id != -1:
                        step_cost = self.cs.list_all_prims[prim_id].length
                    
                    v_g = g + step_cost
                    v_key = (v_type, v_i, v_j)
                    
                    if v_key not in g_scores or v_g < g_scores[v_key]:
                        g_scores[v_key] = v_g
                        v_h = self.heuristic(v_i, v_j, v_type, goal_node)
                        heapq.heappush(open_set, (v_g + v_h, v_g, v_type, v_i, v_j))
                        
                        # Ancestry Logic:
                        # If current is Initial, we are the parent.
                        # If current is Intermediate, inherit our parent (the bundle origin).
                        if theta_curr >= 0:
                            came_from[v_key] = state_key
                        else:
                            if state_key in came_from:
                                came_from[v_key] = came_from[state_key]
                        
                        vis_open.add((v_i, v_j))
                        if (v_i, v_j) in vis_closed: vis_closed.remove((v_i, v_j))

            # Live Update Hook
            if visualizer and (expansion_count % update_freq == 0):
                visualizer.update(vis_open, vis_closed, vis_segments, vis_invalid_segments, vis_initials)
                vis_segments = []
                vis_invalid_segments = []

        return False

    def _collect_essential_tree(self, came_from, open_set_heap, visited_initials):
        """
        Reconstructs the 'Essential Spanning Tree' of the search for visualization.
        
        This method filters the dense 'spaghetti' of explored paths, retaining only 
        the primitives that logically connect visited states (Closed & Frontier).
        It validates collision against the map to ensure the tree looks physically correct.
        """
        essential_segments = []
        processed_edges = set()
        
        # Subroutine to retrieve valid geometry for a topological edge
        def get_valid_prim_geometry(parent_i, parent_j, candidate_prims_ids):
            for pid in candidate_prims_ids:
                prim = self.cs.list_all_prims[pid]
                # Check collision relative to Parent frame
                is_valid = True
                ci = prim.collision_in_i + parent_i
                cj = prim.collision_in_j + parent_j
                for k in range(len(ci)):
                    if not self.map.in_bounds(ci[k], cj[k]) or not self.map.traversable(ci[k], cj[k]):
                        is_valid = False
                        break
                if is_valid:
                    return (prim.x_coords + parent_j, prim.y_coords + parent_i, pid)
            return None

        # 1. Process Active Frontier (Open Set Nodes)
        # Draw edges from their parents to the frontier to show pending expansions.
        for (_, _, u_type, u_i, u_j) in open_set_heap:
            state_key = (u_type, u_i, u_j)
            if state_key not in came_from: continue
            
            p_type, p_i, p_j = came_from[state_key]
            theta_u = self.mesh_info.theta_by_initial_config.get(u_type, -1)
            
            candidate_ids = []
            if theta_u >= 0:
                # Direct edge between Initial configurations (Completed Primitive)
                st = self.mesh_info.theta_by_initial_config[p_type]
                ft = theta_u
                di, dj = u_i - p_i, u_j - p_j
                prim = self.cs.get_prim_between_states(DiscreteState(0,0,st), DiscreteState(di,dj,ft))
                if prim: candidate_ids = [prim.id]
            else:
                # Intermediate bundle edge (Incomplete Primitive)
                finals = self.mesh_info.primitives_in_config.get(u_type, [])
                candidate_ids = [f[4] for f in finals]
            
            result = get_valid_prim_geometry(p_i, p_j, candidate_ids)
            if result:
                xs, ys, pid = result
                edge_sig = (p_i, p_j, pid)
                if edge_sig not in processed_edges:
                    essential_segments.append((xs, ys))
                    processed_edges.add(edge_sig)

        # 2. Process Explored Nodes (Visited Initials / Closed Set)
        # Draw the backbone of the search tree.
        for child_key, parent_key in came_from.items():
            u_type, u_i, u_j = child_key
            # Only process completed edges (Initial -> Initial)
            if self.mesh_info.theta_by_initial_config.get(u_type, -1) >= 0:
                p_type, p_i, p_j = parent_key
                st = self.mesh_info.theta_by_initial_config[p_type]
                ft = self.mesh_info.theta_by_initial_config[u_type]
                di, dj = u_i - p_i, u_j - p_j
                prim = self.cs.get_prim_between_states(DiscreteState(0,0,st), DiscreteState(di,dj,ft))
                if prim:
                    result = get_valid_prim_geometry(p_i, p_j, [prim.id])
                    if result:
                        xs, ys, pid = result
                        edge_sig = (p_i, p_j, pid)
                        if edge_sig not in processed_edges:
                            essential_segments.append((xs, ys))
                            processed_edges.add(edge_sig)
                        
        return essential_segments

    def _reconstruct_path(self, current_key, came_from):
        """Reconstructs the full trajectory geometry from start to goal."""
        full_trace = []
        curr = current_key
        full_trace.append(curr)
        while curr in came_from:
            parent = came_from[curr]
            full_trace.append(parent)
            curr = parent
        full_trace.reverse()
        
        path_geometry = []
        for t in range(len(full_trace) - 1):
            u = full_trace[t]
            v = full_trace[t+1]
            st = self.mesh_info.theta_by_initial_config[u[0]]
            ft = self.mesh_info.theta_by_initial_config[v[0]]
            di = v[1] - u[1]
            dj = v[2] - u[2]
            prim = self.cs.get_prim_between_states(
                DiscreteState(0, 0, st), DiscreteState(di, dj, ft)
            )
            path_geometry.append((prim.x_coords + u[2], prim.y_coords + u[1]))
                
        return path_geometry
