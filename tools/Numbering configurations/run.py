"""
This standalone script generates the Mesh Graph structure from a given Control Set.
It implements the pre-computation stage of the MeshA* algorithm.

================================================================================
TERMINOLOGY MAPPING (CODE vs FILE):
- In this code, we use the strict term "Configuration ID" (or just ID) to refer 
  to the unique integer assigned to a specific configuration of primitives.
- In the output file format, purely for legacy compatibility reasons, this ID 
  is referred to as a "Type". 
  THEREFORE: "Configuration ID" in code == "Type" in file.
================================================================================

IMPORTANT NOTE ON INDEXING:
In the associated research paper, the step index 'k' in a primitive trace is 
often denoted as 1-based (1, ..., U). However, in this implementation and the 
output files, 'k' is strictly 0-based (0, ..., U-1). 
- k=0: The start of the primitive.
- k=U-1: The final cell of the primitive.

================================================================================
OUTPUT FILE FORMAT SPECIFICATION:
The generated text file describes the topology of the Mesh Graph using 4 sections.

1. START CONFIGURATIONS (Legacy: "START TYPES")
   Mapping: Discrete Heading (theta) -> Initial Configuration ID.
   This section defines which Configuration ID corresponds to an "Initial Extended Cell"
   where a bundle of primitives starts at angle theta.
   - Count: Exactly equal to the number of discrete angles (e.g., 16).
   - Format: "control-set-start with theta: <theta_idx> has type: <ConfID>"

2. GOAL CONFIGURATIONS (Legacy: "GOAL TYPES")
   Mapping: Configuration ID -> Discrete Heading.
   This is effectively the INVERSE of the Start section. If a primitive finishes
   in a cell with Configuration ID 'X', it creates a condition equivalent to starting
   a new bundle at that angle.
   - Count: Exactly equal to the number of discrete angles (e.g., 16), as every 
     angle corresponds to exactly one Initial Configuration.
   - Format: "in goal type: <ConfID> ends (or equal starts) prims-in-theta: <theta>"

3. TRANSITION TABLE (i.e. successors)
   Describes the edges of the Mesh Graph. 
   Format:
     "start type is: <ParentConfID>"
     "<di> <dj> <NeighborConfID> <PrimitiveID>"
     ...
     "---"
   
   Column details:
   - di, dj: Spatial displacement to the neighbor.
   - NeighborConfID: The ID of the resulting configuration in the neighbor extended cell.
   - PrimitiveID: 
       * If neighbor is an INITIAL successor (a primitive ended here): This is the ID 
         of the specific primitive that finished. According to "Lemma 1 (Uniqueness of Path Cost)"
         in the paper, there is only one such primitive that yields correct cost.
       * If neighbor is a REGULAR successor (primitives continue): This value is -1.

4. HEURISTIC DATA (the set \texttt{Finals} from paper, section "On The Efficiency Of MeshA*")
   Used to calculate the heuristic (cost-to-go). For a given Configuration ID, 
   lists information about all primitives passing through it.
   Format:
     "final_thetas_of_type: <ConfID> is (ft, di, dj, k, id):"
     "<ft> <di> <dj> <k> <id>"
     ...
     "---"
   Where:
     - ft: Final theta (heading at the end of the primitive).
     - di, dj: Remaining displacement to reach the end of the primitive.
     - k: The CURRENT step index of the primitive (0-based).
     - id: The primitive ID.
"""

if __name__ != "__main__": 
    raise ImportError("This module is a standalone script and should not be imported!")

from typing import List, Tuple, Dict, Generator, Set, Optional
import sys
import math
import copy

# Allow importing common modules from parallel directory common
sys.path.append("../common/")
sys.path.append("tools/common/") 
from KC_structs import *

# ==============================================================================
# Helper Functions
# ==============================================================================

def step_shift(prim: Primitive, k: int) -> Tuple[int, int]:
    """
    Calculates the displacement (Delta) between the k-th and (k+1)-th cell 
    in the collision trace of the primitive.
    
    Formal notation: Delta^{prim}_k = (i' - i, j' - j)
    """
    assert 0 <= k < prim.U - 1, "Cannot calculate shift from the last cell of the trace."
    
    curr_ij = (prim.collision_in_i[k], prim.collision_in_j[k])
    next_ij = (prim.collision_in_i[k+1], prim.collision_in_j[k+1])
    
    di = next_ij[0] - curr_ij[0]
    dj = next_ij[1] - curr_ij[1]
    
    # Verify logical consistency (displacement must be to a neighbor)
    valid_shifts = [(-1, -1), (-1, 0), (-1, 1), 
                    (0, -1),           (0, 1), 
                    (1, -1),  (1, 0),  (1, 1)]
    assert (di, dj) in valid_shifts, f"Invalid step shift: {(di, dj)}"
    
    return di, dj


# ==============================================================================
# Core Classes
# ==============================================================================

class PrimsConfiguration:
    """
    Represents a Configuration of Primitives (Psi).
    
    Psi = { (prim_1, k), ..., (prim_n, k) }
    
    It captures the set of motion primitives passing through a specific grid cell
    and their current progress (step index k) within that cell.
    """
    
    def __init__(self, control_set: ControlSet) -> None:
        self.control_set = control_set
        # We use a set because the order of primitives in a configuration implies no difference.
        self.config_list: Set[Tuple[Primitive, int]] = set()

    def add_element(self, pair: Tuple[Primitive, int]) -> 'PrimsConfiguration':
        """Adds a (primitive, step) pair to the configuration."""
        prim, step = pair
        # Ensure k is within valid bounds [0, U-1]
        assert 0 <= step < prim.U, "Invalid primitive step index."
        self.config_list.add(pair)
        return self
    
    def __eq__(self, other: 'PrimsConfiguration') -> bool:
        """
        Two configurations are equal if they contain the exact same set of (primitive, step) pairs.
        This is crucial for mapping configurations to unique IDs.
        """
        return self.config_list == other.config_list
    
    def __hash__(self) -> int:
        """Hashes the frozen set of pairs to allow using Configuration as a dictionary key."""
        return hash(frozenset(self.config_list))
    
    def __iter__(self) -> Generator[Tuple[Primitive, int], None, None]: 
        """Allows iteration over the pairs in the configuration."""
        for each in self.config_list:
            yield each


def InitConf(l: int, control_set: ControlSet) -> PrimsConfiguration:
    """
    Builds the Initial Configuration (Psi_theta) for a given discrete heading index l.
    
    Implements Algorithm 1 (Building the Initial Configuration).
    Psi_theta consists of all primitives from the Control Set that start at angle l,
    initialized at step k=0.
    """
    conf = PrimsConfiguration(control_set)
    for prim in control_set.get_prims_heading(l):
        conf.add_element((prim, 0))
    return conf


class ExtendedCell:
    """
    Represents an Extended Cell u = (i, j, Psi).
    
    This is the atomic element of the search space in MeshA*.
    It couples a specific grid location (i, j) with the dynamic state of 
    motion primitives passing through it (Psi).
    """
    
    def __init__(self, i: int, j: int, conf: PrimsConfiguration) -> None:
        self.i = i
        self.j = j
        self.conf = conf


def GetSuccessors(u: ExtendedCell) -> List[Tuple[ExtendedCell, float, int]]:
    """
    Generates the set of successors for an extended cell u = (i, j, Psi).
    
    Implements Algorithm 2 (Generating Successors of an Extended Cell).
    It distinguishes between two types of successors:
    1. Initial Successor: Result of a primitive finishing.
    2. Regular Successor: Result of primitives continuing to the next cell.
    
    Returns:
        A list of tuples: (SuccessorCell, TransitionCost, PrimitiveID)
        - PrimitiveID is useful for reconstructing the path later (which primitive caused the transition).
    """
    i, j, conf = u.i, u.j, u.conf
    control_set = conf.control_set
    
    # Dictionary to group continuing primitives by their physical displacement.
    # Key: displacement (di, dj), Value: Configuration forming in that direction.
    confs_regular: Dict[Tuple[int, int], PrimsConfiguration] = dict()
    
    successors = [] # List of (ExtendedCell, cost, prim_id)
    
    for (prim, k) in conf:
        # Calculate displacement to the next cell in the trace
        di, dj = step_shift(prim, k)
        
        # Case 1: Initial Successor (Definition 2)
        # The primitive ends at this step (k is the second to last index, so next is last).
        if k == prim.U - 2:
            theta_end = prim.goal.theta # Angle at which primitive ends
            
            # The successor configuration is the Initial Configuration for that angle
            psi_init = InitConf(theta_end, control_set)
            
            v_init = ExtendedCell(i + di, j + dj, psi_init)
            cost = prim.length # Cost is the full length of the primitive
            
            # Store successor. We record prim.id to know WHICH primitive completed here.
            successors.append((v_init, cost, prim.id))
            
        # Case 2: Regular Successor (Definition 3)
        # The primitive continues to the next cell.
        else:
            if (di, dj) not in confs_regular:
                confs_regular[(di, dj)] = PrimsConfiguration(control_set)
            
            # Add the primitive to the grouping, incrementing its step
            confs_regular[(di, dj)].add_element((prim, k + 1))
            
    # Process Regular Successors
    # Convert grouped configurations into Extended Cells
    for (di, dj), psi_reg in confs_regular.items():
        v_reg = ExtendedCell(i + di, j + dj, psi_reg)
        cost = 0.0 # Transition cost between intermediate cells is 0 (Definition 4)
        
        # prim_id is -1 because this transition isn't caused by a single specific choice,
        # but is a deterministic continuation of the bundle.
        successors.append((v_reg, cost, -1))
        
    return successors


# ==============================================================================
# Main Numbering Logic
# ==============================================================================

class MeshGraphBuilder:
    """
    The engine that builds the Mesh Graph.
    
    It performs the pre-computation logic described in the paper:
    1. Enumerates all reachable Configurations of Primitives.
    2. Assigns a unique Configuration ID to each.
    3. Builds the Transition Table (edges between IDs).
    """
    
    def __init__(self, control_set: ControlSet) -> None:
        self.control_set = control_set
        self.theta_amount = control_set.theta_amount
        
        # Maps a unique Configuration object to a unique Integer ID.
        # Corresponds to the 'Numbers' map in Algorithm 4.
        self.ID_by_Conf: Dict[PrimsConfiguration, int] = dict()
        
        # Maps the unique Integer ID back to the Configuration object.
        self.Conf_by_ID: Dict[int, PrimsConfiguration] = dict()
        
        # Current counter for assigning IDs (The next available ID).
        self.next_id: int = 0
        
        # Transition Table: Configuration ID -> List of successors.
        # This IS the precomputed graph topology.
        # Value format: [(NeighborID, delta_i, delta_j, causing_prim_id), ...]
        self.TransTable: Dict[int, List[Tuple[int, int, int, int]]] = dict()


    def NumberingDFS(self, u: ExtendedCell) -> None:
        """
        Depth-First Search to traverse and number reachable configurations.
        Implements Algorithm 4 (Numbering Configurations of Primitives).
        """
        conf = u.conf
        
        # If this configuration has already been visited/numbered, return.
        if conf in self.ID_by_Conf:
            return
        
        # Assign a new Configuration ID
        curr_id = self.next_id
        self.next_id += 1
        
        self.ID_by_Conf[conf] = curr_id
        self.Conf_by_ID[curr_id] = conf
        self.TransTable[curr_id] = []
        
        # Generate successors using Algorithm 2 logic
        # succ_list contains tuples: (SuccessorCell, cost, prim_id)
        # Note: prim_id is -1 for Regular Successors, and valid ID for Initial Successors.
        succ_list = GetSuccessors(u)
        
        for v, _, prim_id in succ_list: 
            self.NumberingDFS(v)
            
            # Record the transition in the table
            target_id = self.ID_by_Conf[v.conf]
            di = v.i - u.i
            dj = v.j - u.j
            
            self.TransTable[curr_id].append((target_id, di, dj, prim_id))


    def MainProcedure(self) -> None:
        """
        Main entry point (corresponds to 'MainProcedure' in the paper).
        
        Iterates through all possible Initial Configurations (one per heading)
        and triggers DFS to discover the entire reachable component of the 
        configuration space.
        """
        print("Starting Mesh Graph construction (MainProcedure)...")
        
        for theta in range(self.theta_amount):
            # Create the Initial Configuration for heading 'theta'
            # This corresponds to Psi_theta in the paper.
            conf_0 = InitConf(theta, self.control_set)
            
            # Create the Initial Extended Cell (coordinates 0,0 are arbitrary placeholders)
            u_start = ExtendedCell(0, 0, conf_0) 
            
            self.NumberingDFS(u_start)
            
        print(f"Construction finished! Total unique Configuration IDs: {self.next_id}")


    def start_theta(self, conf_id: int) -> Optional[int]:
        """
        Checks if a Configuration ID corresponds to an Initial Configuration Psi_theta.
        Returns theta if yes, None otherwise.
        """
        conf = self.Conf_by_ID[conf_id]
        
        # An Initial Configuration must contain primitives of a single heading, all at step 0.
        starts = set()
        for (prim, k) in conf:
            if k != 0:
                return None 
            starts.add(prim.start.theta % self.theta_amount)
            
        if len(starts) == 1:
            return list(starts)[0]
        return None


    def goal_theta(self, conf_id: int) -> Optional[List[int]]:
        """
        Identifies if this Configuration ID acts as a Goal (Initial) state for primitives.
        Returns list of headings.
        """
        # Since 'Initial Configuration' and 'Goal Configuration' are symmetric concepts
        # (ending a primitive leads to an Initial Config), we reuse logic.
        theta = self.start_theta(conf_id)
        if theta is not None:
            return [theta]
        return None

    
    def get_final_thetas(self, conf_id: int) -> List[Tuple[int, int, int, int, int]]:
        """
        Prepares data for heuristic calculation.
        Returns list of: (final_theta, remaining_di, remaining_dj, current_k, prim_id)
        """
        result = set()
        conf = self.Conf_by_ID[conf_id]
        
        for (prim, k) in conf:
            # SPECIAL CASE: Initial Configuration (Psi_theta)
            # Represents the start of a bundle.
            if k == 0:
                result.add((prim.start.theta, 0, 0, 0, -1))
            else:
                # REGULAR CASE: In-progress primitive
                di, dj = 0, 0
                for l in range(k, prim.U - 1):
                    a, b = step_shift(prim, l)
                    di += a
                    dj += b
                
                result.add((prim.goal.theta, di, dj, k, prim.id))
            
        return list(result)


    def save_graph_to_file(self, file_path: str) -> None:
        """
        Exports the generated Mesh Graph to the text file format.
        """
        
        with open(file_path, "w") as f:
            
            # 1. START CONFIGURATIONS (Legacy: Start Types)
            # There must be exactly one entry for each discrete angle.
            cnt = 0
            for k in range(self.next_id):
                theta = self.start_theta(k)
                if theta is not None:
                    f.write(f"control-set-start with theta: {theta} has type: {k}\n")
                    cnt += 1
            assert cnt == self.theta_amount, \
                f"Error: Expected {self.theta_amount} start configurations, found {cnt}."
            
            # 2. GOAL CONFIGURATIONS (Legacy: Goal Types)
            # This is the inverse mapping of the above.
            cnt_goals = 0
            for k in range(self.next_id):
                thetas = self.goal_theta(k)
                if thetas is not None:
                    str_thetas = " ".join(map(str, thetas))
                    f.write(f"in goal type: {k} ends (or equal starts) prims-in-theta: {str_thetas}\n")
                    cnt_goals += 1
            assert cnt_goals == self.theta_amount, \
                 f"Error: Expected {self.theta_amount} goal configurations, found {cnt_goals}."

            # 3. TRANSITION TABLE (Successors)
            for k in range(self.next_id):
                f.write(f"start type is: {k}\n")
                
                for neighbor_id, di, dj, prim_id in self.TransTable[k]:
                    # prim_id is only relevant if a primitive finishes here (Initial Successor).
                    # Otherwise it is -1.
                    f.write(f"{di} {dj} {neighbor_id} {prim_id}\n")
                f.write("---\n")
            
            # 4. HEURISTIC DATA (Legacy: Final Thetas)
            for k in range(self.next_id):
                f.write(f"final_thetas_of_type: {k} is (ft, di, dj, k, id):\n")
                
                for ft, di, dj, step_k, pid in self.get_final_thetas(k):
                    f.write(f"{ft} {di} {dj} {step_k} {pid}\n")
                f.write("---\n")


# ==============================================================================
# Execution Block
# ==============================================================================

print("Start execution...")

# List of control sets to process
files_control_set = ["base_control_set.txt", "add_control_set.txt"]

# Corresponding output files for the Mesh topology
files_for_mesh = ["base_mesh_info.txt", "add_mesh_info.txt"]

for prim_file_name, mesh_file_name in zip(files_control_set, files_for_mesh):
    
    # Construct paths (adjust logic as needed for your folder structure)
    # Here assuming input primitives are in the same folder or specific data folder
    # For now, using direct names as in your snippet.
    
    print(f"Processing {prim_file_name} -> {mesh_file_name}...")
    
    # 1. Setup Discretization
    theta_16 = Theta() 
    
    # 2. Load Control Set
    # Note: Ensure Primitive.load_primitives logic handles the file path correctly
    control_set = ControlSet(theta_16).load_primitives(prim_file_name) 
    print(f"  Control set loaded. Primitives: {sum(len(l) for l in control_set.control_set)}")

    # 3. Run Numbering Algorithm
    algorithm = MeshGraphBuilder(control_set)
    algorithm.MainProcedure()
    
    # 4. Save Result
    algorithm.save_graph_to_file(mesh_file_name) 
    
    print(f"  Saved {algorithm.next_id} IDs to {mesh_file_name}.")
    print("-" * 30)

print("All tasks completed successfully!")
