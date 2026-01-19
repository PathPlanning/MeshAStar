"""
This module defines the core data structures and functions required for A* search 
on graph structures (e.g., State Lattice or Mesh graphs).
"""

import numpy as np
from heapq import heappop, heappush
from typing import List, Callable, Union, Optional, Tuple, Set

try:
    from typing import Self
except ImportError:
    from typing_extensions import Self 

from KC_structs import DiscreteState

# Generic type for a graph vertex
VertexType = Union[DiscreteState]


class Map:  
    """
    Represents the discrete workspace (occupancy grid).
    Stored as a binary matrix where '1' indicates an obstacle and '0' indicates free space.
    
    Coordinate System Note:
    The map is stored in matrix format (row i, column j), where 'i' increases downwards 
    and 'j' increases to the right. 
    
    When using motion primitives generated in a Cartesian frame (where Y usually points up),
    a coordinate transformation may be required. Typically, if the primitives assume Y-up, 
    but the map 'i' is down, the heading angles on the map will appear mirrored (clockwise vs counter-clockwise).
    This can be resolved by negating the starting heading (-theta) during the search initialization.
    """
    
    def __init__(self) -> None:
        self.cells = None 
        self.width = 0
        self.height = 0
             
    
    def convert_string_to_cells(self, cell_str: str, obs: bool = True) -> Self:
        """
        Parses a string representation of the map into a binary matrix.
        
        Args:
            cell_str: String containing the map ('.' for free, '#' or '@' for obstacles).
            obs: If False, ignores obstacles and treats the entire map as traversable.
        """
        
        # Normalize line endings
        cell_str = cell_str.replace("\r\n", "\n")
        cell_lines = cell_str.split("\n")
        
        cells_list = []

        for line in cell_lines:
            if len(line) == 0:
                continue
            # Ignore lines that are likely metadata (do not start with map characters)
            if line[0] not in ['.', '@', '#', 'T']:
                continue
            
            cells_row = []
            for char in line:
                if char == '.':
                    cells_row.append(0)
                elif (char == '#') or (char == '@') or (char == 'T'):
                    cells_row.append(1 if obs else 0)
                elif (char == ' '):
                    continue
                else:
                    raise Exception(f"Unknown character '{char}' in map string.")
                
            cells_list.append(cells_row)
            
        self.cells = np.array(cells_list, dtype=np.int8)
        self.width = self.cells.shape[1]
        self.height = self.cells.shape[0]
        
        return self


    def in_bounds(self, i: int, j: int) -> bool:
        """Checks if coordinates (i, j) are within map boundaries."""
        return (0 <= i < self.height) and (0 <= j < self.width)


    def traversable(self, i: int, j: int) -> bool:
        """Checks if cell (i, j) is free (not an obstacle)."""
        return (self.cells[i][j] == 0)            
    
    
    def get_slice(self, si: int, fi: int, sj: int, fj: int) -> Self:
        """
        Crops the map to the specified bounds [si, fi) and [sj, fj).
        Useful for performing search on a sub-region.
        """
        self.cells = self.cells[si:fi, sj:fj]
        self.width = self.cells.shape[1]
        self.height = self.cells.shape[0]
        return self


class SearchNode:
    """
    Wrapper class for a graph vertex in the A* algorithm.
    Contains the vertex data along with search metadata (g, h, f values, parent pointer).
    """
    
    def __init__(self, vertex: VertexType,
                 g: float = 0.0, h: float = 0.0, f: Optional[float] = None,
                 parent: Optional['SearchNode'] = None) -> None:
        """
        Args:
            vertex: The graph node (DiscreteState or TypeMesh).
            g: Cost from start to this node.
            h: Heuristic estimate from this node to goal.
            f: Total estimated cost (f = g + h).
            parent: The predecessor node in the optimal path found so far.
        """
        self.vertex = vertex
        self.g = g
        self.h = h
        self.f = f if f is not None else (g + h)   
        self.parent = parent


    def __lt__(self, other: 'SearchNode') -> bool:
        """
        Comparison operator for the priority queue.
        Nodes with lower f-values have higher priority.
        """
        return self.f < other.f
    
    def __eq__(self, other: 'SearchNode') -> bool:
        """
        Equality check is delegated to the underlying vertex.
        Two SearchNodes are considered 'equal' if they represent the same state/vertex.
        """
        return self.vertex == other.vertex
    
    def __hash__(self) -> int:
        """
        Hash is delegated to the underlying vertex to allow storage in sets/dicts.
        """
        return hash(self.vertex)
    
      
class SearchTreePQD:
    """
    Implements the Open and Closed sets for A* search.
    
    Structure:
    - OPEN: A min-heap priority queue storing SearchNodes (sorted by f-value).
      Supports lazy deletion of duplicates.
    - CLOSED: A hash set storing expanded vertices (wrapped in SearchNodes for metadata retention).
    """
    
    def __init__(self) -> None:
        self._open: List[SearchNode] = []   # Heap
        self._closed: Set[SearchNode] = set() # Set
        self._enc_open_duplicates = 0       # Debug counter
         
                                      
    def __len__(self) -> int:
        return len(self._open) + len(self._closed)
       
                
    def open_is_empty(self) -> bool:
        return len(self._open) == 0
    
    
    def add_to_open(self, item: SearchNode) -> None:
        """
        Adds a node to the OPEN set.
        Does not check for duplicates immediately (lazy approach).
        """
        heappush(self._open, item)
        
    
    def add_to_closed(self, item: SearchNode) -> None:
        """
        Adds a node to the CLOSED set, marking it as expanded.
        """
        self._closed.add(item)


    def was_expanded(self, item: SearchNode) -> bool:
        """
        Checks if the vertex contained in 'item' has already been expanded.
        Relies on __hash__ and __eq__ of the underlying vertex.
        """
        return item in self._closed


    def get_best_node_from_open(self) -> Optional[SearchNode]:
        """
        Extracts the node with the lowest f-value from OPEN.
        Handles lazy removal of nodes that have already been expanded (duplicates).
        """
        while True:
            if self.open_is_empty():
                return None
            
            best_node = heappop(self._open)
            
            # If strictly already in closed, it's a suboptimal duplicate -> skip
            if not self.was_expanded(best_node):
                return best_node            
            
            self._enc_open_duplicates += 1
        

    @property
    def opened(self) -> List[SearchNode]:
        """Returns the raw list representing the priority queue."""
        return self._open
    
    @property
    def expanded(self) -> List[SearchNode]:
        """Returns the list of expanded nodes (CLOSED set)."""
        return list(self._closed)

    @property
    def number_of_open_duplicates(self) -> int:
        return self._enc_open_duplicates
    
    
      
def astar(start: VertexType, 
          is_goal: Callable[[VertexType], bool], 
          get_successors: Callable[[VertexType], List[VertexType]],
          heuristic_func: Callable[[VertexType], float], 
          compute_cost: Callable[[VertexType, VertexType], float], 
          w: float = 1.0) -> Tuple[bool, Optional[SearchNode], int, Optional[List[SearchNode]], List[SearchNode]]:
    """
    General implementation of the A* (or Weighted A*) algorithm.
    
    Args:
        start: The starting vertex.
        is_goal: Function returning True if a vertex is the goal.
        get_successors: Function returning a list of neighbor vertices.
        heuristic_func: Function estimating cost from vertex to goal.
        compute_cost: Function calculating actual cost between two adjacent vertices.
        w: Heuristic weight (w=1 for standard A*, w>1 for WA*).
        
    Returns:
        Tuple containing:
        - success (bool): True if path found.
        - final_node (SearchNode): The goal node wrapper (trace parent pointers for path).
        - steps (int): Number of expansions.
        - OPEN (list): Final state of the open set.
        - CLOSED (list): Final state of the closed set.
    """
    
    ast = SearchTreePQD()
    steps = 0
    
    start_node = SearchNode(start, h=w*heuristic_func(start))
    ast.add_to_open(start_node)

    while not ast.open_is_empty():
        current = ast.get_best_node_from_open()
        if current is None:
            break
        
        v = current.vertex
        ast.add_to_closed(current)

        # Check goal condition
        if is_goal(v):
            return True, current, steps, ast.opened, ast.expanded

        # Expand neighbors
        for u in get_successors(v):
            new_node = SearchNode(u)
            
            # Only process if not already expanded (in CLOSED)
            if not ast.was_expanded(new_node):
                g_new = current.g + compute_cost(v, u)
                h_new = w * heuristic_func(u)
                
                new_node.g = g_new
                new_node.h = h_new
                new_node.f = g_new + h_new
                new_node.parent = current
                
                ast.add_to_open(new_node)

        steps += 1
    
    # Path not found
    return False, None, steps, None, ast.expanded
