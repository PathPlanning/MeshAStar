import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection, PatchCollection
from IPython.display import display, clear_output
from PIL import Image
import numpy as np
from typing import Iterable, List, Tuple, Optional

class LiveVisualizer:
    """
    Real-Time Visualization Engine for Search Algorithms.
    
    This class manages the matplotlib backend to render the state-space exploration
    process frame-by-frame. It supports both vector-based rendering (Quality Mode)
    and raster-based rendering (Fast Mode) for scalability.
    
    It is designed to produce publication-quality visualizations (GIFs) demonstrating
    the difference between standard expansion (LBA*) and bundled expansion (MeshA*).
    
    Visualization Semantics (Color Legend):
    ---------------------------------------
    1. Grid Cells:
       - ORANGE (Alpha 0.5): OPEN Set (The search frontier).
       - GREY   (Alpha 0.25): CLOSED Set (Explored / Expanded states).
       - MAGENTA(Alpha 0.15): Initial Extended Cells (MeshA* Pivot states).
       
    2. Motion Primitives (Edges):
       - BLUE   (Alpha 0.4): Valid motion primitives added to the search tree.
       - GREY (Dashed, Alpha 0.3): Invalid primitives (collisions/out-of-bounds).
       - RED    (Solid, Width 2.5): The optimal path found.
       
    3. Post-Processing (MeshA* Specific):
       - DIM GREY (Alpha 0.0): The full exploration history is faded out.
       - BLUE   (Alpha 0.4): The 'Essential Spanning Tree' is redrawn on top,
         highlighting the logical branching structure vs. the geometric density.
    """
    
    def __init__(self, draw_map_func, task_map, start, goal, theta_config=None,
                 title: str = "Search Process", figsize: Tuple[int, int] = (8, 8), dpi: int = 100,
                 fast_draw: bool = False):
        """
        Initializes the plotting environment.
        
        Args:
            draw_map_func: Callback to render the static environment (obstacles, grid).
            task_map: Map object containing grid dimensions and obstacles.
            start: Start configuration.
            goal: Goal configuration.
            theta_config: Heading discretization info (optional).
            title: Title of the plot.
            figsize: Figure dimensions in inches.
            dpi: Dots per inch resolution.
            fast_draw: If True, uses 'imshow' for grid cells (faster for large maps).
                       If False, uses 'PatchCollection' (sharper vector graphics).
        """
        
        # Disable interactive mode to manage display updates manually via IPython
        plt.ioff()
        
        self.fig, self.ax = plt.subplots(figsize=figsize, dpi=dpi)
        self.frames = []
        self.fast_draw = fast_draw
        self.map_shape = (task_map.height, task_map.width)
        
        # --- Static Background Initialization ---
        # Renders obstacles and start/goal markers once.
        try:
            draw_map_func(task_map, start, goal, ax_existing=self.ax, theta_config=theta_config)
        except TypeError:
            draw_map_func(task_map, start, goal, ax_existing=self.ax)
            
        self.ax.set_title(title, fontsize=14, fontweight='bold')
        
        # --- Dynamic Layer Initialization ---
        # Z-orders determine the stacking order:
        # Background (0) < Invalid Lines (1.5) < Overlay/Grid (2) < Valid Lines (3) < Path (10)
        
        if self.fast_draw:
            # FAST MODE: Uses a single RGBA image overlay for all cell states.
            # Efficient for 100x100+ maps.
            self.overlay_data = np.zeros((task_map.height, task_map.width, 4))
            self.overlay_artist = self.ax.imshow(self.overlay_data, origin='upper', zorder=2)

            self.invalid_lines_artist = LineCollection(
                [], colors='grey', linewidths=0.5, alpha=0.4, linestyles='dashed', zorder=1.5
            )
            self.ax.add_collection(self.invalid_lines_artist)
            
            self.lines_artist = LineCollection(
                [], colors='blue', linewidths=0.8, alpha=0.4, zorder=3
            )
            self.ax.add_collection(self.lines_artist)
            
            self.final_path_artist = LineCollection(
                [], colors='red', linewidths=2.5, alpha=1.0, zorder=10, label='Solution'
            )
            self.ax.add_collection(self.final_path_artist)

            self.initials_artist = self.ax.scatter(
                [], [], marker='*', s=60, c='darkmagenta', alpha=0.15, zorder=4, label='Initial Cells'
            )
            self.ax.add_collection(self.initials_artist)

        else:
            # QUALITY MODE: Uses Vector Patches for crisp rendering.
            # Best for smaller maps or high-res publication figures.
            
            # Layer 1: Closed Set (Explored)
            self.closed_artist = PatchCollection(
                [], facecolors='grey', alpha=0.25, edgecolors='none', zorder=1, label='Closed'
            )
            self.ax.add_collection(self.closed_artist)
            
            # Layer 2: Invalid Primitives (Collisions)
            self.invalid_lines_artist = LineCollection(
                [], colors='grey', linewidths=0.5, alpha=0.4, linestyles='dashed', zorder=1.5
            )
            self.ax.add_collection(self.invalid_lines_artist)

            # Layer 3: Open Set (Frontier)
            self.open_artist = PatchCollection(
                [], facecolors='orange', alpha=0.5, edgecolors='none', zorder=2, label='Open'
            )
            self.ax.add_collection(self.open_artist)
            
            # Layer 4: Valid Primitives (Search Tree)
            self.lines_artist = LineCollection(
                [], colors='blue', linewidths=0.8, alpha=0.4, zorder=3
            )
            self.ax.add_collection(self.lines_artist)
            
            # Layer 5: Initial Cells (MeshA* Pivots)
            # self.initials_artist = PatchCollection(
            #     [], facecolors='magenta', alpha=0.15, edgecolors='none', zorder=4, label='Initial Cells'
            # )
            self.initials_artist = self.ax.scatter(
                [], [], marker='*', s=60, c='darkmagenta', alpha=0.15, zorder=4, label='Initial Cells'
            )  # instead of cells (approach in prev lines) we use star for marking
            self.ax.add_collection(self.initials_artist)
            
            # Layer 6: Optimal Path
            self.final_path_artist = LineCollection(
                [], colors='red', linewidths=2.5, alpha=1.0, zorder=10, label='Solution'
            )
            self.ax.add_collection(self.final_path_artist)

        self.accumulated_valid = []
        self.accumulated_invalid = []
        
        # Capture the initial state (empty map)
        self._capture_frame()
        display(self.fig)

    def update(self, 
               open_cells: Iterable[Tuple[int, int]], 
               closed_cells: Iterable[Tuple[int, int]], 
               valid_segments: List[Tuple[np.ndarray, np.ndarray]], 
               invalid_segments: Optional[List[Tuple[np.ndarray, np.ndarray]]] = None,
               initial_cells: Optional[Iterable[Tuple[int, int]]] = None):
        """
        Updates the visual state of the search algorithm.
        
        Args:
            open_cells: List of (row, col) tuples in the Open Set (Frontier).
            closed_cells: List of (row, col) tuples in the Closed Set.
            valid_segments: List of (xs, ys) arrays for newly added valid primitives.
            invalid_segments: List of (xs, ys) arrays for rejected primitives (collisions).
            initial_cells: List of (row, col) tuples identified as MeshA* Pivots.
        """
        
        # 1. Update Primitive Geometry (Accumulate History)
        for xs, ys in valid_segments:
            self.accumulated_valid.append(np.column_stack([xs, ys]))
        self.lines_artist.set_paths(self.accumulated_valid)
        
        if invalid_segments:
            for xs, ys in invalid_segments:
                self.accumulated_invalid.append(np.column_stack([xs, ys]))
            self.invalid_lines_artist.set_paths(self.accumulated_invalid)
        
        # 2. Update Grid Cell Colors
        if self.fast_draw:
            if closed_cells:
                rows_c, cols_c = zip(*closed_cells)
                self.overlay_data[rows_c, cols_c] = [0.5, 0.5, 0.5, 0.2] # Grey
            if open_cells:
                rows_o, cols_o = zip(*open_cells)
                self.overlay_data[rows_o, cols_o] = [1.0, 0.65, 0.0, 0.5] # Orange
            # if initial_cells:
            #     rows_i, cols_i = zip(*initial_cells)
            #     self.overlay_data[rows_i, cols_i] = [1.0, 0.0, 1.0, 0.15] # Magenta
            self.overlay_artist.set_data(self.overlay_data)
        else:
            if closed_cells:
                self.closed_artist.set_paths([patches.Rectangle((j-0.5, i-0.5), 1, 1) for i, j in closed_cells])
            if open_cells:
                self.open_artist.set_paths([patches.Rectangle((j-0.5, i-0.5), 1, 1) for i, j in open_cells])
            # if initial_cells:
            #     self.initials_artist.set_paths([patches.Rectangle((j-0.5, i-0.5), 1, 1) for i, j in initial_cells])
        if initial_cells:
            # Scatter ждет координаты (x, y), то есть (col, row)
            # initial_cells - это список (row, col)
            rows, cols = zip(*initial_cells)
            # Передаем массив Nx2: [[col1, row1], [col2, row2], ...]
            self.initials_artist.set_offsets(np.column_stack([cols, rows]))

        # Render and capture frame
        self._capture_frame()
        clear_output(wait=True)
        display(self.fig)

    def dim_history(self):
        """
        Prepares the visualizer for the post-processing phase.
        
        This method sets the alpha channel of the entire accumulated search tree 
        to 0.0 (invisible). This effectively "clears" the canvas of the search history
        without losing the data, allowing the 'Essential Spanning Tree' to be drawn 
        cleanly on top in the next step.
        
        Implementation Note:
        We intentionally do NOT capture a frame here. Capturing a frame with alpha=0.0 
        would insert a blank/flashing frame into the GIF. By skipping capture, 
        the animation transitions smoothly from the 'Full Density' view directly 
        to the 'Spanning Tree' view.
        """
        self.lines_artist.set_color('dimgrey')
        self.lines_artist.set_alpha(0.0) 

    def draw_essential_tree(self, tree_segments: List[Tuple[np.ndarray, np.ndarray]], 
                            duration_frames: int = 20):
        """
        Renders the 'Essential Spanning Tree' (Blue) on top of the map.
        
        This visualization highlights the logical structure of the search. 
        While the full search (drawn during execution) shows geometric reachability,
        this tree shows the actual edges connecting the visited states, filtering out
        dominated or redundant trajectories.
        """
        
        segments_stack = [np.column_stack([xs, ys]) for xs, ys in tree_segments]
        
        # Create a new collection for the tree to ensure it renders on top
        # Alpha=0.4 matches the visual style of the original search for consistency.
        tree_collection = LineCollection(
            segments_stack,
            colors='blue', linewidths=0.8, alpha=0.4, zorder=3.6, label='Spanning Tree'
        )
        self.ax.add_collection(tree_collection)
        
        # Force a canvas update
        self.fig.canvas.draw()
        
        # Capture multiple frames to hold this final state in the GIF
        for _ in range(duration_frames):
            self._capture_frame()
            
        # FIX for Jupyter Notebook Artifacts:
        # Calling clear_output() before the final display() prevents the 
        # previous frame from lingering underneath the new one ("Double Image" effect).
        clear_output(wait=True) 
        display(self.fig)

    def animate_final_path(self, path_geometry: List[Tuple[np.ndarray, np.ndarray]], 
                           duration_frames: int = 10):
        """
        Incrementally draws the solution path geometry to create an animation effect.
        
        Args:
            path_geometry: List of (xs, ys) arrays representing the path segments.
            duration_frames: Number of static frames to hold after drawing the path,
                             allowing the viewer to inspect the result before post-processing.
        """
        accumulated_path = []
        for xs, ys in path_geometry:
            accumulated_path.append(np.column_stack([xs, ys]))
            self.final_path_artist.set_paths(accumulated_path)
            self._capture_frame()
            clear_output(wait=True)
            display(self.fig)
            
        # Hold the final path state for a moment
        for _ in range(duration_frames):
            self._capture_frame()

    def _capture_frame(self):
        """
        Captures the current figure canvas into a PIL Image.
        
        This method handles alpha channel masking to ensuring a clean white 
        background, preventing transparency artifacts in the generated GIF.
        """
        self.fig.canvas.draw()
        buf = self.fig.canvas.buffer_rgba()
        arr = np.asarray(buf)
        img = Image.fromarray(arr)
        
        # Composite RGBA against a white background
        bg = Image.new("RGB", img.size, (255, 255, 255))
        bg.paste(img, mask=img.split()[3]) 
        self.frames.append(bg)

    def save_gif(self, filename: str = "search.gif", fps: int = 10, final_pause: float = 1.5):
        """
        Exports the recorded frame sequence to a GIF file.
        
        Args:
            filename: Output filename.
            fps: Frames per second.
            final_pause: Duration (seconds) to freeze on the very last frame.
        """
        if not self.frames: return
        duration = int(1000 / fps)
        
        # Convert pause duration in seconds to frame count
        pause_frames = int(final_pause * fps)
        
        # Duplicate the last frame to create the pause effect
        frames_to_save = self.frames + [self.frames[-1]] * pause_frames
        
        frames_to_save[0].save(
            filename, save_all=True, append_images=frames_to_save[1:], 
            optimize=True, duration=duration, loop=0
        )
        print(f"Animation successfully saved to {filename}")
