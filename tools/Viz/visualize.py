import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection, PatchCollection
from IPython.display import display, clear_output
from PIL import Image
import numpy as np
from typing import Iterable, List, Tuple, Optional

class LiveVisualizer:
    """
    Real-time visualization engine for search algorithms (AAAI Publication Quality).
    
    Manages a layered rendering pipeline to demonstrate state-space exploration:
    - Layer 1 (Bottom): CLOSED Set (Expanded nodes) - Transparent Grey.
    - Layer 2: Invalid/Blocked Primitives - Dashed Grey lines.
    - Layer 3: OPEN Set (Frontier nodes) - Transparent Orange.
    - Layer 4: Valid Motion Primitives - Transparent Blue lines.
    - Layer 5: Initial Extended Cells (Mesh pivots) - Transparent Magenta.
    - Layer 6 (Top): Final Trajectory - Solid Red line.
    """
    
    def __init__(self, draw_map_func, task_map, start, goal, theta_config=None,
                 title: str = "Search Process", figsize: Tuple[int, int] = (8, 8), dpi: int = 100,
                 fast_draw: bool = False):
        """
        fast_draw: If True, uses raster graphics (imshow) for updating cell states (Open/Closed).
                       This is significantly faster for large maps.
                       If False, uses vector patches (PatchCollection), which is sharper but slower.
        """
        
        plt.ioff() # Disable interactive mode to manage frame capture manually
        
        self.fig, self.ax = plt.subplots(figsize=figsize, dpi=dpi)
        self.frames = []
        self.fast_draw = fast_draw
        self.map_shape = (task_map.height, task_map.width)
        
        # --- Static Background ---
        # Draw grid, obstacles, start/goal configuration
        try:
            draw_map_func(task_map, start, goal, ax_existing=self.ax, theta_config=theta_config)
        except TypeError:
            draw_map_func(task_map, start, goal, ax_existing=self.ax)
            
        self.ax.set_title(title, fontsize=14, fontweight='bold')
        
        # --- Dynamic Layers (Collections) ---
        
        if self.fast_draw:
            # FAST MODE: Use a single RGBA image overlay for all cell states
            # Initialize fully transparent image
            self.overlay_data = np.zeros((task_map.height, task_map.width, 4))
            self.overlay_artist = self.ax.imshow(self.overlay_data, origin='upper', zorder=2)

            # Lines are always drawn as vectors (LineCollection is efficient enough for moderate counts)
            # Invalid Primitives
            self.invalid_lines_artist = LineCollection(
                [], colors='grey', linewidths=0.5, alpha=0.3, linestyles='dashed', zorder=1.5
            )
            self.ax.add_collection(self.invalid_lines_artist)
            
            # Valid Primitives
            self.lines_artist = LineCollection(
                [], colors='blue', linewidths=0.8, alpha=0.4, zorder=3
            )
            self.ax.add_collection(self.lines_artist)
            
            # Final Path
            self.final_path_artist = LineCollection(
                [], colors='red', linewidths=2.5, alpha=1.0, zorder=10, label='Solution'
            )
            self.ax.add_collection(self.final_path_artist)

        else:
            # 1. CLOSED Set (Expanded)
            self.closed_artist = PatchCollection(
                [], facecolors='grey', alpha=0.25, edgecolors='none', zorder=1, label='Closed'
            )
            self.ax.add_collection(self.closed_artist)
            
            # 2. Invalid Primitives (Collisions)
            self.invalid_lines_artist = LineCollection(
                [], colors='grey', linewidths=0.8, alpha=0.5, linestyles='dashed', zorder=1.5
            )
            self.ax.add_collection(self.invalid_lines_artist)

            # 3. OPEN Set (Frontier)
            self.open_artist = PatchCollection(
                [], facecolors='orange', alpha=0.5, edgecolors='none', zorder=2, label='Open'
            )
            self.ax.add_collection(self.open_artist)
            
            # 4. Valid Primitives (Explored Edges)
            self.lines_artist = LineCollection(
                [], colors='blue', linewidths=0.8, alpha=0.4, zorder=3
            )
            self.ax.add_collection(self.lines_artist)
            
            # 5. Initial Extended Cells (Pivot States in Mesh Graph)
            self.initials_artist = PatchCollection(
                [], facecolors='magenta', alpha=0.15, edgecolors='none', zorder=4, label='Initial Cells'
            )
            self.ax.add_collection(self.initials_artist)
            
            # 6. Final Path
            self.final_path_artist = LineCollection(
                [], colors='red', linewidths=2.5, alpha=1.0, zorder=10, label='Solution'
            )
            self.ax.add_collection(self.final_path_artist)

        # Accumulators for vector graphics
        self.accumulated_valid = []
        self.accumulated_invalid = []
        
        self._capture_frame()
        display(self.fig)

    def update(self, 
               open_cells: Iterable[Tuple[int, int]], 
               closed_cells: Iterable[Tuple[int, int]], 
               valid_segments: List[Tuple[np.ndarray, np.ndarray]], 
               invalid_segments: Optional[List[Tuple[np.ndarray, np.ndarray]]] = None,
               initial_cells: Optional[Iterable[Tuple[int, int]]] = None):
        """
        Updates the visualization state.
        
        Args:
            open_cells: Set of (i, j) tuples currently in OPEN.
            closed_cells: Set of (i, j) tuples currently in CLOSED.
            valid_segments: List of new valid primitive geometries [(xs, ys), ...].
            invalid_segments: List of new invalid primitive geometries (optional).
            initial_cells: Set of (i, j) tuples identified as Initial Extended Cells (MeshA* specific).
        """
        # 1. Update Primitives (Accumulate to avoid re-uploading full history every frame if possible, 
        # but set_paths requires full list. We append locally.)
        for xs, ys in valid_segments:
            self.accumulated_valid.append(np.column_stack([xs, ys]))
        self.lines_artist.set_paths(self.accumulated_valid)
        
        if invalid_segments:
            for xs, ys in invalid_segments:
                self.accumulated_invalid.append(np.column_stack([xs, ys]))
            self.invalid_lines_artist.set_paths(self.accumulated_invalid)
        
        # 2. Update Grid Cells
        # Note: Grid coordinates (i, j) map to plot coordinates (x=j, y=i).
        # We shift by -0.5 because grid cells are centered at integers in the drawing function.
        if self.fast_draw:
            # --- RASTER UPDATE (FAST) ---
            # We modify the pixels of the overlay image directly
            
            # Extract indices efficiently
            if closed_cells:
                rows_c, cols_c = zip(*closed_cells)
                # Grey with alpha=0.2 -> RGBA (0.5, 0.5, 0.5, 0.2)
                self.overlay_data[rows_c, cols_c] = [0.5, 0.5, 0.5, 0.2]
                
            if open_cells:
                rows_o, cols_o = zip(*open_cells)
                # Orange with alpha=0.5 -> RGBA (1.0, 0.65, 0.0, 0.5)
                self.overlay_data[rows_o, cols_o] = [1.0, 0.65, 0.0, 0.5]

            if initial_cells:
                rows_i, cols_i = zip(*initial_cells)
                # Magenta with alpha=0.5 -> RGBA (1.0, 0.0, 1.0, 0.5)
                # Note: This overwrites previous colors, acting as "layering"
                self.overlay_data[rows_i, cols_i] = [1.0, 0.0, 1.0, 0.5]
            
            # Push updates to the artist
            self.overlay_artist.set_data(self.overlay_data)
            
        else:
            if closed_cells:
                self.closed_artist.set_paths([patches.Rectangle((j-0.5, i-0.5), 1, 1) for i, j in closed_cells])
            
            if open_cells:
                self.open_artist.set_paths([patches.Rectangle((j-0.5, i-0.5), 1, 1) for i, j in open_cells])
                
            if initial_cells:
                self.initials_artist.set_paths([patches.Rectangle((j-0.5, i-0.5), 1, 1) for i, j in initial_cells])
        
        self._capture_frame()
        clear_output(wait=True)
        display(self.fig)

    def animate_final_path(self, path_geometry: List[Tuple[np.ndarray, np.ndarray]]):
        """
        Sequentially draws the final trajectory to create a smooth 'solution found' effect.
        """
        accumulated_path = []
        for xs, ys in path_geometry:
            accumulated_path.append(np.column_stack([xs, ys]))
            self.final_path_artist.set_paths(accumulated_path)
            self._capture_frame()
            clear_output(wait=True)
            display(self.fig)

    def _capture_frame(self):
        """Captures the current canvas state into a PIL Image."""
        self.fig.canvas.draw()
        
        # Convert RGBA buffer to RGB image
        buf = self.fig.canvas.buffer_rgba()
        arr = np.asarray(buf)
        img = Image.fromarray(arr)
        
        # Create a white background to prevent alpha artifacts in GIF
        bg = Image.new("RGB", img.size, (255, 255, 255))
        bg.paste(img, mask=img.split()[3]) 
        
        self.frames.append(bg)

    def save_gif(self, filename: str = "search.gif", fps: int = 10, final_pause: int = 20):
        """Exports the recorded frames as a GIF."""
        if not self.frames:
            print("No frames recorded to save.")
            return

        duration = int(1000 / fps)
        # Repeat the last frame for the 'final_pause' duration
        frames_to_save = self.frames + [self.frames[-1]] * final_pause
        
        frames_to_save[0].save(
            filename, save_all=True, append_images=frames_to_save[1:], 
            optimize=True, duration=duration, loop=0
        )
        print(f"Animation successfully saved to {filename}")
