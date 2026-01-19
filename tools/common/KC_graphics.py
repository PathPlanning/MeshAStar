"""
This module contains core functions for visualizing kinematic trajectory search results:
rendering grids, trajectories, online animation during parameter tuning,
and the discrete workspace (map) with obstacles.
"""

import matplotlib.pyplot as plt
import matplotlib.axes
import matplotlib.patches as patches
import matplotlib.ticker as ticker
from matplotlib.colors import ListedColormap
from typing import Optional
import numpy as np
from typing import Optional, List, Union, Tuple
from IPython.display import clear_output

from KC_structs import ShortTrajectory, Primitive, DiscreteState, Theta
from KC_searching import Map


def draw_grid(ax: matplotlib.axes.Axes, 
              xs: int = -8, ys: int = -8, 
              xf: int = 8, yf: int = 8, 
              tick_step: int = 1) -> None:
    """ 
    Draws a grid and sets coordinate axis ticks.
    
    Args:
        ax: The matplotlib axes to draw on.
        xs, ys, xf, yf: Coordinates defining the bounding box of the grid.
        tick_step: Step size for axis ticks.
        
    Note: The grid is drawn such that integer coordinates represent cell centers 
    (grid lines are at half-integer coordinates).
    """
    
    # Fix for the warning: explicitly tell matplotlib to adjust the plot box, not the data limits
    ax.set_aspect('equal', adjustable='box') 
    
    ax.set(xlim=(xs, xf), xticks=np.arange(xs, xf, tick_step),
           ylim=(ys, yf), yticks=np.arange(ys, yf, tick_step))
    
    # Horizontal grid lines
    for y in np.arange(ys + 0.5, yf, 1):
        ax.axhline(y, color='grey', alpha=0.45)
        
    # Vertical grid lines
    for x in np.arange(xs - 0.5, xf, 1):
        ax.axvline(x, color='grey', alpha=0.45)


def plot_arrow(x: float, y: float, theta: float, 
               length: float = 1.0, width: float = 0.5, 
               fc: str = "r", ec: str = "k", 
               ax: Optional[matplotlib.axes.Axes] = None) -> None:
    """
    Draws an arrow indicating direction/heading.
    
    Args:
        x, y, theta: Position and heading angle (in radians).
        length, width: Dimensions of the arrow head.
        fc, ec: Face color and edge color.
        ax: Axes to draw on (uses plt if None).
    """
    
    board = plt if (ax is None) else ax
    board.arrow(x, y, length * np.cos(theta), length * np.sin(theta),
                fc=fc, ec=ec, head_width=width, head_length=width)


def show_trajectory(traj: ShortTrajectory, 
                    col: str = 'r', 
                    arrow: bool = True, 
                    ax: Optional[matplotlib.axes.Axes] = None) -> None:
    """
    Visualizes a trajectory.
    
    Args:
        traj: The ShortTrajectory object.
        col: Color string.
        arrow: Whether to draw an arrow at the final state.
        ax: Axes to draw on.
    """
    
    board = plt if (ax is None) else ax
    
    xc = traj.sample_x()
    yc = traj.sample_y()
    board.plot(xc, yc, "-" + col)
    
    if arrow:
        final = traj.goal
        plot_arrow(final.x, final.y, final.theta, fc=col, ax=ax)


def redraw_trajectory(prim: ShortTrajectory, iter: int, col: str = 'r') -> None:
    """
    Dynamically redraws a trajectory (useful for animations in Jupyter Notebooks
    during parameter tuning).
    """
    
    if iter % 5 != 0:
        return

    xc = prim.sample_x()
    yc = prim.sample_y()      
    x, y, theta = prim.goal.x, prim.goal.y, prim.goal.theta
    
    clear_output(wait=True)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.axis("equal")
    ax.grid(True)
    
    ax.plot(xc, yc, "-" + col)
    ax.arrow(x, y, 1 * np.cos(theta), 1 * np.sin(theta),
             fc=col, ec="k", head_width=0.5, head_length=0.5)
    
    plt.pause(0.01)
    fig.canvas.draw()
    plt.show()


def draw_primitive(prim: Primitive, 
                   col: str = 'r', 
                   arrow: bool = True, 
                   ax: Optional[matplotlib.axes.Axes] = None, 
                   theta_config: Optional[Theta] = None) -> None:
    """
    Draws a generated motion primitive.
    """
    
    board = plt if (ax is None) else ax
    xc = prim.x_coords
    yc = prim.y_coords
    
    if ax is not None:
        ax.axis("equal")
    else:
        plt.axis("equal")
    
    board.plot(xc, yc, "-" + col)
    
    if arrow and theta_config is not None:
        final = prim.goal
        # Convert discrete theta index to continuous radians
        angle_rad = theta_config.angles[final.theta]
        plot_arrow(final.j, final.i, angle_rad, fc=col, ax=ax)


def draw_task_map(task_map: Map, 
                  start: DiscreteState, 
                  goal: DiscreteState, 
                  dpi: int = 200, 
                  scale: float = 1.0, 
                  R: float = 0, 
                  theta_config: Optional[Theta] = None,
                  ax_existing=None) -> matplotlib.axes.Axes:
    """
    Original function to draw the map, obstacles, start, and goal.
    
    Method:
        Iterates through every cell in the map. If a cell is an obstacle, 
        it adds a matplotlib.patches.Rectangle to the plot.
    
    Pros:
        - Vector graphics (crisp at any zoom).
        - Exact representation of your grid logic.
    Cons:
        - Very slow on large maps (e.g., MovingAI 1000x1000) because it creates
          thousands of individual patch objects.
    """
    
    w, h = 10 * (task_map.width / max(task_map.width, task_map.height)), 10 * (task_map.height / max(task_map.width, task_map.height))
        
    if ax_existing is None:
        fig = plt.figure(figsize=(scale * w, scale * h), dpi=dpi)
        ax = fig.add_subplot(111)
    else:
        ax = ax_existing

    # ===== Draw Map Grid =====
    xs, ys = 0, 0
    xf, yf = task_map.width, task_map.height
    _d = int((xf - xs) / 30) + 1  
    
    # Invert Y axis so 'i' increases downwards
    ax.set(xlim=(xs-0.5, xf-0.5), xticks=np.arange(0, xf, _d),        
           ylim=(ys-0.5, yf-0.5)[::-1], yticks=np.arange(0, yf, _d))  
    # ax.axis('equal')     
    ax.set_aspect('equal')  # adjust box of coordinates

    for y in np.arange(ys-0.5, yf, 1):
        ax.axhline(y, color='grey', alpha=0.45)
    for x in np.arange(xs-0.5, xf, 1):
        ax.axvline(x, color='grey', alpha=0.45)

    # ===== Draw Obstacles (Vector Method) =====
    for i in range(task_map.height):
        for j in range(task_map.width):
            if not task_map.traversable(i, j):
                # i corresponds to y (inverted), j corresponds to x
                ax.add_patch(patches.Rectangle((j-0.5, i-0.5), 1, 1, color='g', alpha=0.25))

    # ===== Draw Start and Goal =====
    # Goal
    circle = patches.Circle((goal.j, goal.i), R, color='red', alpha=0.2)
    ax.add_patch(circle)
    if theta_config:
        plot_arrow(goal.j, goal.i, theta_config.angles[goal.theta], fc='red', ax=ax)
    ax.add_patch(plt.Rectangle((goal.j-0.5, goal.i-0.5), 1, 1, color='r', alpha=1))
    
    # Start
    ax.add_patch(patches.Rectangle((start.j-0.5, start.i-0.5), 1, 1, color='g', alpha=1))
    if theta_config:
        plot_arrow(start.j, start.i, theta_config.angles[start.theta], fc='green', ax=ax)
    ax.add_patch(plt.Rectangle((start.j-0.5, start.i-0.5), 1, 1, color='g', alpha=1))
    return ax


def draw_task_map_fast(task_map: Map, 
                       start: DiscreteState, 
                       goal: DiscreteState, 
                       dpi: int = 200, 
                       scale: float = 1.0, 
                       R: float = 0, 
                       theta_config: Optional[Theta] = None,
                       ax_existing: Optional[matplotlib.axes.Axes] = None) -> matplotlib.axes.Axes:
    """
    Optimized function to draw the map using Raster Graphics (imshow).
    Matches the visual style of the vector-based draw_task_map (green obstacles).

    Args:
        task_map: The Map object containing the binary grid data.
        start: Start state.
        goal: Goal state.
        dpi: Dots per inch.
        scale: Scaling factor for figure size.
        R: Goal radius.
        theta_config: Theta configuration.
        ax_existing: Optional existing axes to draw on.
    """
    
    # 1. Figure Setup
    aspect = task_map.width / task_map.height
    base_h = 8
    base_w = base_h * aspect
    
    if ax_existing is None:
        fig = plt.figure(figsize=(scale * base_w, scale * base_h), dpi=dpi)
        ax = fig.add_subplot(111)
    else:
        ax = ax_existing

    # 2. Draw Map (Optimized)
    # Mask free cells (0) so they are completely transparent
    obstacle_mask = np.ma.masked_where(task_map.cells == 0, task_map.cells)
    
    # Use a solid green colormap to match 'color="g"' from the original function.
    # We set alpha=0.4 to match the visual weight of the original's alpha=0.25 
    # (raster pixels often need slightly higher alpha to look as dense as vector patches).
    cmap_green = ListedColormap(['green'])
    
    ax.imshow(obstacle_mask, cmap=cmap_green, origin='upper', alpha=0.4, 
              extent=[-0.5, task_map.width - 0.5, task_map.height - 0.5, -0.5])

    # 3. Configure Axes (Smart Ticks)
    # MaxNLocator automatically skips numbers to prevent overlap
    locator_x = ticker.MaxNLocator(nbins=20, integer=True)
    locator_y = ticker.MaxNLocator(nbins=20, integer=True)
    
    ax.xaxis.set_major_locator(locator_x)
    ax.yaxis.set_major_locator(locator_y)
    
    # Invert Y axis limits to match the coordinate system (0 at top)
    ax.set_xlim(-0.5, task_map.width - 0.5)
    ax.set_ylim(task_map.height - 0.5, -0.5) 

    # Fast grid drawing (instead of iterating lines)
    ax.grid(which='major', color='grey', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.set_aspect('equal')
    
    # 4. Draw Start and Goal (Vector elements are fine here as there are only 2)
    # Goal
    circle = patches.Circle((goal.j, goal.i), R, color='red', alpha=0.2)
    ax.add_patch(circle)
    
    # Goal
    circle = patches.Circle((goal.j, goal.i), R, color='red', alpha=0.2)
    ax.add_patch(circle)
    if theta_config:
        plot_arrow(goal.j, goal.i, theta_config.angles[goal.theta], fc='red', ax=ax)
    ax.add_patch(plt.Rectangle((goal.j-0.5, goal.i-0.5), 1, 1, color='r', alpha=1))
    
    # Start
    ax.add_patch(patches.Rectangle((start.j-0.5, start.i-0.5), 1, 1, color='g', alpha=1))
    if theta_config:
        plot_arrow(start.j, start.i, theta_config.angles[start.theta], fc='green', ax=ax)
    ax.add_patch(plt.Rectangle((start.j-0.5, start.i-0.5), 1, 1, color='g', alpha=1))

    return ax
