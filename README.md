# MeshA\*: Efficient Path Planning With Motion Primitives

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Conference](https://img.shields.io/badge/Conference-AAAI_2026-blue)](https://aaai.org/)
[![Language](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)
[![Language](https://img.shields.io/badge/Python-3.8%2B-3776AB.svg)](https://www.python.org/)
[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-F37626.svg)](https://jupyter.org/)

This repository contains the official implementation of **MeshA\***, a kinodynamic path planning algorithm presented at the **AAAI Conference on Artificial Intelligence (2026)**.

MeshA* solves path planning problems with differential constraints using a finite set of motion primitives. Unlike conventional **Lattice-Based A\* (LBA\*)**, which searches directly over the state lattice, MeshA* operates on **Extended Cells**. This abstraction allows the algorithm to perform early pruning of unpromising trajectory bundles while guaranteeing both completeness and solution optimality. Empirically, MeshA* reduces runtime by **1.5–2x** compared to state-of-the-art baselines in cluttered environments.


## 🔍 Visual Comparison

The following animations demonstrate the qualitative difference in state-space exploration between the baseline and the proposed method.

* **LBA\*** (Left): Exhibits a high branching factor, explicitly expanding full bundles of primitives at every visited state.
* **MeshA\*** (Right): Propagates as a wavefront across grid cells. The blue primitive bundles shown at Initial Extended Cells are visualized to illustrate the kinematic constraints guiding the search.

<p align="center">
<table>
  <tr>
    <td align="center"><b>Lattice-Based A* (Baseline)</b></td>
    <td align="center"><b>MeshA* (Proposed)</b></td>
  </tr>
  <tr>
    <td align="center"><img src="images/lba_demo.gif" width="400" alt="LBA Search"></td>
    <td align="center"><img src="images/mesha_demo.gif" width="400" alt="MeshA Search"></td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <em>
        Visualization of the search frontier on a small map.<br>
        (Animations generated using the <code>Search Visualization</code> tools provided in this repository).
      </em>
    </td>
  </tr>
</table>
</p>

## 📂 Repository Structure

The project is organized into a high-performance C++ core and a Python-based ecosystem for analysis and visualization:

```text
.
├── data/                       # Configuration files (e.g., control sets, primitives)
├── images/                     # Generated visual assets (GIFs, plots)
├── include/                    # Header files for the C++ implementation
├── maps/                       # Benchmark maps (MovingAI format) and scenarios
├── res/                        # Output directory for search results (logs and trajectories)
├── src/                        # Core C++ source code (High-performance MeshA* & LBA*)
├── tools/                      # Python ecosystem for analysis and pre-computation
│   ├── common/                 # Shared utilities (graphics, data structures)
│   ├── Search Visualization/   # Educational Python implementation of MeshA*/LBA* for interactive visualization (not optimized for speed)
│   ├── Experiment Process/     # Notebooks for analyzing benchmark results from 'res/' and generating paper tables/plots
│   ├── Generating control set/ # Tools for pre-computing custom motion primitives
│   ├── Numbering configurations/ # Pre-computation of Mesh Graph transitions (see Paper Appendix)
│   └── Playground/             # Easy Entry Point: Notebooks to compile/run C++ code and plot resulting trajectories
├── Makefile                    # Build configuration
└── README.md                   # Project documentation

```

## 🚀 Getting Started

### 1. Quick Start (Playground)

The easiest way to test the planner is via the **Playground**.

1. Navigate to `tools/Playground/`.
2. Open `Analysis.ipynb`.
3. This notebook automatically compiles the C++ code, runs the planner on a test scenario, and plots the resulting trajectory.

### 2. Building & Running via CLI

To build the project manually:

```bash
make mesh_astar
```

To view available command-line arguments:

```bash
./mesh_astar --help
```

To run a benchmark (requires configuring `src/KC_testing.cpp`):

```bash
./mesh_astar --mode benchmark
```

## 🔬 Research Workflow

If you intend to modify the algorithm or reproduce the full paper experiments, we recommend the following workflow:

1. **Generate Control Set:**
Create a desired control set of motion primitives (varying in expressiveness, length, etc.) using `tools/Generating control set/base_control_set_generation.ipynb`.
2. **Pre-compute Mesh Graph:**
Run the `run.py` script in `tools/Numbering configurations/` for your control set. This numbers the configurations and pre-calculates the transition table (as described in the Appendix).
3. **Validate Quality:**
Use `tools/Playground/` to generate single trajectories with your new control set and verify the path quality.
4. **Visualize Behavior:**
Use `tools/Search Visualization/` to generate animations and inspect how the new control set affects the search frontier and branching factor.
5. **Benchmark:**
Configure the scenarios in `src/KC_testing.cpp` and run `./mesh_astar --mode benchmark`. Finally, use `tools/Experiment Process/` to parse the results and generate performance tables.

## 📄 Citation

If you use this code or ideas in your research, please cite our AAAI paper:

```bibtex
@misc{agranovskiy2025meshaefficientpathplanning,
      title={MeshA*: Efficient Path Planning With Motion Primitives}, 
      author={Marat Agranovskiy and Konstantin Yakovlev},
      year={2025},
      eprint={2412.10320},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2412.10320}, 
}
```
