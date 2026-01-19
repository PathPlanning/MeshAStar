# MeshA\*: Efficient Path Planning With Motion Primitives

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Conference](https://img.shields.io/badge/Conference-AAAI_2026-blue)](https://aaai.org/)
[![Language](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)
[![Language](https://img.shields.io/badge/Python-3.8%2B-3776AB.svg)](https://www.python.org/)
[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-F37626.svg)](https://jupyter.org/)

This repository contains the official implementation of **MeshA\***, a kinodynamic path planning algorithm presented at the **AAAI Conference on Artificial Intelligence (2026)**.

MeshA* solves path planning problems with differential constraints using a finite set of motion primitives. Unlike conventional **Lattice-Based A\* (LBA\*)**, which searches directly over the state lattice, MeshA* operates on **Extended Cells**. This abstraction allows the algorithm to perform early pruning of unpromising trajectory bundles while guaranteeing both completeness and solution optimality. Empirically, MeshA* reduces runtime by **1.5–2x** compared to state-of-the-art baselines in cluttered environments.

## Visual Comparison

The following animations demonstrate the qualitative difference in state-space exploration between the baseline and the proposed method.
* **LBA\*** (Left): Exhibits a high branching factor, generating full bundles of primitives at every expanded state.
* **MeshA\*** (Right): Propagates as a wavefront across grid cells. Primitive bundles are only generated at specific "Pivot" states (Initial Extended Cells), allowing for significant pruning of the search space.

<p align="center">
<table>
  <tr>
    <td align="center"><b>Lattice-Based A* (Baseline)</b></td>
    <td align="center"><b>MeshA* (Proposed)</b></td>
  </tr>
  <tr>
    <td align="center"><img src="images/lba_demo.gif" width="400" alt="LBA Search"></td>
    <td align="center"><img src="images/mesh_demo.gif" width="400" alt="MeshA Search"></td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <em>
        Visualization of the search frontier on a cluttered map.<br>
        (Animations generated using the visualization tools provided in this repository).
      </em>
    </td>
  </tr>
</table>
</p>

## Repository Structure

...


## Citation

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
