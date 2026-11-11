# SOFA Simulation of a Concentric Tube Robot (CTR) for Liver Cancer Treatment

This project simulates the deployment of a Concentric Tube Robot (CTR) within a human heart model using the SOFA (Simulation Open Framework Architecture) framework. The simulation is designed to assist in the planning and visualization of minimally invasive surgical procedures, focusing on the physical interaction between the robot and deformable tissues.

## Overview

The simulation models the interaction between a flexible, multi-tube catheter (the CTR) composed of three concentric tubes and the deformable tissue of the heart. The goal is to accurately navigate the CTR, generate its workspace, and analyze the contact forces during tissue interaction.

### Key Features

* **Volumetric Mesh Generation:** Transforms a 3D surface model of the heart (.obj) into a tetrahedral volumetric mesh for Finite Element Method (FEM) analysis, allowing for realistic deformations.
* **Deformable Heart Model:** Utilizes a biomechanical model of the heart as an elastic object, with configurable physical properties (stiffness, Young's modulus, mass) and boundary conditions (fixed boxes).
* **CTR Kinematics & Modeling:** Implements a Concentric Tube Robot made of three tubes. The model defines visual and mechanical properties, including `WireRestShape` (Straight and Spire sections) and `EdgeSetGeometryAlgorithms`.
* **Interactive Keyboard Controller:** Features a custom controller (`onKeypressedEvent`) that allows users to manually select (keys 1, 2, 3), drive, and steer (keys j, k, m, n) the CTR in real-time. It also includes a function to automatically generate the workspace (key w).
* **Collision Detection and Response:** Simulates the physical interaction between the surgical instrument and the tissue using Triangle, Line, and Point Collision Models.

### CTR Specifications

The Concentric Tube Robot is modeled with the following physical parameters:

| Parameter (mm) | Tube 1 (Outer) | Tube 2 (Middle) | Tube 3 (Inner) |
| :--- | :---: | :---: | :---: |
| **Straight Length** | 115 | 140 | 175 |
| **Curved Length** | 40 | 50 | 65 |
| **Tube Radius** | 1 | 0.85 | 0.7 |
| **Curvature Radius** | 105 | 115 | 135 |

## Repository Contents

* `gruppo6.py` / `gruppo_6.py`: The main Python scripts defining the SOFA scene, including the liver/heart model, the CTR, the boundary conditions, and the solver configurations.
* `team Project 6.pptx`: A presentation detailing the project's background, methodology, and results.
* `video e immagini/`: A directory containing visual demonstrations of the simulation.

## Demo & Visuals

### 1. Interaction Analysis: Rigid vs. Deformable Heart

The following animations compare the behavior of the CTR and the surrounding tissue when interacting with a rigid heart model versus a deformable (soft) heart model.

| Contact with Rigid Heart | Contact with Deformable Heart | 
| :---: | :---: | 
| ![Rigid Heart](video%20e%20immagini/contatto_cuore_rigido.gif) | ![Soft Heart](video%20e%20immagini/contatto_cuore_cedevole.gif) |
| **Young's Modulus (E) = 60 × 10⁶ Pa**<br>Minimal deformation of the heart, confirming its high stiffness. | **Young's Modulus (E) = 5 × 10⁵ Pa**<br>Evident tissue deformation and adaptation of the CTR. Lower contact force compared to the rigid case. |

### 2. Parameter Tuning for Target Contact Force

A key objective of this project was to adjust the mechanical parameters to achieve a stable contact force of approximately **0.4 N** between the CTR and the heart.

After initial simulations, the following parameters of the **Outer Tube (Tube 1)** were varied to achieve this goal:

* **Outer Tube Radius:** Decreased from **1.0 mm to 0.9 mm** to increase the transmitted force due to a larger cross-section constraint.
* **Outer Tube Young's Modulus:** Decreased from **1 × 10⁹ Pa to 1.75 × 10⁶ Pa** to adjust the stiffness of the material.

Every combination was tested until a stable contact force of **~0.41 N** was reached. The final force was validated in real-time using the custom `register_force_value()` function during the simulation.

### 3. Workspace Simulation Overview

The workspace of the instrument is generated automatically via a series of nested `for` loops that execute all possible combinations of translational and rotational movements for the three tubes. At each iteration, the Cartesian coordinates of the instrument's tip are saved to a CSV file to map the reachable volume.

Analyzing the generated 3D workspace plots revealed how the curvature of tubes 2 and 3 affects the overall shape:

| Nominal Workspace | 13% Modification | 50% Modification |
| :---: | :---: | :---: |
| ![Nominal](video%20e%20immagini/workspace_nominale.png) | ![13% Mod](video%20e%20immagini/workspace_minus_13.png) | ![50% Mod](video%20e%20immagini/workspace_minus_50.png) |
| Baseline parameters | **Minimal differences** compared to the nominal workspace. | Decreasing the curvature parameter drastically **widens** the curvature radius, resulting in a noticeably broader shape. |

*Note: In the graphs above, the color gradient indicates the extension of the third instrument.*

## Prerequisites and Installation

To run this simulation, you need to have the [SOFA framework](https://www.sofa-framework.org/download/) installed with Python support.

1. **Install SOFA:** Download and install the software from the official link above. Ensure that you enable the `Python3` plugin during installation.
2. **Required Plugins:** This simulation relies on several SOFA plugins. Ensure the following are enabled in your SOFA Plugin Manager or loaded via your script:
   * `SofaPython3`
   * `Sofa.Component.StateContainer`
   * `Sofa.Component.Topology.Container.Constant`
   * `Sofa.Component.Visual`
   * `Sofa.Component.SolidMechanics.FEM.Elastic`
   * `Sofa.Component.Constraint.Projective`
   * `Sofa.Component.Collision.Geometry`
   * `Sofa.Component.Collision.Detection.Algorithm`
   * `Sofa.Component.Collision.Detection.Intersection`
   * `Sofa.Component.Collision.Response.Contact`

## How to Run and Interact

1. You can run the simulation scene directly using the `runSofa` executable provided with your SOFA installation:

```bash
runSofa gruppo_6.py
```

2. Once the SOFA GUI opens, press the **"Animate"** button to start the simulation.
3. **Keyboard Control:** Click inside the 3D viewer window to ensure it has focus. You can then use the custom keyboard controller:
   * Press `1`, `2`, or `3` to select the tube you want to control.
   * Press `j`, `k`, `m`, `n` to steer and drive (translate/rotate) the selected tube.
   * Press `w` to automatically generate the workspace CSV file.
