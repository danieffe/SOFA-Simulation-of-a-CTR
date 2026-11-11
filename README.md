# SOFA Simulation of a Concentric Tube Robot (CTR) for Liver Cancer Treatment

This project simulates the deployment of a Concentric Tube Robot (CTR) within a human heart model using the SOFA (Simulation Open Framework Architecture) framework. The simulation is designed to assist in the planning and visualization of minimally invasive surgical procedures, focusing on the physical interaction between the robot and deformable tissues.

## Overview

The simulation models the interaction between a flexible, multi-tube catheter (the CTR) composed of three concentric tubes and the deformable tissue of the heart. The goal is to accurately navigate the CTR, generate its workspace, and analyze the contact forces during tissue interaction.

### Key Features
*   **Volumetric Mesh Generation:** Transforms a 3D surface model of the heart (.obj) into a tetrahedral volumetric mesh for Finite Element Method (FEM) analysis, allowing for realistic deformations.
*   **Deformable Heart Model:** Utilizes a biomechanical model of the heart as an elastic object, with configurable physical properties (stiffness, Young's modulus, mass) and boundary conditions (fixed boxes).
*   **CTR Kinematics & Modeling:** Implements a Concentric Tube Robot made of three tubes. The model defines visual and mechanical properties, including `WireRestShape` (Straight and Spire sections) and `EdgeSetGeometryAlgorithms`.
*   **Interactive Keyboard Controller:** Features a custom controller (`onKeypressedEvent`) that allows users to manually select (keys 1, 2, 3), drive, and steer (keys j, k, m, n) the CTR in real-time. It also includes a function to automatically generate the workspace (key w).
*   **Collision Detection and Response:** Simulates the physical interaction between the surgical instrument and the heart tissue using `TriangleCollisionModel`, `LineCollisionModel`, and `PointCollisionModel`.
*   **Force Feedback Tuning:** The simulation aims to reach a specific contact force (~0.4 N) by tuning the material properties of the robot's tubes using the `GenericConstraintSolver`.

## Concentric Tube Specifications

The CTR is modeled with the following physical parameters for its three tubes:

| Parameter (mm) | Tube 1 (Outer) | Tube 2 (Middle) | Tube 3 (Inner) |
| :--- | :--- | :--- | :--- |
| **Straight Length** | 115 | 140 | 175 |
| **Curved Length** | 40 | 50 | 65 |
| **Tube Radius** | 1.0 (adjusted to 0.9) | 0.85 | 0.7 |
| **Radius of Curvature** | 105 | 115 | 135 |

## Simulation Analysis & Results

### 1. Heart Interaction Comparison

We tested the CTR interaction against two different material stiffness profiles for the heart:

| Contact with Rigid Heart | Contact with Deformable Heart |
| :---: | :---: |
| <img src="video%20e%20immagini/contatto_cuore_rigido.gif" width="400" alt="Rigid Heart Interaction"> | <img src="video%20e%20immagini/contatto_cuore_cedevole.gif" width="400" alt="Deformable Heart Interaction"> |
| **$E = 60 \times 10^6$ Pa**<br>Minimal deformation, higher contact force. | **$E = 5 \times 10^5$ Pa**<br>Evident deformation, lower contact force. |

### 2. Parameter Tuning for Target Contact Force

A key objective of this project was to adjust the mechanical parameters to achieve a stable contact force of approximately **0.4 N** between the CTR and the heart. 

After initial simulations, the following parameters of the **Outer Tube (Tube 1)** were varied to achieve this goal:
*   **Outer Tube Radius:** Decreased from **$1.0$ mm to $0.9$ mm** to increase the transmitted force due to a larger cross-section constraint.
*   **Outer Tube Young's Modulus:** Decreased from **$1 \times 10^9$ Pa to $1.75 \times 10^6$ Pa** to adjust the stiffness of the material.

Every combination was tested until a stable contact force of ~$0.41$ N was reached. The final force was validated in real-time using the custom `register_force_value()` function during the simulation.

### 3. Workspace Simulation Overview

The workspace of the instrument is generated automatically via a series of nested `for` loops that execute all possible combinations of translational and rotational movements for the three tubes. At each iteration, the Cartesian coordinates of the instrument's tip are saved to a CSV file to map the reachable volume.

Analyzing the generated 3D workspace plots revealed how the curvature of tubes 2 and 3 affects the overall shape:
*   A **13% modification** of the parameters results in minimal differences compared to the nominal workspace.
*   A **50% modification** (decreasing the curvature parameter) drastically widens the curvature radius, resulting in a noticeably broader and larger overall workspace shape.

*(Simulation Overview from Oct 13, 2025)*<br>
<img src="video%20e%20immagini/Video%20del%2013-10-2025%2010%3A38%3A39.gif" width="600" alt="Simulation Overview">

## Prerequisites and Installation

To run this simulation, you need to have the SOFA framework installed with Python support.

1.  **Install SOFA:** Download and install SOFA from the official website ([https://www.sofa-framework.org/](https://www.sofa-framework.org/)). Ensure that you enable the Python3 plugin during installation.
2.  **Required Plugins:** This simulation relies on several SOFA plugins. Ensure the following are enabled in your SOFA Plugin Manager or loaded via your script:
    *   `SofaPython3`
    *   `Sofa.Component.StateContainer`
    *   `Sofa.Component.Topology.Container.Constant`
    *   `Sofa.Component.Visual`
    *   `Sofa.Component.SolidMechanics.FEM.Elastic`
    *   `Sofa.Component.Constraint.Projective`
    *   `Sofa.Component.Collision.Geometry`
    *   `Sofa.Component.Collision.Detection.Algorithm`
    *   `Sofa.Component.Collision.Detection.Intersection`
    *   `Sofa.Component.Collision.Response.Contact`

## How to Run and Interact

1. You can run the simulation scene directly using the `runSofa` executable provided with your SOFA installation:

```bash
runSofa gruppo_6.py
```

2. Once the SOFA GUI opens, press the "Animate" button to start the simulation.
3. Click inside the viewer window to ensure it has focus, then use the **keyboard controls**:
    *   `1`, `2`, `3`: Select the active tube (Outer, Middle, Inner).
    *   `j`, `k`: Translate the selected tube backwards and forwards (steps of ~1mm).
    *   `m`, `n`: Rotate the selected tube clockwise and counter-clockwise (steps of ~0.4 rad or ~22.5°).
    *   `w`: Automatically generate the workspace points and export them to a `work_space_1.txt` / CSV file.
