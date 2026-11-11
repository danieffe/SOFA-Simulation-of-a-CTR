# SOFA Simulation of a Concentric Tube Robot (CTR) for Liver Cancer Treatment

This project simulates the deployment of a Concentric Tube Robot (CTR) within a human liver model using the SOFA (Simulation Open Framework Architecture) framework. The simulation is designed to assist in the planning and visualization of radiofrequency ablation (RFA) treatments for hepatocellular carcinoma (HCC).

## Overview

The simulation models the interaction between a flexible, multi-tube catheter (the CTR) and the deformable tissue of the liver. The goal is to accurately navigate the CTR to a targeted tumor location while accounting for tissue deformation caused by the insertion process and the breathing motion of the patient (e.g., interaction with the diaphragm and heart).

### Key Features
*   **Deformable Liver Model:** Utilizes a biomechanical model of the liver to simulate realistic tissue deformation during CTR insertion.
*   **CTR Kinematics:** Implements the kinematics of a Concentric Tube Robot, allowing for complex, non-linear paths within the tissue.
*   **Collision Detection and Response:** Simulates the physical interaction between the surgical instrument and the liver parenchyma.
*   **SOFA Framework:** Built upon the SOFA engine, leveraging its advanced physical modeling and simulation capabilities.

## Repository Contents

*   `gruppo6.py` / `gruppo_6.py`: The main Python scripts defining the SOFA scene, including the liver model, the CTR, the boundary conditions, and the solver configurations.
*   `team Project 6.pptx`: A presentation detailing the project's background, methodology, and results.
*   `video e immagini/`: A directory containing visual demonstrations of the simulation.

## Demo & Visuals

Here are some visual examples of the simulation in action:

### Workspace Generation Code
![Workspace Generation Code](video%20e%20immagini/codice_generazione_workspace.png)

### Simulation Videos
*Note: Depending on where you view this README, you may need to download the videos or use a compatible viewer if the embedded player does not work.*

**Contact with Deformable Heart:**
<video width="640" controls>
  <source src="video%20e%20immagini/contatto_cuore_cedevole.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

**Simulation Overview (Oct 13, 2025):**
<video width="640" controls>
  <source src="video%20e%20immagini/Video%20del%2013-10-2025%2010%3A38%3A39.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

## Prerequisites and Installation

To run this simulation, you need to have the SOFA framework installed with Python support.

1.  **Install SOFA:** Download and install SOFA from the official website (https://www.sofa-framework.org/download/). Ensure that you enable the Python3 plugin during installation.
2.  **Required Plugins:** This simulation relies on several SOFA plugins. Ensure the following are enabled in your SOFA Plugin Manager or loaded via your script:
    *   `SofaPython3`
    *   `Sofa.Component.StateContainer`
    *   `Sofa.Component.Topology.Container.Constant`
    *   `Sofa.Component.Visual`
    *   (Add any other specific plugins used in your `gruppo_6.py` file, such as those for collision, solvers, or constraints).

## How to Run

You can run the simulation scene directly using the `runSofa` executable provided with your SOFA installation:

```bash
runSofa gruppo_6.py
```

Once the SOFA GUI opens, press the "Animate" button to start the simulation.
