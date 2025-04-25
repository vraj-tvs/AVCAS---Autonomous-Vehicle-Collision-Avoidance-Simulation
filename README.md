# Autonomous Vehicle Collision Avoidance Simulation

This project implements a collision avoidance simulation framework for autonomous vehicles using Model Predictive Control (MPC) and vehicle dynamics. The simulation framework is based on the methodology described in our reference paper:

**Reference Paper:**  
*Title: A MPC COMBINED DECISION MAKING AND TRAJECTORY PLANNING FOR AUTONOMOUS VEHICHLE COLLISION AVOIDANCE*  
*Authors: Manel Ammour, Rodolfo Orjuela, and Michel Basset* 
*Published in: IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 23, NO. 12 DECEMBER 2022*

The project integrates several modules:
- **Vehicle Dynamics:** Implements the point-mass model for vehicles.
- **MPC Controller:** Formulates the MPC optimization problem using CasADi to compute control actions (accelerations & steering) to avoid collisions.
- **Simulation:** Manages the simulation loop, state updates, and logging of simulation data.
- **Environment & Visualization:** Contains the road model, lane definitions, and visualization utilities to plot vehicle trajectories and safety barriers.
- **Scenarios:** Sets up different driving scenarios (e.g., overtaking, lane changes) as described in Table II of the reference paper.
- **Plotting Results:** Utilities to visualize simulation data (e.g., trajectories, velocities, accelerations).

## Directory Structure

```
Project_Root/
├── src/
│   ├── Env.py
│   ├── Fsm.py
│   ├── main.py
│   ├── Mpc_Controller.py
│   ├── Plot_Results.py
│   ├── Scenarios.py
│   ├── Simulation.py
│   ├── Utils.py
│   ├── Vehicle_Dynamics.py
│   ├── simulation_data.npz  # generated after simulation run
├── README.md
├── .gitignore
└── requirements.txt
```

## Prerequisites

- **Python 3.8+**
- Recommended package dependencies are listed in the corresponding requirements file.

### Key Dependencies
- NumPy
- Matplotlib
- CasADi

## Setup and Installation

It is recommended to run the project in a Python virtual environment. Follow the steps below:

1. **Clone the repository:**

   ```bash
   git clone <repository_url>
   cd <repository_directory>
   ```

2. **Create a virtual environment:**

   On Windows:
   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```

   On macOS/Linux:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

   *(If you don't already have a `requirements.txt`, create one with the following basic entries:)*

   ```
   numpy>=1.18.0
   matplotlib>=3.3.0
   casadi>=3.5.5
   ```

## Compile and Run Instructions

### Running the Simulation
After setting up the virtual environment and installing dependencies, run the simulation by executing:

```bash
python src/main.py
```

The simulation will:
- Initialize the environment and scenarios using parameters from `Scenarios.py`.
- Compute vehicle dynamics and control actions using the MPC in `Mpc_Controller.py`.
- Log simulation data and update vehicle states in `Simulation.py`.
- Visualize trajectories and safety barriers using the functions in `Env.py`.

### Plotting Results
To generate graphical plots of the simulation logs, run:

```bash
python src/Plot_Results.py
```

This will load the simulation data from `simulation_data.npz` and produce plots such as trajectory charts, velocity profiles, and acceleration profiles.

## Project Overview

This simulation framework aims to demonstrate collision avoidance capabilities through:
- **MPC-based control:** Using CasADi, the optimization problem includes acceleration and jerk constraints, ensuring smooth vehicle maneuvers.
- **Dynamic Environment:** Realistic vehicle interactions are modeled using a point-mass model, with lane information and road boundaries visualized.
- **Scenario-based Testing:** The system can switch between multiple driving scenarios (as defined in `Scenarios.py`) to evaluate performance under diverse conditions.

Further details, including mathematical formulations and experimental results, can be found in the reference paper provided.

## Future Work

- Improve model fidelity by integrating more complex vehicle dynamics.
- Extend scenario diversity and include multi-agent interactions.
- Experiment with risk-averse formulations and further tuning of control parameters.

## Acknowledgments

This project was developed as part of research in autonomous vehicle collision avoidance. Special thanks to [collaborators, mentors, institutions, etc.].

---

For any questions or issues, please create an issue in this repository or contact the project maintainer.