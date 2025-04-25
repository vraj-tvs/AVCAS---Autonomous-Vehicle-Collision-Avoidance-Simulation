from Scenarios import setup_scenario
from Env import Environment
from Utils import SigmoidBarrier
from Fsm import DecisionMaking
from Mpc_Controller import MPC
import numpy as np

class Simulation:
    def __init__(self, dt=0.2, sim_time=30, environment=None, scenario_num=1):
        """Initialize simulation with time step and duration"""
        self.dt = dt
        self.sim_time = sim_time
        self.environment = environment if environment else Environment()
        self.ego_vehicle, self.surrounding_vehicles = setup_scenario(self, scenario_num)
        self.sigmoid_barrier = SigmoidBarrier()
        self.decision_maker = DecisionMaking(TTC=2, TIV=4)
        self.mpc = MPC(dt=dt)
        self.time = 0
                
    def run(self, visualize=True):
        """Run the simulation"""
        print(f"Starting simulation...")
        
        # Reference lane and desired velocity
        y_ref = self.environment.get_lane_center(lane_idx=0)  # Right lane
        v_des = 30  # m/s
        
        # Number of time steps
        num_steps = int(self.sim_time / self.dt)
        
        # Initialize data logging
        self.time_history = []
        self.ego_y_history = []
        self.ego_vx_history = []
        self.ego_ax_history = []
        self.ego_ay_history = []
        self.vehicles_history = {veh.id: {'y': [], 'vx': []} for veh in self.surrounding_vehicles}
        
        # Initial optimization of sigmoid barrier parameter
        self.sigmoid_barrier.optimize_zeta(self.ego_vehicle.state[2])
        
        for step in range(num_steps):
            self.time = step * self.dt

            
            for veh in self.surrounding_vehicles:
                self.vehicles_history[veh.id]['y'].append(veh.state[1])
                self.vehicles_history[veh.id]['vx'].append(veh.state[2])
                
            # Update sigmoid barrier parameter
            self.sigmoid_barrier.optimize_zeta(self.ego_vehicle.state[2])
            
            # Solve MPC
            a_ex, a_ey, _ = self.mpc.solve(
                self.ego_vehicle,
                self.surrounding_vehicles,
                self.sigmoid_barrier,
                self.decision_maker,
                y_ref,
                v_des
            )

            # Apply control inputs with anti-windup
            a_ex = np.clip(a_ex, self.mpc.a_ex_min, self.mpc.a_ex_max)
            a_ey = np.clip(a_ey, self.mpc.a_ey_min, self.mpc.a_ey_max)
            
            # Update ego vehicle
            self.ego_vehicle.update(a_ex, a_ey, self.dt)
            
            # Update surrounding vehicles
            for vehicle in self.surrounding_vehicles:
                veh_a_x, veh_a_y = vehicle.state[4], vehicle.state[5]
                vehicle.update(veh_a_x, veh_a_y, self.dt)

            self.time_history.append(self.time)
            self.ego_y_history.append(self.ego_vehicle.state[1])
            self.ego_vx_history.append(self.ego_vehicle.state[2])
            self.ego_ax_history.append(self.ego_vehicle.state[4])
            self.ego_ay_history.append(self.ego_vehicle.state[5])
            
        # Visualize if required
        if visualize:
            self.environment.visualize(vehicles=self.surrounding_vehicles + [self.ego_vehicle])

         # Save data after simulation
        np.savez('simulation_data.npz',
                 time=np.array(self.time_history),
                 ego_y=np.array(self.ego_y_history),
                 ego_vx=np.array(self.ego_vx_history),
                 ego_ax=np.array(self.ego_ax_history),
                 ego_ay=np.array(self.ego_ay_history),
                 vehicles=self.vehicles_history)
                
        print(f"Simulation completed after {self.time:.2f}s")
