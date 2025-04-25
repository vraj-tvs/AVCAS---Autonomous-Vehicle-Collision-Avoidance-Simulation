import casadi as ca
import numpy as np
import os
import sys
import warnings
import contextlib

@contextlib.contextmanager
def suppress_all_output():
    with open(os.devnull, 'w') as fnull:
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = fnull
        sys.stderr = fnull
        try:
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr

class MPC:
    def __init__(self, dt=0.2, N_p=20, N_c=3):
        """Initialize MPC controller with prediction and control horizons"""
        self.dt = dt
        self.N_p = N_p  # Prediction horizon
        self.N_c = N_c  # Control horizon
        
        # Define bounds for acceleration inputs
        self.a_ex_min = -4.0  # m/s²
        self.a_ex_max = 2.0   # m/s²
        self.a_ey_min = -2.0  # m/s²
        self.a_ey_max = 2.0   # m/s²
        self.da_ex_max = 1.5  # m/s³
        self.da_ey_max = 0.5  # m/s³
        
        # Define weights for cost function (Equation 14)
        self.Q_lat = 100.0   # Weight for lateral position error
        self.Q_vel = 10.0  # Weight for velocity error
        self.R_da_ex = 0.1  # Weight for longitudinal jerk
        self.R_da_ey = 0.1  # Weight for lateral jerk
        self.chi = 1000.0  # Weight for slack variables
        
        # Define road parameters
        self.y_min = 0.0  # m
        self.y_max = 7.0  # m
        self.vx_min = 0.0  # m/s
        self.vx_max = 40.0  # m/s
        self.beta_max = np.tan(np.radians(10))  # Maximum slip angle
        
        # Initialize CasADi optimizer
        self.setup_optimizer()
        
    def setup_optimizer(self):
        """Set up the CasADi optimizer for MPC"""
        # Define symbolic variables
        self.opti = ca.Opti()
        
        # Define states over the prediction horizon
        self.X = self.opti.variable(self.N_p)       # longitudinal position
        self.Y = self.opti.variable(self.N_p)       # lateral position
        self.v_x = self.opti.variable(self.N_p)     # longitudinal velocity
        self.v_y = self.opti.variable(self.N_p)     # lateral velocity
        
        # Control inputs over the control horizon
        self.a_ex = self.opti.variable(self.N_c)    # longitudinal acceleration
        self.a_ey = self.opti.variable(self.N_c)    # lateral acceleration
        self.da_ex = self.opti.variable(self.N_c-1) # longitudinal jerk
        self.da_ey = self.opti.variable(self.N_c-1) # lateral jerk

        # Slack variables
        self.slack_y = self.opti.variable(self.N_p)       # Road boundary slack
        self.slack_barrier = self.opti.variable(self.N_p) # Barrier slack

    def solve(self, ego_vehicle, surrounding_vehicles, sigmoid_barrier, decision_maker, y_ref, v_des):
        """Solve the MPC optimization problem with full constraints"""

        # Validate inputs
        if ego_vehicle is None or len(ego_vehicle.state) < 6:
            raise ValueError("Invalid ego vehicle state")
        
        # Initialize optimization problem
        self.opti = ca.Opti()
        self.setup_optimizer()

        # Extract initial state
        x0, y0, vx0, vy0 = ego_vehicle.state[0], ego_vehicle.state[1], ego_vehicle.state[2], ego_vehicle.state[3]
        
        # Define slack variables for collision avoidance
        xi_x = []
        for _ in range(len(surrounding_vehicles)):
            xi_x.append(self.opti.variable(self.N_p))
        
        # Cost function (Equation 14-15)
        cost = 0
        for i in range(self.N_p):
            # Tracking costs
            cost += self.Q_lat * (self.Y[i] - y_ref)**2
            cost += self.Q_vel * (self.v_x[i] - v_des)**2
            # Slack penalties
            cost += self.chi * self.slack_y[i]**2
            cost += self.chi * self.slack_barrier[i]**2
            
        for i in range(self.N_c-1):
            # Control smoothness
            cost += self.R_da_ex * self.da_ex[i]**2
            cost += self.R_da_ey * self.da_ey[i]**2

        # State error cost
        for i in range(self.N_p):
            # Slack variable cost
            for j in range(len(surrounding_vehicles)):
                cost += self.chi * xi_x[j][i]**2
        
        # Set objective    
        self.opti.minimize(cost)
                
        # Initial constraints
        self.opti.subject_to(self.X[0] == x0)
        self.opti.subject_to(self.Y[0] == y0)
        self.opti.subject_to(self.v_x[0] == vx0)
        self.opti.subject_to(self.v_y[0] == vy0)
        
        # Dynamic constraints over prediction horizon
        for i in range(self.N_p - 1):
            # Control input for current step
            a_ex_i = self.a_ex[min(i, self.N_c-1)]
            a_ey_i = self.a_ey[min(i, self.N_c-1)]
            
            # Ego vehicle dynamics (Equation 8)
            self.opti.subject_to(self.v_x[i+1] == self.v_x[i] + a_ex_i * self.dt)
            self.opti.subject_to(self.v_y[i+1] == self.v_y[i] + a_ey_i * self.dt)
            self.opti.subject_to(self.Y[i+1] == self.Y[i] + self.v_y[i] * self.dt)
            self.opti.subject_to(self.X[i+1] == self.X[i] + self.v_x[i] * self.dt)
                    
        # Physical constraints (Equations 16-21)
        for i in range(self.N_c):
            # Acceleration limits
            self.opti.subject_to(self.a_ex[i] >= self.a_ex_min)
            self.opti.subject_to(self.a_ex[i] <= self.a_ex_max)
            self.opti.subject_to(self.a_ey[i] >= self.a_ey_min)
            self.opti.subject_to(self.a_ey[i] <= self.a_ey_max)

        for i in range(self.N_c - 1):
            # Longitudinal jerk constraint
            self.opti.subject_to(self.da_ex[i] == self.a_ex[i+1] - self.a_ex[i])
            self.opti.subject_to(self.da_ex[i] <= self.da_ex_max)
            self.opti.subject_to(self.da_ex[i] >= -self.da_ex_max)
            # Lateral jerk constraint
            self.opti.subject_to(self.da_ey[i] == self.a_ey[i+1] - self.a_ey[i])
            self.opti.subject_to(self.da_ey[i] <= self.da_ey_max)
            self.opti.subject_to(self.da_ey[i] >= -self.da_ey_max)
        
        # Safety constraints (Equations 23-27)
        for i in range(self.N_p):
            # Road boundaries with slack
            vehicle_half_width = ego_vehicle.width / 2
            self.opti.subject_to(self.Y[i] >= (self.y_min + vehicle_half_width) - self.slack_y[i])
            self.opti.subject_to(self.Y[i] <= (self.y_max - vehicle_half_width) + self.slack_y[i])
            
            # Slip angle constraint (Eq. 21)
            self.opti.subject_to(self.v_y[i] >= -self.v_x[i] * self.beta_max)
            self.opti.subject_to(self.v_y[i] <= self.v_x[i] * self.beta_max)
            
            # Get activation signals from decision maker
            delta, eta, _ = decision_maker.determine_activation_signals(ego_vehicle, surrounding_vehicles)
        
            for j, veh in enumerate(surrounding_vehicles):
                # Sigmoid barrier constraints
                s_f = decision_maker.TIV * self.v_x[i]
                delta_x = veh.state[0] - self.X[i]
                delta_y = veh.state[1] - self.Y[i]
                barrier = sigmoid_barrier.generate_barrier(delta_x = delta_x, s_f = s_f, delta = delta)
                conditional_exp = ca.if_else(delta_y <= 1, -delta_y, delta_y)
                self.opti.subject_to(conditional_exp >= barrier - self.slack_barrier[i])
                
                # Braking constraint with Big-M method
                M = 1e3
                kappa = 10
                self.opti.subject_to(delta_x + kappa * xi_x[j][i] >= decision_maker.TTC * self.v_x[i] - M * eta)

            # Slack variable constraints (Section VII-D)
            self.opti.subject_to(self.slack_y[i] >= 0)
            self.opti.subject_to(self.slack_barrier[i] >= 0)
            for j in range(len(surrounding_vehicles)):
                self.opti.subject_to(xi_x[j][i] >= 0)
        
        # Set solver options
        p_opts = {"expand": True, "print_time": False}
        s_opts = {"print_level": 0, "max_iter": 1000, "acceptable_tol": 1e-4, "acceptable_obj_change_tol": 1e-4}
        self.opti.solver("ipopt", p_opts, s_opts)
        
        # ---- Initial Conditions ----
        self.opti.set_initial(self.X, x0)
        self.opti.set_initial(self.Y, y0)
        self.opti.set_initial(self.v_x, vx0)
        self.opti.set_initial(self.v_y, vy0)

        try:
            # Solve the optimization problem
            with suppress_all_output():
                sol = self.opti.solve()
            
            # Extract optimal control inputs for the first step
            a_ex_opt = sol.value(self.a_ex[0])
            a_ey_opt = sol.value(self.a_ey[0])
            
            return a_ex_opt, a_ey_opt, []
            
        except Exception as e:
            # print(f"Optimization failed: {e}")
            warnings.filterwarnings("ignore")
            return -2.0, 0.0, []  # Fallback values [Emergency Braking]
