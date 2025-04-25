from Mpc_Controller import MPC
import numpy as np

class Vehicle:
    def __init__(self, initial_state, length=1, width=0.5, id=None):
        """
        Initialize a vehicle with state [x, y, v_x, v_y, a_x, a_y]
        """
        # Ensure state has 6 elements
        if len(initial_state) != 6:
            raise ValueError("Initial state must contain 6 elements: [x, y, v_x, v_y, a_x, a_y]")
        self.state = np.array(initial_state, dtype=np.float64)
        self.length = length
        self.width = width
        self.id = id
        self.trajectory = [self.state.copy()]
        self.mpc = MPC()
        
    def update(self, a_x, a_y, dt):
        """Update vehicle state using point-mass model"""
        x, y, v_x, v_y, _, _ = self.state
        
        # Update velocities (Equations 8 from paper)
        v_x_new = np.clip(v_x + a_x * dt, 0, self.mpc.vx_max)
        v_y_new = np.clip(v_y + a_y * dt, 0, self.mpc.vx_max)
        
        # Update positions
        x_new = x + v_x_new * dt
        y_new = y + v_y_new * dt
        
        # Update state
        self.state = np.array([x_new, y_new, v_x_new, v_y_new, a_x, a_y])
        self.trajectory.append(self.state.copy())
        