import numpy as np

class SigmoidBarrier:
    def __init__(self, y_lat=3.5):
        """Initialize sigmoid barrier with lateral offset"""
        self.y_lat = y_lat
        self.zeta = None
        
    def optimize_zeta(self, v_x, mu=0.8, g=9.81, y_err=0.05, s=50):
        """
        Optimize zeta parameter based on vehicle dynamics (Equations 10-13)
        """
        if v_x < 1e-3:  # Prevent division by zero
            v_x = 1e-3
        # Calculate maximum yaw rate (Eq. 11)
        psi_dot_max = 0.85 * mu * g / v_x
        
        # Calculate maximum curvature (Eq. 12)
        rho_max = psi_dot_max / v_x
        
        # Calculate lambda value
        lambda_val = np.exp(1.3170)
        
        # Calculate zeta_max (Eq. 10)
        zeta_max = np.sqrt(rho_max * (lambda_val + 1)**3 / (self.y_lat * lambda_val * (lambda_val - 1)))
        
        # Calculate zeta_min (Eq. 13)
        zeta_min = -2 * np.log(y_err / (1 - y_err)) / s
        
        # Use mean value for zeta
        self.zeta = (zeta_min + zeta_max) / 2
        
        return self.zeta
        
    def generate_barrier(self, delta_x, s_f, delta=1):
        """Generate sigmoid barrier value (Equation 23)"""
        if self.zeta is None:
            raise ValueError("Zeta parameter not optimized. Call optimize_zeta first.")
            
        # Sigmoid function (Eq. 23)
        barrier = (delta * self.y_lat) / (1 + np.exp(-self.zeta * (-delta_x + s_f)))
        
        return barrier
