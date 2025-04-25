import matplotlib.pyplot as plt
import numpy as np

class Environment:
    def __init__(self, y_min=0, y_max=3, num_lanes=2, lane_width=1.5):
        """Initialize road environment with specified dimensions"""
        self.y_min = y_min
        self.y_max = y_max
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.lane_centers = [y_min + (i + 0.5) * lane_width for i in range(num_lanes)]
        
    def get_lane_center(self, lane_idx):
        """Get the center y-coordinate of a lane"""
        if 0 <= lane_idx < self.num_lanes:
            return self.lane_centers[lane_idx]
        else:
            raise ValueError(f"Invalid lane index: {lane_idx}")
            
    def visualize(self, vehicles=None, safety_barriers=None):
        """Visualize environment, vehicles, and trajectories"""
        plt.figure(figsize=(10, 5))
        
        # Draw road boundaries
        plt.axhline(y=self.y_min, color='k', linestyle='-', linewidth=2)
        plt.axhline(y=self.y_max, color='k', linestyle='-', linewidth=2)
        
        # Draw lane markings
        for i in range(1, self.num_lanes):
            y = self.y_min + i * self.lane_width
            plt.axhline(y=y, color='k', linestyle='--', linewidth=1)
            
        # Draw vehicles if provided
        if vehicles:
            for vehicle in vehicles:
                x, y = vehicle.state[0], vehicle.state[1]
                length, width = vehicle.length, vehicle.width
                
                # Draw vehicle as rectangle
                vehicle_rect = plt.Rectangle((x - length/2, y - width/2), length, width, 
                                           fill=True, color='r' if vehicle.id == 'ego' else 'b',
                                           alpha=0.7)
                plt.gca().add_patch(vehicle_rect)
                
                # Draw vehicle trajectory
                traj = np.array(vehicle.trajectory)
                plt.plot(traj[:, 0], traj[:, 1], 'g-' if vehicle.id == 'ego' else 'b-', alpha=0.5)
        
        # Draw safety barriers if provided
        if safety_barriers:
            for barrier_name, barrier_data in safety_barriers.items():
                x_vals, y_vals = barrier_data
                plt.plot(x_vals, y_vals, 'm-', linewidth=1, alpha=0.6)
                
        plt.grid(True)
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Autonomous Vehicle Collision Avoidance Simulation')
        plt.show()
