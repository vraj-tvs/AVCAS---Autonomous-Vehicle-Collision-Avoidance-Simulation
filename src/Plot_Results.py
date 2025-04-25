import numpy as np
import matplotlib.pyplot as plt

def plot_simulation_results(log_file='simulation_data.npz'):
    # Load saved data
    data = np.load(log_file, allow_pickle=True)
    time = data['time']
    ego_y = data['ego_y']
    ego_vx = data['ego_vx']
    ego_ax = data['ego_ax']
    ego_ay = data['ego_ay']
    vehicles = data['vehicles'].item()
    
    # Create figure with subplots
    plt.figure(figsize=(12, 6))
    
    # Plot trajectory (Y position vs Time)
    plt.subplot(2, 2, 1)
    plt.plot(time, ego_y, label='Ego Vehicle', linestyle='-')
    for veh_id, veh_data in vehicles.items():
            plt.plot(time, veh_data['y'], label=f'{veh_id}', linestyle='-')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral [y] Position (m)')
    plt.title('Vehicle Trajectories')
    plt.grid(True)
    plt.legend()
    
    # Plot velocity profile
    plt.subplot(2, 2, 2)
    plt.plot(time, ego_vx, label='Ego Vehicle', linestyle='-')
    for veh_id, veh_data in vehicles.items():
        plt.plot(time, veh_data['vx'], label=f'{veh_id}', linestyle='-')
    plt.xlabel('Time (s)')
    plt.ylabel('v_x (m/s)')
    plt.title('Longitudinal Velocity Profiles')
    plt.grid(True)
    plt.legend()
    
    # Plot acceleration profile
    plt.subplot(2, 2, 3)
    plt.plot(time, ego_ax, label='Ego Vehicle', linestyle='-')
    plt.xlabel('Time (s)')
    plt.ylabel('a_x (m/s^2)')
    plt.title('Longitudinal Acceleration Profile')
    plt.grid(True)
    plt.legend()
    
    # Plot velocity profile
    plt.subplot(2, 2, 4)
    plt.plot(time, ego_ay, label='Ego Vehicle', linestyle='-')
    plt.xlabel('Time (s)')
    plt.ylabel('a_y (m/s^2)')
    plt.title('Lateral Acceleration Profile')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_simulation_results()
