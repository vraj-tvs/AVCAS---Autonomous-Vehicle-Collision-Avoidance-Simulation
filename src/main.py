from Env import Environment
from Simulation import Simulation
from Plot_Results import plot_simulation_results

def main():
    # Create environment
    env = Environment(y_min=0, y_max=3, num_lanes=2, lane_width=1.5)
    
    # Choose scenario (1, 2, or 3 from the paper)
    scenario = 1
    
    # Create simulation
    sim = Simulation(dt=0.2, sim_time=30, environment=env, scenario_num = scenario)
    
    # Run simulation
    sim.run(visualize=True)

    plot_simulation_results()
    
if __name__ == "__main__":
    main()
