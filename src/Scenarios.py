from Vehicle_Dynamics import Vehicle
from Env import Environment

def setup_scenario(self, scenario_num):
    """Set up a specific scenario based on Table II in the paper"""
    # Reset vehicles
    self.surrounding_vehicles = []
    
    # Initialize ego vehicle
    env = Environment()
    ego_lane = 0  # Right lane
    ego_y = env.get_lane_center(lane_idx=ego_lane)
    
    # for the specified scenario (1, 2, or 3)
    if scenario_num == 1:
        # Scenario 1: Overtaking slow vehicle with free adjacent lane
        ego_x = 0
        ego_v_x = 25  # m/s
        ego_v_y = 0
        ego_a_x = 0
        ego_a_y = 0
        self.ego_vehicle = Vehicle([ego_x, ego_y, ego_v_x, ego_v_y, ego_a_x, ego_a_y], id='ego')
        
        # Front vehicle in same lane
        veh1_x = ego_x + 180
        veh1_y = ego_y
        veh1_v_x = 28  # m/s
        veh1_v_y = 0
        veh1_a_x = -4  # m/s²
        veh1_a_y = 0
        self.surrounding_vehicles.append(Vehicle([veh1_x, veh1_y, veh1_v_x, veh1_v_y, veh1_a_x, veh1_a_y], id='veh1'))
        
        # Vehicle in adjacent lane
        veh2_x = ego_x + 250
        veh2_y = env.get_lane_center(lane_idx=1)  # Left lane
        veh2_v_x = 31  # m/s
        veh2_v_y = 0
        veh2_a_x = 0
        veh2_a_y = 0
        self.surrounding_vehicles.append(Vehicle([veh2_x, veh2_y, veh2_v_x, veh2_v_y, veh2_a_x, veh2_a_y], id='veh2'))
        
    elif scenario_num == 2:
        # Scenario 2: Vehicle in danger zone, adjacent lane blocked
        ego_x = 0
        ego_v_x = 30  # m/s
        ego_v_y = 0
        ego_a_x = 0
        ego_a_y = 0
        self.ego_vehicle = Vehicle([ego_x, ego_y, ego_v_x, ego_v_y, ego_a_x, ego_a_y], id='ego')
        
        # Front vehicle in same lane
        veh1_x = ego_x + 100
        veh1_y = ego_y
        veh1_v_x = 18  # m/s
        veh1_v_y = 0
        veh1_a_x = 0  # m/s²
        veh1_a_y = 0
        self.surrounding_vehicles.append(Vehicle([veh1_x, veh1_y, veh1_v_x, veh1_v_y, veh1_a_x, veh1_a_y], id='veh1'))
        
        # Vehicle in adjacent lane
        veh2_x = ego_x - 50
        veh2_y = env.get_lane_center(lane_idx=1)  # Left lane
        veh2_v_x = 36  # m/s
        veh2_v_y = 0
        veh2_a_x = 0  # m/s²
        veh2_a_y = 0
        self.surrounding_vehicles.append(Vehicle([veh2_x, veh2_y, veh2_v_x, veh2_v_y, veh2_a_x, veh2_a_y], id='veh2'))
        
    elif scenario_num == 3:
        # Scenario 3: Critical situation
        ego_x = 0
        ego_v_x = 30  # m/s
        ego_v_y = 0
        ego_a_x = 0  # m/s²
        ego_a_y = 0
        self.ego_vehicle = Vehicle([ego_x, ego_y, ego_v_x, ego_v_y, ego_a_x, ego_a_y], id='ego')
        
        # Front vehicle in same lane
        veh1_x = ego_x + 80
        veh1_y = ego_y
        veh1_v_x = 18  # m/s
        veh1_v_y = 0
        veh1_a_x = -4  # m/s²
        veh1_a_y = 0
        self.surrounding_vehicles.append(Vehicle([veh1_x, veh1_y, veh1_v_x, veh1_v_y, veh1_a_x, veh1_a_y], id='veh1'))
        
        # Vehicle in adjacent lane
        veh2_x = ego_x - 100
        veh2_y = env.get_lane_center(lane_idx=1)  # Left lane
        veh2_v_x = 36  # m/s
        veh2_v_y = 0
        veh2_a_x = 0  # m/s²
        veh2_a_y = 0
        self.surrounding_vehicles.append(Vehicle([veh2_x, veh2_y, veh2_v_x, veh2_v_y, veh2_a_x, veh2_a_y], id='veh2'))

    return self.ego_vehicle, self.surrounding_vehicles