class DecisionMaking:
    def __init__(self, TTC=2, TIV=4):
        """Initialize decision making with safety parameters"""
        self.TTC = TTC  # Time to collision (seconds)
        self.TIV = TIV  # Intervehicular time (seconds)
        self.current_state = "Lane Keeping"  # Initial state
        self.delta = 0  # Lane change activation
        self.eta = 1    # Braking activation (0 = brake, 1 = no brake)
        self.lane_change_progress = 0  # 0-100% completion
        self.abort_flag = False

    def determine_activation_signals(self, ego_vehicle, surrounding_vehicles):
        """Determine activation signals based on FSM from Figure 3"""
        # Extract ego vehicle state
        ego_x, ego_y = ego_vehicle.state[0], ego_vehicle.state[1]
        ego_v_x = ego_vehicle.state[2]
        
        # Initialize variables
        adjacent_lane_blocked = False
        front_vehicle = None
        emergency_brake = False
        
        # Calculate safety distances
        s_f = self.TIV * ego_v_x  # Danger zone
        s_long = self.TTC * ego_v_x  # Mitigation zone
        
        # Identify relevant vehicles
        for vehicle in surrounding_vehicles:
            veh_x, veh_y = vehicle.state[0], vehicle.state[1]
            delta_x = veh_x - ego_x
            delta_y = veh_y - ego_y
            
            # Check emergency braking condition
            if abs(delta_y) <= 1 and 0 < delta_x < 5:  # Immediate collision risk
                emergency_brake = True
                
            # Front vehicle in same lane
            if abs(delta_y) <= 0.5 and delta_x > 0:
                if front_vehicle is None or delta_x < front_vehicle[0]:
                    front_vehicle = (delta_x, vehicle)
            
            # Adjacent lane vehicles
            if abs(abs(delta_y) - 3.5) < 0.5 and -20 < delta_x < 50:
                adjacent_lane_blocked = True

        # FSM state transitions (Figure 3 logic)
        if self.current_state == "Lane Keeping":
            if emergency_brake:
                self.current_state = "Emergency Braking"
                self.delta = 0
                self.eta = 0
            elif front_vehicle and front_vehicle[0] < s_f:
                if not adjacent_lane_blocked:
                    self.current_state = "Lane Change"
                    self.delta = 1
                    self.eta = 1
                    self.lane_change_progress = 0
                else:
                    if front_vehicle[0] < s_long:
                        self.current_state = "Braking"
                        self.delta = 0
                        self.eta = 0
                    else:
                        self.current_state = "Following"
                        self.delta = 0
                        self.eta = 1
            else:
                self.delta = 0
                self.eta = 1

        elif self.current_state == "Lane Change":
            self.lane_change_progress += 2  # Simulating progress
            
            if emergency_brake:
                self.current_state = "Emergency Braking"
                self.delta = 0
                self.eta = 0
            elif adjacent_lane_blocked and self.lane_change_progress < 50:
                self.current_state = "Abort Lane Change"
                self.delta = -1
                self.eta = 1
            elif self.lane_change_progress >= 100:
                self.current_state = "Lane Keeping"
                self.delta = 0
                self.eta = 1
                self.lane_change_progress = 0
            else:
                self.delta = 1
                self.eta = 1

        elif self.current_state == "Abort Lane Change":
            self.lane_change_progress -= 2  # Reverting lane change
            
            if self.lane_change_progress <= 0:
                self.current_state = "Lane Keeping"
                self.delta = 0
                self.eta = 1
                self.lane_change_progress = 0
            else:
                self.delta = -1
                self.eta = 1

        elif self.current_state == "Following":
            if emergency_brake:
                self.current_state = "Emergency Braking"
                self.delta = 0
                self.eta = 0
            elif front_vehicle is None or front_vehicle[0] > s_f:
                self.current_state = "Lane Keeping"
                self.delta = 0
                self.eta = 1
            elif front_vehicle[0] < s_long:
                self.current_state = "Braking"
                self.delta = 0
                self.eta = 0
            else:
                self.delta = 0
                self.eta = 1

        elif self.current_state == "Braking":
            if emergency_brake:
                self.current_state = "Emergency Braking"
                self.delta = 0
                self.eta = 0
            elif front_vehicle is None or front_vehicle[0] > s_long:
                self.current_state = "Following"
                self.delta = 0
                self.eta = 1
            else:
                self.delta = 0
                self.eta = 0

        elif self.current_state == "Emergency Braking":
            # Stay in emergency braking until full stop
            if ego_v_x < 0.1:  # Near complete stop
                self.current_state = "Lane Keeping"
                self.delta = 0
                self.eta = 1
            else:
                self.delta = 0
                self.eta = 0
        
        return self.delta, self.eta, self.current_state
