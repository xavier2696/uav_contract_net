import mapmath
import math
k1 = 0.5
k2 = 0.5

class UAVAgent:

    def __init__(self, uav_id, current_missions, forbidden_areas, threats):
        self.uav_id = uav_id
        self.current_missions = current_missions
        self.forbidden_areas = forbidden_areas
        self.threats = threats
        self.active = True
        self.current_utility = self.calculate_utility(self.current_missions)

    def update(self, current_missions, threats):
        self.current_missions = current_missions
        self.threats = threats
        self.current_utility = self.calculate_utility(self.current_missions)

    def deactivate(self):
        self.active = False

    def evaluate_bid(self, new_target):
        bid = 1
        return bid

    def accept_mission(self, new_target):
        print("mission", new_target, "accepted by uav", self.uav_id)

    def calculate_utility(self, mission_list):
        utility = 0
        for mission in mission_list:
            risk = self.calculate_mission_risk(mission)
            fuel = self.calculate_mission_fuel(mission)
            utility += -k1*risk-k1*fuel
        return utility

    def calculate_mission_risk(self, mission):
        mission_path = mission[1]#.copy().append(mission[0])
        return mapmath.path_risk(self.threats, mission_path)

    def calculate_mission_fuel(self, mission):
        mission_path = mission[1]#.copy().append(mission[0])
        t_distance = 0
        for i in range(len(mission_path) - 1):
            t_distance += math.sqrt(sum([(a - b) ** 2 for a, b in zip(mission_path[i], mission_path[i+1])]))
        return t_distance
