import mapmath
import math
import copy
import time
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
        self.new_missions = None
        self.new_utility = None

    def update(self, current_missions, threats):
        self.current_missions = current_missions
        self.threats = threats
        self.current_utility = self.calculate_utility(self.current_missions)

    def deactivate(self):
        self.active = False

    def post_bid(self, new_target, bids):
        if self.uav_id > len(bids) - 1 or self.active == False:
            return
        self.new_missions = copy.deepcopy(self.current_missions)
        closest_mission_index = 0
        for i in range(0, len(self.new_missions)):
            closest_path_index = 0
            for j in range(0, len(self.new_missions[i][1])):
                if self.get_distance(self.new_missions[i][1][j], new_target[1][0]) \
                        < self.get_distance(self.new_missions[closest_mission_index][1][closest_path_index], new_target[1][0]):
                    closest_mission_index = i
                    closest_path_index = j
        self.new_missions[closest_mission_index][1].insert(closest_path_index, new_target[1][0]) # assuming no forbidden areas in between

        self.new_utility = self.calculate_utility(self.new_missions)
        bids[self.uav_id] = self.current_utility - self.new_utility

    def accept_mission(self, new_target):
        if len(self.new_missions) > 0: # TODO: add check if new_missions contains the new_target mission
            self.current_missions = self.new_missions
            self.new_missions = None
            self.current_utility = self.new_utility
            self.new_utility = None
        print("mission", new_target, "accepted by uav", self.uav_id)

    def cancel_new_mission(self, new_target):
        if self.new_missions: # TODO: add check if new_missions contains the new_target mission
            self.new_missions = None
            self.new_utility = None

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
            t_distance += self.get_distance(mission_path[i], mission_path[i+1])
        return t_distance

    def get_distance(self, p1, p2):
        return math.sqrt(sum([(a - b) ** 2 for a, b in zip(p1, p2)]))
