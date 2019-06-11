import mapmath
import math
import copy
import time
k1 = 0.3
k2 = 0.7

class UAVAgent:

    def __init__(self, uav_id, current_missions, forbidden_areas, threats):
        self.uav_id = uav_id
        self.current_missions = current_missions
        self.forbidden_areas = forbidden_areas
        self.threats = threats
        self.current_utility = self.calculate_utility(self.current_missions)
        self.new_mission = None
        self.new_utility = None
        self.leader = None
        self.is_leader = (self.uav_id == 0 or self.uav_id == 5 or self.uav_id == 10)
        self.current_index = self.uav_id

    def update(self, current_missions, current_index, threats=None):
        self.current_missions = current_missions
        if threats:
            self.threats = threats
        self.current_utility = self.calculate_utility(self.current_missions)
        self.current_index = current_index

    def set_leader(self, leader):
        self.leader = leader

    def get_leader_bid(self, new_target):
        new_missions = copy.deepcopy(self.current_missions)
        closest_mission_index = 0
        for i in range(0, len(new_missions) - 1):
            closest_path_index = 0
            for j in range(0, len(new_missions[i][1])-1):
                if self.get_distance(new_missions[i][1][j], new_target[1][0]) \
                        < self.get_distance(new_missions[closest_mission_index][1][closest_path_index], new_target[1][0]):
                    closest_mission_index = i
                    closest_path_index = j
        new_missions[closest_mission_index][1].insert(closest_path_index, new_target[1][0]) # assuming no forbidden areas in between

        new_utility = self.calculate_utility(new_missions)
        return (self, self.current_utility - new_utility)

    def post_bid(self, new_target, bids):
        if self.current_index > len(bids) - 1:
            return
        if self.is_leader:
            bids[self.current_index] = ([(self, self.current_missions)], -2)
            # do something
        else:
            # get the closest path to the new target
            new_mission_set = copy.deepcopy(self.current_missions)
            closest_mission_index = 0
            for i in range(0, len(new_mission_set) - 1):
                closest_path_index = 0
                #for j in range(0, len(new_mission_set[i][1])):
                j = len(new_mission_set[i][1]) - 1
                if self.get_distance(new_mission_set[i][1][j], new_target[1][0]) \
                        < self.get_distance(new_mission_set[closest_mission_index][1][closest_path_index], new_target[1][0]):
                    closest_mission_index = i
            # insert the new mission
            new_mission_set.insert(closest_mission_index, new_target) # assuming no forbidden areas in between
            new_missions = [(self, new_mission_set)]
            # check if after inserting the new mission it is within the leader radius for the rest of the time

                # if it is return the bid (change in utility cost of the new missions compared to the old missions
                # if it is not within the leader radius make the leader and all the other uavs move to a position where it is
                # within the radius and wait(outside a threat area preferably), then return the bid
                # (change of utility cost of all uavs in the group)

            new_total_utility = 0
            for mission_set in new_missions:
                new_utility = self.calculate_utility(mission_set[1])
                new_total_utility += mission_set[0].current_utility - new_utility
            bids[self.current_index] = (new_missions, -1*new_utility)

    def calculate_utility(self, mission_list):
        utility = 0
        for mission in mission_list:
            risk = self.calculate_mission_risk(mission)
            fuel = self.calculate_mission_fuel(mission)
            utility += -k1*risk-k2*fuel
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

