import mapmath
import math
import copy
import time
k1 = 0.3
k2 = 0.7
max_risk_total = 900

class UAVAgent:

    def __init__(self, uav_id, current_missions, forbidden_areas, threats):
        self.uav_id = uav_id
        self.current_missions = current_missions
        self.all_missions = current_missions
        self.forbidden_areas = forbidden_areas
        self.threats = threats
        self.current_utility = calculate_utility(self.all_missions, self.threats)
        self.leader = None
        self.is_leader = (self.uav_id == 0 or self.uav_id == 5 or self.uav_id == 10)
        self.current_index = self.uav_id
        self.print_info()

    def update(self, current_missions, current_index, threats=None):
        old_missions = self.current_missions
        self.current_missions = current_missions
        if len(old_missions) != len(self.current_missions):
            self.all_missions = get_new_all_missions(self.all_missions, self.current_missions)

        if threats:
            self.threats = threats
        self.current_utility = calculate_utility(self.all_missions, self.threats)
        self.current_index = current_index

    def print_info(self):
        print("UAV", self.uav_id, "agent, utility:", self.current_utility, " risk:",
              calculate_total_risk(self.all_missions, self.threats), " number missions:",
              len(self.all_missions))

    def set_leader(self, leader):
        self.leader = leader

    def get_leader_bid(self, new_target):
        new_missions = copy.deepcopy(self.current_missions)
        closest_mission_index = 0
        for i in range(0, len(new_missions) - 1):
            closest_path_index = 0
            for j in range(0, len(new_missions[i][1])-1):
                if get_distance(new_missions[i][1][j], new_target[1][0]) \
                        < get_distance(new_missions[closest_mission_index][1][closest_path_index], new_target[1][0]):
                    closest_mission_index = i
                    closest_path_index = j
        new_missions[closest_mission_index][1].insert(closest_path_index, new_target[1][0]) # assuming no forbidden areas in between

        new_utility = calculate_utility(new_missions, self.threats)
        return (self, self.current_utility - new_utility)

    def post_bid(self, new_target, bids):
        if self.current_index > len(bids) - 1:
            return
        if self.is_leader:
            bids[self.current_index] = ([(self, self.current_missions)], -2, False)
            # do something
        else:
            # get the closest path to the new target
            new_mission_set = copy.deepcopy(self.current_missions)
            closest_mission_index = 1
            for i in range(1, len(new_mission_set) - 1):
                closest_path_index = 0
                #for j in range(0, len(new_mission_set[i][1])):
                j = len(new_mission_set[i][1]) - 1
                if get_distance(new_mission_set[i][1][j], new_target[1][0]) \
                        < get_distance(new_mission_set[closest_mission_index][1][closest_path_index], new_target[1][0]):
                    closest_mission_index = i
            # insert the new mission
            last_coordinate_before = new_mission_set[closest_mission_index-1][1][len(new_mission_set[closest_mission_index-1][1]) - 1]
            new_target[1].insert(0, last_coordinate_before)
            mission_after = new_mission_set[closest_mission_index]
            new_mission_coordinate = new_target[1][len(new_target[1]) - 1]
            mission_after[1][0] = new_mission_coordinate
            new_mission_set.insert(closest_mission_index, new_target) # assuming no forbidden areas in between
            new_missions = [(self, new_mission_set)]
            # check if after inserting the new mission it is within the leader radius for the rest of the time

                # if it is return the bid (change in utility cost of the new missions compared to the old missions
                # if it is not within the leader radius make the leader and all the other uavs move to a position where it is
                # within the radius and wait(outside a threat area preferably), then return the bid
                # (change of utility cost of all uavs in the group)

            new_total_utility = 0
            over_risk_threshold = False
            for mission_set in new_missions:
                all_missions = get_new_all_missions(mission_set[0].all_missions, mission_set[1])
                new_total_utility = new_total_utility + calculate_utility(all_missions, mission_set[0].threats)
                total_risk = calculate_total_risk(all_missions, mission_set[0].threats)
                if total_risk > max_risk_total:
                    over_risk_threshold = True
            bids[self.current_index] = (new_missions, -1*new_total_utility, over_risk_threshold)

#helper methods
def calculate_utility(mission_list, threats):
    utility = 0
    for mission in mission_list:
        risk = calculate_mission_risk(mission, threats)
        fuel = calculate_mission_fuel(mission)
        utility += -k1*risk-k2*fuel
    return utility

def calculate_total_risk(mission_list, threats):
    risk = 0
    for mission in mission_list:
        risk += calculate_mission_risk(mission, threats)
    return risk

def calculate_mission_risk(mission, threats):
    mission_path = mission[1]#.copy().append(mission[0])
    return mapmath.path_risk(threats, mission_path)

def calculate_mission_fuel(mission):
    mission_path = mission[1]#.copy().append(mission[0])
    t_distance = 0
    for i in range(len(mission_path) - 1):
        t_distance += get_distance(mission_path[i], mission_path[i+1])
    return t_distance

def get_distance(p1, p2):
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(p1, p2)]))

def get_new_all_missions(all_missions, current_missions):
    all_missions = copy.deepcopy(all_missions)
    all_missions_indexes = [mission[0] for mission in all_missions]
    if len(current_missions) > 0 and current_missions[0][0] in all_missions_indexes:
        index_current = all_missions_indexes.index(current_missions[0][0])
        all_partial = all_missions[:index_current + 1]
        all_missions = all_partial + current_missions[1:len(current_missions)]
    return all_missions

