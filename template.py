import mapmath
import threading
import time
from uav_agent import UAVAgent
import copy


# A basic Mission_list structure is [ (targer.x, targer.y), [(p1x,p1y),(p2x,p2y)...]]
# each UAV contain a few Mission_list
# and M_list store all of 15 UAVs' Mission_lists
#M_list = None
T_list = None # include original T_note and T_old
# Start with the same position as it in T_list and update when detect threat move
T_note = None
# The not-moving threat
T_old = None
# every Forbid[i] is the vertex set of the Forbid area
Forbid = None
# It'a
TarNew = None
Risk = [0 for i in range(15)]

#list of uav agents
uav_list = []


def update(pos, info, M_list , T_list, T_note, T_old, Forbid, TarNew):

    # update the state of all uav agents
    if len(uav_list) == 0:
        #example of a mision to make uav wait for 4 seconds
        #new_array.append([-2, [[600, 600], [600, 615], [600, 600], [600, 615], [600, 600]]])
        for i in range(0, len(M_list)):
            uav_list.append(UAVAgent(i, copy.deepcopy(M_list[i]), Forbid, T_list))
        for uav in uav_list:
            if uav.is_leader == False:
                if uav.uav_id < 5:
                    uav.set_leader(uav_list[0])
                elif uav.uav_id < 10:
                    uav.set_leader(uav_list[5])
                elif uav.uav_id < 15:
                    uav.set_leader(uav_list[10])
    else:
        for i in range(0, len(uav_list)):
            uav_list[i].update(copy.deepcopy(M_list[i]), i, T_list)

    # info is a list of INFOM type meaning that multiple cases could be happened in a second
    #INFOM type:
    # [0] UAV fail,    [1] index of fail UAV
    # [0] new target, [1] target coord (x, y)  , [2] detected by UAV i (index of UAV)
    # [0] detect threat, [1] threat coord (x, y) , [2] detected by UAV i (index of UAV)
    #allocate different case
    for i in info:
        if(i[0] == 'new target'):
            print('New Target')
            print(TarNew)
            print(i[2])
            contractnet_negotiation('new target', i, M_list, TarNew)
        elif(i[0] == 'fail'):
            print('UAV Fail')
            print(i[1])
            contractnet_negotiation('fail', i, M_list, TarNew)
            TarNew.clear()
        elif(i[0] == 'detect threat'):
            print('New Threat')
            print(i[1])
            path_replanning(pos, i,M_list, T_list)


def contractnet_negotiation(case,info,M_list, TarNew):
    # example1: If UAV i fail turn its mission to the leader
    if(case == 'fail'):
        if uav_list[info[1]].is_leader: # handle the case where the uav leader fails
            uav_list[info[1]+1].is_leader = True
            for i in range(info[1]+2, len(uav_list)):
                if i >= len(uav_list) or uav_list[i].is_leader:
                    break
                uav_list[i].set_leader(uav_list[info[1]+1])
        uav_list.pop(info[1])
#        leader = int(info[1]/5)*5
#        # make leader took over the mission when original mission had all done
#        mission_of_leader = [M_list[leader][i][0] for i in range(len(M_list[leader]))]
#        for T in TarNew:
#            if(T[0] not in mission_of_leader):
#                M_list[leader].append(T) # mission woulbe be put into TarNew when UAV fail
    # example2: new target
    # allocate new mission to UAV 0
    elif(case == 'new target'):
        if len(TarNew) == 0:
            return
        # if TarNew[0][0] == 38:
        #     TarNew[0][0] = -4
        #     M_list[6].insert(len(M_list[6]) - 1, TarNew[0])
        #     TarNew.pop()
        #     # update missions
        #     for i in range(0, len(uav_list)):
        #         uav_list[i].update(copy.deepcopy(M_list[i]), i)
        #     print(M_list[6])
        #     return
        new_mission = TarNew[0]
        #get bids from leaders to know which group will handle the new target
        leader_bids = []
        for uav in uav_list:
            if uav.is_leader:
                leader_bids.append(uav.get_leader_bid(new_mission))
        winner = leader_bids[0]
        for bid in leader_bids:
            print(bid[1])
            if bid[1] < winner[1]:
                winner = bid
        print("leader uav", winner[0].uav_id, "will take the mission")
        #initiate contract net protocol between the uavs under the command of the leader to decide who will take the mission
        uavs_in_group = 1
        for i in range(winner[0].current_index + 1, len(uav_list)):
            if i >= len(uav_list) or uav_list[i].is_leader:
                break
            uavs_in_group += 1
        bids = [(-1, -1)] * len(uav_list)
        for i in range(winner[0].current_index, winner[0].current_index + uavs_in_group):
            x = threading.Thread(target=uav_list[i].post_bid, args=(new_mission, bids))
            #threads.append(x)
            x.start()
        time.sleep(0.900)

        for bid in bids:
            print(bid[1])
        index_min = 0
        all_negative = True
        for i in range(0, len(bids)):
            if bids[i][1] < 0:
                continue
            if all_negative:
                index_min = i
            all_negative = False
            if bids[i][1] < bids[index_min][1]:
                index_min = i
        if all_negative: # TODO deal with the case that no bids have been proposed
            return
        winning_bid = bids[index_min]
        # winning_bid[0] contains the list of (uav, new_mission_set)
        print("mission", new_mission, "accepted by uav", winning_bid[0][0][0].uav_id, "in index", winning_bid[0][0][0].current_index)
        for mission_set in winning_bid[0]:
            M_list[mission_set[0].current_index] = mission_set[1]
        # print(index_min)
        # print(M_list[index_min])
        # print(uav_list[index_min].current_missions)
        # M_list[index_min] = uav_list[index_min].current_missions
        # append last end point
        #M_list[info[2]].insert(-1, new_mission)# new target would also be mlist_struct with [[xnewtar, xnewtar], [[[xnewtar, xnewtar]]],...]
        #M_list[info[2]].insert(0, new_mission)
        TarNew.pop()
        #update missions
        for i in range(0, len(uav_list)):
            uav_list[i].update(copy.deepcopy(M_list[i]), i)



def path_replanning(pos ,info,M_list, T_list):
    return
#    thr = info[1]
#    # the UAV(info[2]) now([0]) path[1]
#    nowpath = M_list[info[2]][0][1]
#    # It's based on calculation time
#    # Since we allow at most 1 seconde calculation time in every step of update(relate to one second flying time)
#    # This is a templete to check wheather we are still in the same situation when calculation has done.
#    ifmuta = M_list[info[2]][0][1][0].copy() # first point of nowpath , or can check M_list[info[2]][0][0], if we still in the same mission
#    # example 3 new threat
#    # make a function call to the built function to test if still can afford the risk
#    plus_risk = mapmath.path_risk(T_list, [pos[info[2]],  nowpath[0]])
#    # if the path would not pass the threat range
#    if (plus_risk == 0):
#        return
#    # else if risk still in tollerance range
#    elif(Risk[info[2]] + plus_risk < 60): # 60 is the half of the tollerance risk
#        Risk[info[2]]  += plus_risk
#        return
#    
#    # or to insert a new point to let UAV move around the threat
#    
#    else:
#        #orth = (nowpath[0][1] - pos[info[2]][1] , nowpath[0][0] - pos[info[2]][0])
#        #move_around = [thr[0] + thr[2]*orth[0], thr[1] + thr[2]*orth[1]]
#        move_around = [thr[1], thr[0]]
#        # to check if M_list has been revised
#        if ifmuta == M_list[info[2]][0][1][0]:
#            # change all in the team'
#            nowpath.insert(0, pos[info[2]]) # stop at the current pos
#            nowpath.insert(1, move_around) # move away from the  threat
#            return




