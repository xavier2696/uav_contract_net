
# coding: utf-8

# In[3]:



'''
Possibility road map(PRM) is used for avoiding forbidden areas in path planning, 
the steps are as follows:
1. sampling(): create sample points
2. generate_roadmap(): create road map base on sample points
3. dijkstra(): find shortest path between two nodes base on roadmap
4. PRM(): create M_list from above functions, which is a standard data structure for uavs.py to call, 
        inputs include: 
        - start: (startx,starty)
        - targets: [(t1_x,t1_y), (t2_x,t2_y)...]
        - forbid: vertex of polygons [[(p1x,p1y), (p2x,p2y), (p3x,p3y)...(p1x,p1y)],[]...]
        - index_of_path: target order of each UAV [[m5,m2,m26],[m38,m23,m8,m3],...]

'''

import random
import math
import matplotlib.pyplot as plt
from matplotlib.path import Path

# constant
map_size = (1500,1500)
N_SAMPLE = 500
max_edge = 50


#Node class for dijkstra search
class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        
    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


# Vector(), vector_product(), is_intersected(), is_collision() used in generating road map
# detect if an edge is intersected with polygons
# reject if yes
def Vector(pointA, pointB):
    return (pointB[0]-pointA[0], pointB[1]-pointA[1])   

def vector_product(vectorA, vectorB):
    return vectorA[0] * vectorB[1] - vectorB[0] * vectorA[1]

def vector_dot(vectorA, vectorB):
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1]

def is_intersected(A, B, C, D):
    ZERO = 1e-9
    AC = Vector(A, C)
    AD = Vector(A, D)
    BC = Vector(B, C)
    BD = Vector(B, D)
    return (vector_product(AC, AD) * vector_product(BC, BD) <= ZERO)         and (vector_product(AC, BC) * vector_product(AD, BD) <= ZERO)

def is_collision(start, goal, forbid):

    for i in range(len(forbid)):
        for j in range(len(forbid[i])-1):
            if is_intersected(start, goal, forbid[i][j], forbid[i][j+1]): 
                return True
    return False

def line_risk(line, T_list):
    risk = 0
    
    for threat in T_list:
        point = threat[0:2]
        r = threat[2]
        start = line[0] 
        end = line[1] 
        dx, dy = end[0]-start[0], end[1]-start[1] # vector(dx,dy)
        
        #distance from starting point/ending point to circle center
        dist_start = ((start[0]-point[0])**2 + (start[1]-point[1])**2)**0.5 
        dist_end = ((end[0]-point[0])**2 + (end[1]-point[1])**2)**0.5
        
        # if line segment is in the circle, return distance of line segment
        if dist_start <= r and dist_end <= r:
            risk += (dx**2+dy**2)**0.5
        
        else:
            # parametric equation 參數式
            # s=start, t=parameter, v=vector, q=circle center
            # |s + t*v - q| = r, square both side
            # t^2*(v*v) + 2t(v*(s−q)) + (s*s + q*q − 2s*q − r^2)=0
            a = vector_dot((dx, dy), (dx, dy))
            b = 2*(vector_dot((dx, dy), (start[0]-point[0],start[1]-point[1])))
            c = vector_dot(start,start) + vector_dot(point,point) - 2*vector_dot(start,point) - r**2
            disc = b**2 - 4*a*c #discriminant
            
            # no intersections whether the line extends => no risk
            if disc <= 0: 
                continue
            else:
                t1 = (-b + disc**0.5) / (2*a)
                t2 = (-b - disc**0.5) / (2*a)
                intersect1 = (start[0]+dx*t1, start[1]+dy*t1)
                intersect2 = (start[0]+dx*t2, start[1]+dy*t2)
                #print("intersections:", intersect1, intersect2)
                #print("t1,t2:", t1, t2)
                
                # no intersections in line segment 
                if not (0 < t1 < 1 or 0 < t2 < 1):
                    continue
                # 2 intersections, risk = |intersection1-intersection2|
                elif (0 < t1 < 1 and 0 < t2 < 1):
                    risk += ((intersect1[0]-intersect2[0])**2 + (intersect1[1]-intersect2[1])**2)**0.5
                # only 1 intersection, risk = |inner_point-intersection|
                else:
                    if dist_start <= r:
                        if 0 <= t1 <= 1:
                            risk += ((intersect1[0]-start[0])**2 + (intersect1[1]-start[1])**2)**0.5
                        else:
                            risk += ((intersect2[0]-start[0])**2 + (intersect2[1]-start[1])**2)**0.5
                    elif dist_end <= r:
                        if 0 <= t1 <= 1:
                            risk += ((intersect1[0]-end[0])**2 + (intersect1[1]-end[1])**2)**0.5
                        else:
                            risk += ((intersect2[0]-end[0])**2 + (intersect2[1]-end[1])**2)**0.5
    return risk

def sampling(start, targets, forbid):

    (maxx,maxy) = map_size
    (minx,miny) = (0,0)
    sampleList = []
    sampleList.append(start)
    sampleList.extend(targets)
    
    while len(sampleList) <= N_SAMPLE + len(targets):
        sample_x = random.randint(minx,maxx)
        sample_y = random.randint(miny,maxy)
        
        #detect if random sample point is in polygons, reject if yes
        append = True
        for i in range (len(forbid)):
            ov = Path(forbid[i])
            if ov.contains_point((sample_x,sample_y)):
                append = False
                break
        if (append == True):
            sampleList.append((sample_x,sample_y))
            
    # plot samples for debugging
    '''
    samples = list(zip(*sampleList)) 
    sample_x, sample_y = samples[0], samples[1]
    plt.plot( sample_x, sample_y, ".r")
    '''
    return sampleList

def generate_roadmap(sampleList, forbid):
    road_map = []
    nsample = len(sampleList)
    for (i, start) in zip(range(nsample), sampleList):
        edge_id = []
        for (j, goal) in zip(range(nsample), sampleList):
            if(i is not j):
                if not is_collision(start, goal, forbid):
                    edge_id.append(j)
                if len(edge_id) >= max_edge:
                    break       
        road_map.append(edge_id)
        
    # plot road map
    '''
    samples = list(zip(*sampleList)) 
    sample_x, sample_y = samples[0], samples[1]
    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]
            plt.plot([sample_x[i], sample_x[ind]], [sample_y[i], sample_y[ind]], "-k")
    '''
    return road_map
    

def dijkstra(target_start_index, target_end_index, road_map, sampleList, threats):
    
    target_start_index += 1
    target_end_index += 1
    sx = sampleList[target_start_index][0]
    sy = sampleList[target_start_index][1]
    gx = sampleList[target_end_index][0]
    gy = sampleList[target_end_index][1]

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)
    openset, closedset = dict(), dict()
    openset[target_start_index] = nstart

    while True:
        if not openset:
            print("from target",target_start_index-1,"to",target_end_index-1,"Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        if c_id == target_end_index:
            #print("goal found!")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sampleList[n_id][0] - current.x
            dy = sampleList[n_id][1] - current.y
            d = math.sqrt(dx**2 + dy**2)
            risk = line_risk([sampleList[n_id],[current.x, current.y]], threats)
            
            #print("d,risk:", d, risk)
            #node = Node(sampleList[n_id][0], sampleList[n_id][1], current.cost + d, c_id)
            weighting_cost = d + risk
            node = Node(sampleList[n_id][0], sampleList[n_id][1], current.cost + weighting_cost, c_id)
            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    path = [(ngoal.x, ngoal.y)]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        path.append((n.x, n.y))
        pind = n.pind
    path.reverse()
    return path

def PRM(start, targets, forbid, threats, index_of_path):

    start = [int(start[0]), int(start[1])]
    sampleList = sampling(start, targets, forbid)
    road_map = generate_roadmap(sampleList, forbid)
    
    # route planning by using dijkstra 
    # initially 3 routes, UAV0-4, UAV5-9, UAV10-14
    # save routes in M_list of i UAVs
    M_list = []
    
    for i in range (len(index_of_path)): # for each UAV
        
        M_list.append([])
        previous_index_of_target = -1 # -1 for starting point
        j = 0
        for j in range(len(index_of_path[i])):

            M_list[i].append([])
            index_of_target = index_of_path[i][j]
            M_list[i][j].append(index_of_target) # append target index in M_list[i][j], ex:[[32, [(.., ..), (.., ..)]]...
            
            path = dijkstra(previous_index_of_target, index_of_target, road_map, sampleList, threats)    
            M_list[i][j].append(path) # append path in M_list[i][j], ex:[[..,[(750, 750), (1389, 248)]]...
            previous_index_of_target = index_of_target
        
        # return back the starting point
        M_list[i].append([])
        path = dijkstra(previous_index_of_target, -1, road_map, sampleList, threats)
        M_list[i][j+1].append(int(-1))
        M_list[i][j+1].append(path)
    return M_list

def drawMap(start, targets, forbid, threats, M_list):
    
    #plt.figure(figsize=(6,6))
    plt.grid(True)
    plt.plot(start[0], start[1], "^r")
    
    # draw obstacles
    for i in range (len(forbid)):
        ov_x = list(zip(*forbid[i]))[0]
        ov_y = list(zip(*forbid[i]))[1]
        plt.plot(ov_x,ov_y, "-k") 
        
    # draw threats
    for i in range (len(threats)):
        plt.plot(threats[i][0], threats[i][1], "^r")
        circle = plt.Circle((threats[i][0], threats[i][1]), threats[i][2], color='c', fill=False)
        ax = plt.gca()
        ax.add_artist(circle)
    
    # draw targets
    target = list(zip(*targets)) 
    target_x, target_y = target[0], target[1]
    plt.plot(target_x, target_y, ".b")
    
    # draw final routes
    for i in range (len(M_list)):
        targets_route = list(zip(*M_list[i]))[1]
        for route in targets_route:
            #print("route =",route)
            rx = list(zip(*route))[0]
            ry = list(zip(*route))[1]
            plt.plot(rx,ry,"-g")

            
def threaten_value(start, end, threats):
    value = 0

    return
    
############### testing ###############
############### testing ###############
def main():
    
    #testing data as follows:
    start = (750,750)
    forbid = []
    threats = [(1467, 632, 142), (1495, 125, 111), (1381, 1078, 150), (1362, 303, 127), (211, 45, 104), 
               (806, 1268, 122), (810, 1335, 147), (1369, 622, 127), (1276, 1116, 149), (32, 503, 121),
               (1132, 1335, 128), (1261, 319, 137), (1322, 1356, 111), (109, 1061, 100), (71, 766, 110),
               (23, 444, 117), (1312, 703, 137), (423, 1480, 124), (1249, 937, 104), (343, 1280, 145)] 
    
    targets: [(752, 206), (894, 49), (25, 157), (80, 785), (449, 1395), (563, 1256), (1253, 301), (1251, 927), (996, 1268), (1183, 1097), (222, 549), (236, 1280), (1322, 804), (202, 1000), (1437, 935), (276, 15), (384, 1236), (1211, 40), (781, 1378), (344, 152), (1485, 167), (1271, 379), (167, 591), (309, 384), (89, 1327), (74, 378), (261, 387), (1401, 1269), (992, 1340), (292, 152), (968, 89), (1427, 183), (17, 274), (147, 159), (76, 926), (1197, 370), (1353, 142), (1368, 110), (1213, 981), (426, 1165)] 
    # targets order of each UAV 0-14 
    # initially divided into 3 groups
    
    index_of_path =  [[32, 4, 6, 28, 34]]     
    
    
    #targets = [(1389,248),(1200,800),(1400,1500),(6,1276)]
    #index_of_path = [[0,1,2,3]]
    M_list = PRM(start, targets, forbid, threats, index_of_path)
    drawMap(start, targets, forbid, threats, M_list)
    print(M_list)


if __name__ == '__main__':
    main()

