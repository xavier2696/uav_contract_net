

'''
主要包含兩個function: 
1. 
road_map, sample_x, sample_y = PRM_planning(sx, sy, target, ox, oy, safe_distance)
#取樣做出所有避開障礙物之road map
    
input------------------------------
    sx: int or float, UAV起點x座標
    sy: int or float, UAV起點y座標
    target: [], 目標點集合, e.g: [(27, 4), (24, 36), (39, 63), (67, 15), (54, 74)]
    ox: [], 障礙物x點集合
    oy: [], 障礙物y點集合
    safe_distance: float, 路徑與障礙物距離最小距離，目前設為1
output-----------------------------
    road_map: [], 避開障礙物之road map
    sample_x: [], 取樣點x集合
    sample_y: [], 取樣點y集合
    
    
2.
    rx, ry = dijkstra(target_start_index, target_end_index, road_map, sample_x, sample_y)
    #取樣點與road map完成後進行路徑規劃
    
input------------------------------
    target_start_index: int, 路徑規劃之起點 (起點為0, target[0]為1, target[1]為2...)
    target_end_index: int, 路徑規劃之終點 (起點為0, target[0]為1, target[1]為2...)
    road_map: []
    sample_x: []
    sample_y: []
output-----------------------------   
    rx: [], 路徑x點集合
    ry: [], 路徑y點集合
'''

import random
import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
import map

# parameter
N_SAMPLE = 200  # number of sample_points
N_KNN = 50  # number of edge from one sampled point
safe_distance = 1
map_size = map.map_size
MAX_EDGE_LEN = map_size[0]*2  # Maximum edge length

class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        #store kd-tree
        self.tree = scipy.spatial.cKDTree(data)
        
    def search(self, inp, k=1):
        """
        Search NN
        inp: input data, single frame or multi frame
        """
        
        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []
            
            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """
        index = self.tree.query_ball_point(inp, r)
        return index

####################################################

def PRM_planning(sx, sy, target, ox, oy, safe_distance):
    
    sample_x, sample_y = sampling(sx, sy, target, ox, oy, safe_distance)
    #plt.plot(sample_x, sample_y, ".b")
    road_map = generate_roadmap(sample_x, sample_y, ox, oy, safe_distance)
    #print(road_map)
    
    return road_map, sample_x, sample_y


def dijkstra(target_start_index, target_end_index, road_map, sample_x, sample_y):

    sx = sample_x[target_start_index]
    sy = sample_y[target_start_index]
    gx = sample_x[target_end_index]
    gy = sample_y[target_end_index]

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[target_start_index] = nstart

    while True:
        if not openset:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        '''
        # show animation
        if len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)
        '''
        if c_id == target_end_index:
            #print("goal is found!")
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
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

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
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry

def sampling(sx, sy, target, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T) # np.vstack format -> e.g. [[ 0.0.] [ 1.0.] [ 2.0.]]

    #range of sample nodes
    (maxx,maxy) = map_size
    minx,miny = 0,0

    sample_x, sample_y = [], []
    
    sample_x.append(sx)
    sample_y.append(sy)
    for i in range(0,len(target)):
        sample_x.append(target[i][0])
        sample_y.append(target[i][1])

    while len(sample_x) <= N_SAMPLE: #if current samples is less than expect numbers of sample
        
        #create random samples
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)

        index, dist = obkdtree.search(np.array([tx, ty]).reshape(2, 1))
        
        if dist[0] >= rr:    # distance of sample to obstacle must larger than safe distance
            sample_x.append(tx)
            sample_y.append(ty)
    
    return sample_x, sample_y


def generate_roadmap(sample_x, sample_y, ox, oy, rr):
    """
    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: save distance
    obkdtree: KDTree object of obstacles
    
    return a 2-dimension list, e.g.,
    [[1,2,3]
     [0,2]
     [0,1,3]
     [0,2,4]
     [3]]
    (sample_x[0],sample_y[0]) is able to reach (x[1],y[1]), (x[2],y[2]), (x[3],y[3]) and so on.
    
    """
    
    obkdtree = KDTree(np.vstack((ox, oy)).T)
    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):
        index, dists = skdtree.search(np.array([ix, iy]).reshape(2, 1), k=nsample)
        inds = index[0]
        edge_id = []

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])
            
            if len(edge_id) >= N_KNN:
                break
                
        road_map.append(edge_id)

    #plot_road_map(road_map, sample_x, sample_y)

    return road_map

def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = round(d / D)

    for i in range(nstep):
        idxs, dist = okdtree.search(np.array([x, y]).reshape(2, 1))
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.array([gx, gy]).reshape(2, 1))
    if dist[0] <= rr:
        return True  # collision

    return False  # OK

def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")

