import math
import os

m = None
M_list = []
T_list = []
Forbid = []
Tar = []
fail = {}
thr_move = {}

write_flag = True


    

def path_risk(T_list, path):
    risk = 0
    for i in range(len(path)-1):
        line = [path[i],path[i+1]]
        risk += line_risk(T_list, line)
    return risk

def vector_dot(vectorA, vectorB):
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1]

def line_risk(T_list, line):
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






def read():
    if os.path.isfile('mapdata.txt') is False:
        return False
    global write_flag
    write_flag = False
    member = pre_write()
    with open('mapdata.txt','r') as f:
        words = f.readlines()
        for word in words:
            # split line to key(name of variable) and list data
            word = word.strip('\n').replace(' ','').split(':')
            if word == ['']:
                continue
            key = word[0]
            data = word[1].rstrip(']').lstrip('[') #take off the list part of data
            # list of coord
            if 'M_list'in key:
                member[key] = []
                for d in data.split(']]'):
                    li = []
                    d = d.lstrip(',[').replace('[','')
                    point = d.replace('(','').rstrip(')').split('),')
                    for x in point:
                        x = x.split(',')
                        li.append(list(map(int, x)))
                    member[key].append(li)
                continue
            elif 'Forbid'in key :
                member[key] = []
            elif 'fail' in key:
                if 'index' in key:
                    fail['index'] = int(data)
                elif 'time' in key:
                    fail['time'] = int(data)
                continue
            elif 'move' in key:
                if 'to' in key:
                    thr_move['to'] = []
                    key = 'thr_move_to'
                    member[key] = thr_move['to']
                elif 'count' in key:
                    thr_move['count'] = []
                    key = 'thr_move_count'
                    member[key] = thr_move['count']
            point = data.replace('(','').rstrip(')').split('),')

            for x in point:
                x = x.split(',')
                member[key].append(list(map(int, x)))
    pro_M_list_process(member)
    return True

def pro_M_list_process(member):
    for k, v in member.items():
        if 'M_list' in k:
            li = []
            for every_v in v :
                target = every_v[0][0]
                path = [[every_v[0][1],every_v[0][2]]]
                path += every_v[1:]
                li.append([target, path])
            M_list.append(li)
        elif 'Forbid' in k:
            Forbid.append(v)


def pre_write():
    point_member = ['T_list', 'Tar']
    list_member = ['M_list', 'Forbid']
    member = {}
    for w in point_member:
        member[w] = globals()[w]
    # save
    for l in list_member:
        li = globals()[l]
        for i in range(len(li)):
            member['%s[%s]' %(l,i)] = li[i]
    return member

def write():
    if write_flag is False:
        return
    member = pre_write()
    # to write T_list as tuple
    for i in range(len(T_list)):
        T_list[i] = tuple(T_list[i])
    with open('mapdata.txt', 'w') as f:
        for k, v in member.items():
            f.write(k+': ')
            f.write(str(v)+' ')
            f.write('\n\n')
        f.write('fail_index: %s \n'%str(fail['index']))
        f.write('fail_time: %s \n'%str(fail['time']))
        tup_to = []
        tup_count = []
        for i in range(len(thr_move['to'])):
            tup_to.append(tuple(thr_move['to'][i]))
            tup_count.append(tuple(thr_move['count'][i]))
        f.write('thr_move_to: %s \n'%str(tup_to))
        f.write('thr_move_count: %s \n'%str(tup_count))

#因為函數發散,取估計值
def est_threaten_cost(cos, interval, prec = 1000):
    s = (interval[1]-interval[0])/prec
    sum = 0
    for i in range(prec+1):
        sum += (((interval[0]+i*s)**2+cos**2)**1/2)**-4/(prec+1)
    return sum*(interval[1]-interval[0])


def cos2sin(cos,r =5):
    return  math.sqrt((r**2-cos**2))

def distance(p1, p2):
    return math.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))

def normalize(vec):
    
    total = 0
    nv = []
    for v in vec:
        total += v**2
    norm = math.sqrt(total)
    for v in vec:
        nv.append(v/norm)
    
    return nv


def backward_rolling(arg):
    pattern = arg['distance'][2]
    #割線長
    secant = 2*cos2sin(arg['distance'][0], arg['r_radus'])
    # 為了避免字串比較,pattern 使用數字代碼
    # 0為 其他,1 為 on path, 2為by path
    if(pattern == 1):
        rolling =  arg['distance'][0]- arg['r_radus']
        #扣掉原路
        circle_time = 2*math.pi*arg['r_radus'] - secant
    elif(pattern == 2):
        rolling = cos2sin(arg['distance'][1], arg['r_radus'])
        circle_time = math.acos(arg['distance'][1]/arg['r_radus'])*arg['r_radus']*2*math.pi - secant
    else:
        rolling = arg['distance'][0]
        circle_time = 0
    
    return [rolling, circle_time]



# 回傳三個值 到最近點該點距離，2為離最近點距離, 3 為類型
def dis_p2l(point, line):
    # 定義 兩條向量
    qp = [line[0][0]-point[0], line[0][1]-point[1]]
    v = [line[0][0]-line[1][0], line[0][1]-line[1][1]]
    # 沒移動
    if(v == [0, 0]):
        return [0, math.sqrt((qp[0]**2)+(qp[1]**2))]
    # 內積外積 及 path 長
    dot = qp[0]*v[0]+qp[1]*v[1]
    cross = qp[0]*v[1]-qp[1]*v[0]
    v_value = math.sqrt((v[0]**2)+(v[1]**2))
    # proj
    proj = dot/v_value
    # orth
    orth = abs(cross/v_value)
    # 在起點前
    if(proj< 0):
        return [0 ,math.sqrt((qp[0]**2)+(qp[1]**2)),0]
    #在路線上
    elif((cross == 0 and proj <= v_value)):
        return [math.sqrt((qp[0]**2)+(qp[1]**2)), 0, 1]
    # 在終點後
    elif(proj > v_value):
        return [v_value, math.sqrt(((point[0]-line[1][0])**2)+((point[1]-line[1][1])**2)),0]
    #在線段中
    else:
        return [proj, orth, 2]
