import math

def line_risk(T_list, line):
    risk = 0
    for T in T_list:
        thr = T[0:2]
        dis = dis_p2l(thr, line)
        # not in thr
        if dis[1] > T[2]:
            continue
        v = [line[0][0]-line[1][0], line[0][1]-line[1][1]]
        line_len = math.sqrt((v[0]**2)+(v[1]**2))
        # with a special case that dis[1]=0 => cos = T[2] = radius
        cos = cos2sin(dis[1], r =T[2])
        # 2p in thr
        # one in thr
        post_len = line_len - dis[0]
        disr = dis_p2l(thr, [line[1],line[0]])
        pre_len = line_len - disr[0]
        print('post',post_len)
        print('pre',pre_len)
        print()
        if post_len-pre_len == -line_len:
            risk += T[2] - dis[1]
            continue
        if pre_len >= cos:
            pre_len = cos
        if post_len >= cos:
            post_len = cos
        risk += (post_len+pre_len)

    return risk


def cos2sin(cos,r =5):
    
    return  math.sqrt((r**2-cos**2))

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


T_list = [(5,3,5)]
line=[(0,0),(3,0)]

print(line_risk(T_list, line))
