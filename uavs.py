import random
import math
import map
import copy
from mapmath import*
import visualization
import center_org
import prm_planning
import template
import threading, time

class shared_Obj():
    
    shared = {}






class DoWork(threading.Thread):
    
    pos = []
    info = []
    Sobj = {'Mlist':[] , 'T_list':[], 'T_note':[], 'T_old':[], 'Forbid':[], 'TarNew':[]}
    
    def __init__(self, shared, *args, **kwargs):
        super(DoWork,self).__init__(*args, **kwargs)
        self.Sobj = shared
        if(len(kwargs['args']) != 0):
            self.pos = kwargs['args'][0]
            self.info = kwargs['args'][1]
    
    
    def run(self):
        shared = self.Sobj.shared
        template.update(pos = self.pos, info = self.info, M_list = shared['M_list'],T_list = shared['T_list'], T_note = shared['T_note'], T_old = shared['T_old'], Forbid = shared['Forbid'], TarNew = shared['TarNew'])


class uav():
    
    
    def __init__(self, replan = False,target_num = (35, 5) ,threaten_num = (17,3)):
        self.m = map.map_gen()
        self.m.target_num = target_num
        self.m.threaten_num = threaten_num
        self.m.gen()
        self.TarNew = []
        self.T_note = []
        # record the time when UAV out of 500m constrain
        self.out_range = {'open':[],'record':[]}
        self.hid_visual = True
        # not load the pre planning
        if (self.m.read is False or replan is True):
            paths = self.mission_allocation()
            self.m.path_planning(paths)
        self.m.set_data()
        self.num = len(self.m.paths)
        
    def run(self, visual = True):
        self.visual = visual
        if(visual):
            self.v = visualization.show_visual(self.m, self.mov)
            self.v.show()
        else:
            for i in self.mov():
                a = 1
    
    def set_up_templete(self):
        #set_up_M_list()
        self.S_obj = shared_Obj()
        # shared obj dictionary
        shared = self.S_obj.shared
        shared['M_list'] = self.m.M_list
        shared['T_list'] = copy.deepcopy(self.m.threaten)
        shared['T_old'] = self.m.threaten[:self.m.threaten_num[0]]
        shared['Forbid'] = self.m.forbidden_area_vertices
        # shared new pr
        shared['T_note'] = self.T_note
        shared['TarNew'] = self.TarNew





    def evn_detect(self,pos, time):
        
        # init parameter
        risk = [0 for i in range(self.num)]
        return_time = [-1 for i in range(self.num)]
        end = False
        m = self.m
        # num of UAV not return
        out_count = self.num
        out_range = {}
        out_range['open'] = [False for i in range(self.num)]
        out_range['record'] = []
        hidden_tar = m.target[m.target_num[0]:]
        info = []
        #M_list_mgr = Manager().list(M_list)
        mission_complete = []
        mission_complete_index = []
        M_list = m.M_list
        #settinf random fail
        # 放入 m
        fail_uav_index = self.m.fail['index']
        fail_uav_time = self.m.fail['time']
        fail_list = []
        # thr moving
        # gen moving
        thr_move_to = self.m.thr_move['to']
        thr_move_counter = self.m.thr_move['count'][:]
        for i in range(m.threaten_num[1]):
            index = i + m.threaten_num[0]
            m.threaten[index] = list(m.threaten[index])
            self.T_note.append(m.threaten[index])
        #set up algorithm process
        self.set_up_templete()
        algo = DoWork(shared = shared_Obj ,args=(pos, info) ,name='algo')
        algo.start() # 程序開始
        while(True):
            tq = time[0]
            info = []
            # thr move
            for i in range(m.threaten_num[1]):
                index = i + m.threaten_num[0]
                #
                m.threaten[index][0] += thr_move_to[i][0]
                m.threaten[index][1] += thr_move_to[i][1]
                thr_move_counter[i][0] +=1
                if(thr_move_counter[i][0] == thr_move_counter[i][1]):
                    thr_move_to[i][0] *= -1
                    thr_move_to[i][1] *= -1
                    thr_move_counter[i][0] = 0
            # detect by every UAV
            for i in range(len(pos)):
                if i in fail_list:
                    continue
                if(distance(pos[i],self.m.start_pos) >= self.m.map_size[1]/3):
                    # detect inter range
                    # leader would not in to this
                    leader = 5*int(i/5)
                    for s in range(i+1,leader+5):
                        dis = distance(pos[i],pos[s])
                        # collision range is 100m
#                        if(dis < 10):
##                            print('collision on %d, %d' %(i,s))
##                            out_count -= 2
##                            self.TarNew += M_list[i]
##                            self.TarNew += M_list[s]
##                            M_list[i] = []
##                            M_list[s] = []
##                            info.append(['fail', i])
##                            info.append(['fail', s])
##                            fail_list.append(i)
##                            fail_list.append(s)
                        #communication range is 500m
                        #elif(s == leader):
                        if(s == leader):
                            if out_range['open'][i] is False and dis > 50:
                                out_range['open'][i] = tq
                            elif out_range['open'][i] is not False and dis < 50:
                                ts = out_range['open'][i]
                                out_range['record'].append((i, ts , tq))
                                out_range['open'][i] = False
                del_list = []
                p = pos[i]
                # new target(detect hidden target)
                for j in range(len(hidden_tar)):
                    hitar = hidden_tar[j]
                    if distance(p, hitar) < map.sensor_r:
                        del_list.append(j)
                        self.TarNew.append([m.target.index(hitar)  , [hitar]])
                        if self.hid_visual and self.visual:
                            self.v.ani.set_hidden(hitar, 'target' )
                        #print('new target' +str(hitar))
                        info.append(['new target', hitar,i])
                for j in del_list:
                    del hidden_tar[j]
                # mission complete(target)
                # only autodetect the target in mlist
                if len(M_list[i]) == 0:
                    if return_time[i] == -1 and distance(p, m.start_pos) == 0:
                        print('UAV %d return' %i)
                        out_count -= 1
                        return_time[i] = tq
                else:
                    for j in range(len(M_list[i])):
                        mis = 0
                        if M_list[i][j][0] == -1:
                            # return start detect
                            if len(M_list[i]) == 1:
                                continue
                            #swap
                            elif j ==0:
                                mis = m.target[M_list[i][j+1][0]]
                                temp = M_list[i][j+1][:]
                                M_list[i][j+1] = M_list[i][j]
                                M_list[i][j] = temp
                            else:
                                continue
                        else:
                            mis = m.target[M_list[i][j][0]]
                        # when out of range can't detect target
                        if out_range['open'][i] is not False:
                            break
                        # 檢查index
                        if mis != 0 and (distance(p, mis) < map.reach_range):
                            iofmis = m.target.index(mis)
                            if iofmis in mission_complete_index:
                                continue
                            mission_complete.append((M_list[i][j],tq))
                            mission_complete_index.append(iofmis)
                # 逢 10 動, 且 二階 detect
                for thr_index in range(len(m.threaten)):
                    thr = m.threaten[thr_index]
                    if distance(p, thr[0:2]) < thr[2]:
                        risk[i] += 1
                    # 會頻繁發送
                    if  thr_index > m.threaten_num[0] and  distance(p, thr[0:2]) < map.sensor_r:
                        index =  thr_index - m.threaten_num[0]
                        if self.hid_visual and self.visual:
                            self.v.ani.set_hidden(thr, 'threat' , index)
                        #print('threat move' +str(thr))
                        info.append(['detect threat', thr, i])
                        #T_list[thr_index] = thr
                        self.T_note[index] = thr
                #fail when in to forbid
                if(m.in_area(p)):
                    print('console: ',M_list[i][0][0])
                    print('in forbid fail: %d   , at time :  %d at pos %s' %(i,tq,str(p)))
                    # not fail again
                    out_count -= 1
                    self.TarNew += M_list[i]
                    M_list[i] = []
                    info.append(['fail', fail_uav_index])
                    fail_list.append(i)
    ##test
    #            if(len(M_list[i]) == 0):
    #                print('error i:%d  t:%d' %(i,tq))
    #                out_count -= 1
    #                fail_list.append(i)
            # 幫助修改結構
            if tq == fail_uav_time:
                print('auto fail',fail_uav_index)
                out_count -= 1
                self.TarNew += M_list[fail_uav_index]
                M_list[fail_uav_index] = []
                info.append(['fail', fail_uav_index])
                fail_list.append(fail_uav_index)
            if (algo.is_alive() is False):
                algo = DoWork(shared = shared_Obj ,args=(pos, info) ,name='algo')
                algo.start() # 程序開始
            else:
                algo.join(1)
            if(out_count == 0):
                end = True
            yield {'end':end,'mission_complete':mission_complete,'return_time':return_time,'risk':risk,'out_range':out_range}



    def takeoff(self, start, a, x, far):
        
        num = self.num
        t = []
        for i in range(num):
            t.append(self.m.M_list[i][0][1][1][:])
        to = [normalize([t[j][i]-start[j][i] for i in range(2)]) for j in range(num)]
        direc = [(0,0),(-1,-1),(-1,1),(1,1),(1,-1)]
        target = [[start[j][i]+a*to[j][i]+x*direc[j%5][i] for i in range(2)] for j in range(num)]
        v = []
        p = []
        dis = []
        step = [0 for i in range(num)]
        for i in range(num):
            dif = [target[i][0]-start[0][0],target[i][1]-start[0][1]]
            v.append(dif)
            p.append(start[i][:])
            dis.append(distance(target[i],start[0]))
        for tt in range(far):
            for i in range(num):
                p[i][0] += v[i][0]/far
                p[i][1] += v[i][1]/far
            yield p
        for i in range(num):
            del self.m.M_list[i][0][1][0]



    #修改中心
    def mov(self):
            #set reference
            num = self.num
            m = self.m
            #
            time = [0]
            p = [[m.start_pos[0], m.start_pos[1]] for i in range(num)]  # 每分鐘位子
            #point_index = [0 for i in range(num)] # 取下一個位子index
            change_target = [True for i in range(num)] # flag
            # 移動位子參考
            x = [0 for i in range(num)]
            y = [0 for i in range(num)]
            step = [0 for i in range(num)] # 目前走幾步
            d = [2 for i in range(num)] # 該path 長
            # 目標採樣點
            start = [[m.start_pos[0], m.start_pos[1]] for i in range(num)]
            end = [[m.start_pos[0], m.start_pos[1]] for i in range(num)]
            evnd = self.evn_detect(p, time)
            count = 0
            a = 50
            inter_dis = 50
            far = int(math.sqrt(((inter_dis)**2)+((inter_dis+a)**2)))+1
            takeoff_flag = True
            # paths_d = [m.paths_d[i][0:] for i in range(num)]
            #next_point = [[0, 0] for i in range(num)]
            # temp
            #self.hidden_trigger_time = self.m.hidden_trigger_time.copy()
            for tq in range(map.time+500):
                time[0] = tq
                if(takeoff_flag and tq < far-1):
                    for ttq in self.takeoff(p, a,inter_dis, far):
                        p[:] = ttq[:]
                        yield {'tq': tq, 'newd': p}
                        tq += 1
                    takeoff_flag = False
                for i in range(15):
                    # fail or no path to go
                    if(len(m.M_list[i]) == 0):
                        continue
                    # revise by algorithm to do a turn
                    if(len(m.M_list[i][0][1]) > 0 and end[i] != m.M_list[i][0][1][0]):
                        change_target[i] = True
                        end[i] = p[i]
                    elif(step[i] >= d[i]):
                        change_target[i] = True
                        #pop_out the first point(auto delete the first point in path)
                        if(tq >=1) and (len(m.M_list[i][0][1]) > 1):
                            del m.M_list[i][0][1][0]
                        elif (tq >=1) and (len(m.M_list[i][0][1]) == 1):
                            del m.M_list[i][0]
                            if (len(m.M_list[i]) == 0):
                                continue
                    if(change_target[i]):
                        #print(m.M_list[i][0][1])
                        ##################################
                        # 條件變更
                        start[i] = p[i]
                        # the [i][0][1][0] is UAV [i] first mission [0] path[1] one[0]
                        end[i] = m.M_list[i][0][1][0]
                        #定義移動距離
                        x[i] = end[i][0] - start[i][0]
                        y[i] = end[i][1] - start[i][1]
                        # d 是到目標點需要步數，step 是目前步數
                        step[i] = 0
                        d[i] = math.sqrt((x[i]**2)+(y[i]**2))
                        # 設立初始位置
                        change_target[i] = False
                    step[i] += map.speed
                    if(d[i] == 0):
                        continue
                    # 限制精確位數
                    p[i][0] = round(p[i][0] + x[i]*map.speed/d[i],3) if step[i]< d[i] else end[i][0]
                    p[i][1] = round(p[i][1] + y[i]*map.speed/d[i],3) if step[i]< d[i] else end[i][1]
                result = next(evnd)
                if result['end'] == True:
                    break
                yield {'tq': tq, 'newd': p}
            self.report_result(result,tq)
                              
    def report_result(self,result,tq):
        
        print('Total time:%d' %tq)
        print('return time:%s' %str(result['return_time']))
        preturn_time = []
        for t in result['return_time']:
            if t != -1:
                preturn_time.append(t)
        print('time sum',sum(preturn_time))
        print('mean time:',(sum(preturn_time)/len(preturn_time)))
        print('risk:%s' %str(result['risk']))
        print('risk sum',(sum(result['risk'])))
        print('mean risk:',(sum(result['risk'])/len(result['risk'])))
        print('max risk:',max(result['risk']))
        print('complete target num:%d' %(len(result['mission_complete'])))
        self.intp_out_range(max(result['return_time']), result['out_range'])
        #resotre
        self.m.M_list = copy.deepcopy(self.m.paths)
        self.m.threaten = copy.deepcopy(self.S_obj.shared['T_list'])



    def intp_out_range(self, ftq,out_range):
        
        open = out_range['open']
        record = out_range['record']
        for i in range(len(open)):
            if open[i] is not False:
                record.append((i, open[i] , ftq))
        for r in record:
            print('UAV %d: out of communication in %d ~ %d'%(r[0],r[1],r[2]))

    def mission_allocation(self):
       
        m = self.m
        # order decide
        param = {'GrpNum':3,'Print':False,'Cycle':12}
        #use 'Print' to print the cycles of clustering
        #if the eval takes too much time please reduce the cycle
        center = center_org.Center_ver2(m,m.target_num[0])
        path = center.clustering(param)
        tar = center.tar
        path_idx = [[999 for j in range(len(path[i])-2)] for i in range(len(path))]
        for i in range(len(path)):
            for j in range(1,len(path[i])-1):
                for k in range(len(tar)):
                    if path[i][j]==tar[k]:path_idx[i][j-1]=k
        ###duplicate path_idx
        dul_path_idx = []
        for dul in path_idx:
            for d_times in range(5):
                dul_path_idx.append(dul)

        prm_planning.map_size = m.map_size
        preMlist = prm_planning.PRM(m.start_pos, m.target[:m.target_num[0]], m.forbidden_area_vertices,m.threaten, dul_path_idx)

        return preMlist




if __name__ == '__main__':
    
    u = uav(threaten_num = (19,1))
    u.run(visual = True)




