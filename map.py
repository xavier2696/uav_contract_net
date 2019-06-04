import random
import math
import copy
from matplotlib.path import Path
import matplotlib.pyplot as plt
import mapmath

#make everything grid based
#map_size = 15000 #(m) 15km
grid = 10 # m
time = 40*60 #min
speed = 15/grid #m/sec
map_size = (1500, 1500)#
start_pos = (int(map_size[0]/2), int(map_size[1]/2))
#Its sensor can detect new threats and forbidden area when it come close to 2 km.
sensor_r = 2000/grid #km
#We assume when UAVs fly nearby a target by 10m , the UAV has reached the target.
reach_range = 10/grid
#1 km to 1.5 km
threaten_range = (100, 150)
print_data = False
Fixed_map = True

class map_gen():
    
    def __init__(self):
        # 原尺寸設定為 40+? 3+2 40,40
        # 後面表示為隱藏數
        self.target_num = (35, 5)
        self.threaten_num = (17,3)
        self.num = 15
        self.map_size = map_size
        self.start_pos = start_pos
        self.target = []
        self.threaten = []
        self.forbidden_area = []
        self.forbidden_area_vertices = []
        self.hidden = {}
        #grid
        self.ox = []
    
    def gen(self):
        if(Fixed_map and mapmath.read()):
            self.set_by_data()
            self.read = True
        else:
            self.gen_forbidden()
            # set up map generator
            self.g = self.rand_map()
            self.gen_threaten()
            self.gen_target()
            self.rand_par()
            self.rand_thr()
            #self.gen_hidden()
            self.read = False
        if len(self.thr_move['to']) != self.threaten_num[1]:
            self.rand_thr()
        if(print_data):
            # forvi
            print('Forbid:'+ str(self.forbidden_area_vertices))
            print('threaten:'+ str(self.threaten))
            print('target:'+ str(self.target))

    def set_by_data(self):
        self.forbidden_area_vertices = mapmath.Forbid
        self.forbid_form()
        self.threaten = mapmath.T_list
        self.target = mapmath.Tar
        self.paths = copy.deepcopy(mapmath.M_list)
        self.M_list = mapmath.M_list
        self.fail = mapmath.fail
        self.thr_move = mapmath.thr_move
    
    
    def set_data(self):
        mapmath.Forbid = self.forbidden_area_vertices
        mapmath.T_list = self.threaten
        mapmath.Tar = self.target
        mapmath.M_list = self.M_list
        mapmath.fail = self.fail
        mapmath.thr_move = self.thr_move
        mapmath.write()
    
    def rand_par(self):

        fail_uav_index = random.randint(0, self.num-1)
        if fail_uav_index in [0,5,10]:
            fail_uav_index += 1
        fail_uav_time = random.randint(500, 1000)# 參數化
        self.fail = {'index':fail_uav_index,'time':fail_uav_time}

    def rand_thr(self):
        self.thr_move = {}
        # direction
        to = []
        #
        count = []
        for i in range(self.threaten_num[1]):
            to.append([random.randint(-1, 1), random.randint(-1, 1)])
            count.append([0, random.randint(100, 500)])
        self.thr_move = {'to':to,'count':count}


    def rand_map(self):
        index = -1
        coord_table = []
        while True:
            x = (random.randint(0, self.map_size[0]-1), random.randint(0, self.map_size[0]-1))
            if x not in coord_table and self.in_area(x) is False and mapmath.distance(x, start_pos) >= map_size[1]/3:
                coord_table.append(x)
                yield x

    
    def gen_spot(self, li, num):
        for i in range(num):
            li.append(next(self.g))
    
    def gen_forbidden(self):
        self.p = []
        # 依照角度產生位置
        n_rad = random.randint(3,6)
        rad = [2*math.pi * (i+ random.uniform(0, 0.5))/n_rad  for i in range(n_rad)]
        random.shuffle(rad)
        self.f_area_num = random.randint(2, n_rad)
        for i in range(self.f_area_num):
            center = [
                        self.map_size[0]/2 + math.sin(rad[i])*self.map_size[0]/random.randint(2,4),
                        self.map_size[1]/2 + math.cos(rad[i])*self.map_size[1]/random.randint(2,4)
                    ]
            verts = []
            n_of_polygon = 4
            angle = [2*math.pi * (n+ random.uniform(0, 0.5))/n_of_polygon for n in range(n_of_polygon)]
            for j in range(n_of_polygon):
                x = int(center[0]+math.sin(angle[j])*random.randint(1,3)*self.map_size[0]/30)
                y = int(center[1]+math.cos(angle[j])*random.randint(1,3)*self.map_size[0]/30)
                verts.append((x, y))
            verts.append(verts[0])
            self.forbidden_area_vertices.append(verts)
        self.forbid_form()
            
    def forbid_form(self):
        self.f_area_num = len(self.forbidden_area_vertices)
        codes = [Path.MOVETO]
        for j in range(3):
            codes.append(Path.LINETO)
        codes.append(Path.CLOSEPOLY)
        for v in self.forbidden_area_vertices:
            p = Path(v, codes)
            self.get_pip(p)
            self.forbidden_area.append(p)


    def get_pip(self, path):
        #point in path
        # 可用凸多邊形 最高點向兩邊向量必向下加速
        pip = []
        box = path.get_extents(transform=None)
        xmin, ymin, xmax, ymax =  box.x0, box.y0, box.x1, box.y1
            
        for i in range(int(xmin),int(xmax)+1):
            for j in range(int(ymin), int(ymax)+1):
                if(path.contains_points([[i, j]])[0]):
                    self.ox.append([i,j])

        #return pip


    #整成一個？
    def gen_threaten(self):
        self.gen_spot(self.threaten, sum(self.threaten_num))
        for i in range(len(self.threaten)):
            thr = list(self.threaten[i])
            thr.append(random.randint(threaten_range[0], threaten_range[1]))
            self.threaten[i] = thr
                
    def gen_target(self):
        self.gen_spot(self.target, sum(self.target_num))
    

    
    # could deal with threaten, forbidden, target dense
    # 判斷是否落在path 所連範圍，邊線內為True，含邊線外為False
    def in_area(self, point):
        for i in range(self.f_area_num):
            if(self.forbidden_area[i].contains_points([point])[0]):
                return True
        return False



    def path_planning(self, paths):
        self.paths = paths
        self.M_list = copy.deepcopy(self.paths)
        #self.paths_d = [self.travel_dis_count(p) for p in paths]







if __name__ == '__main__':
    m = map_gen()
    m.gen()
    m.path_planning(paths)
    m.show()
    m.set_data()



