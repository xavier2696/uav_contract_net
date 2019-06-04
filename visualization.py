import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import math


dot_form = {'threaten':'r^', 'target':'b.', 'forbidden':'g' ,'trace':'r--','UAV':'r.'}
UAV_num = 15


class ani():
    
    def __init__(self, v, mov, paths = []):
        self.v = v
        self.m = v.map
        self.start_pos = v.map.start_pos
        self.num = len(paths)
        self.mov = mov
        #self.num = len(path)
        #self.path = path
        l = self.v.ax.plot(0, 0)
        # 顏色要建
        self.UAV,   = self.v.ax.plot([], [], dot_form['UAV'])
        self.trace = {}
        for i in range(self.num):
            self.trace[i], =  self.v.ax.plot([], [], '--')#dot_form['trace'])
        #self.trace[1], =  self.v.ax.plot([], [], 'g--')
        self.hidden,   = self.v.ax.plot([], [], dot_form['threaten'])
        # temp
        self.h_circle = []
    
    
    
    # 輸入飛行路徑,每一個轉折定義多個轉彎目標點
    def update(self, pack):
        newd = pack['newd']
        tq = pack['tq']
        return self.update_pos(newd, tq), self.update_trace(newd,tq), self.update_hidden(tq)

    def update_pos(self, newd, tq):
        
        x = [newd[i][0] for i in range(self.num)]
        y = [newd[i][1] for i in range(self.num)]
        self.UAV.set_data(x,y)
        label = 'Time: {0}\n'.format(tq)
        #label += 'UAV[0] now pos: {0}\n'.format(newd[0])
        #label += 'UAV[1] now pos: {0}\n'.format(newd[1])
        self.v.ax.set_xlabel(label)
        return self.UAV, self.v.ax
    
    # 跟著path後面產生移動軌跡
    def update_trace(self, newd, tq):
        if tq == 0:
            self.init_trace(self.num)
        for i in range(self.num):
            self.xtrace[i].append(newd[i][0])
            self.ytrace[i].append(newd[i][1])
            self.trace[i].set_data(self.xtrace[i], self.ytrace[i])
        return self.trace
            
    def init_trace(self, num):
        self.xtrace = [[] for i in range(num)]
        self.ytrace = [[] for i in range(num)]
    
    # 採用dict 整理不同hidden類型
    # how to detect already add index
    def set_hidden(self, data,case, index = 0):
        #case include target, threat
        if case == 'target':
            self.hidden_target_data[0].append(data[0])
            self.hidden_target_data[1].append(data[1])
        elif case == 'threat':
            self.hidden_threat_data[0][index] = data[0]
            self.hidden_threat_data[1][index] = data[1]
            self.h_circle[index].remove()
            self.h_circle[index] = self.v.draw_circle(data)

    
    def update_hidden(self, tq):
        if(tq==0):
            # store data
            self.hidden_target_data = [[],[]]
            # set test
            #pre threat num
            pthrn = self.m.threaten_num[0]
            rhithr_num = range(self.m.threaten_num[1])
            self.hidden_threat_data =[[self.m.threaten[pthrn+i][j] for i in rhithr_num] for j in range(3)]
            # store data point on graph
            self.hidden_target, = self.v.ax.plot([], [],dot_form['target'],)
            self.hidden_threat, = self.v.ax.plot(self.hidden_threat_data[0], self.hidden_threat_data[1], dot_form['threaten'],)
            for c in self.h_circle:
                c.remove()
            self.h_circle = [self.v.draw_circle(self.m.threaten[pthrn+i]) for i in rhithr_num]
        else:
            tx = self.hidden_target_data[0]
            ty = self.hidden_target_data[1]
            thrx = self.hidden_threat_data[0]
            thry = self.hidden_threat_data[1]
            self.hidden_target.set_data(tx , ty)
            self.hidden_threat.set_data(thrx , thry)
        return self.hidden_target, self.hidden_threat
    
    
    
    def show(self):
        ani = FuncAnimation(self.v.fig, self.update, frames = self.mov, interval = 1)
        #ani.save('test.gif', writer='pillow', fps=10)
        plt.show()


class show_visual():

    def __init__(self, map, mov):
        self.map = map
        self.fig, self.ax = plt.subplots()
        self.start_pos = map.start_pos
        # 因對照尺寸 x:y = 4:3 , 並使中心點在中間
        x = map.map_size[0]
        y = map.map_size[1]

        plt.xlim(x/2-y*1.1*2/3, x/2+y*1.1*2/3)
        plt.ylim(y/2-(1.1*y/2), y/2+(1.1*y/2))
        
        #plt.xlim(map.map_size[0]*-0.1, map.map_size[0]*1.1)
        #plt.ylim(0, map.map_size[1])
        #plt.axis('equal')
        self.ani = ani(self, mov, paths = self.map.paths)
        # 為了其他串接，若沒問題可刪
        map.fig = self.fig
        map.ax = self.ax


    def show(self):
        self.gen_start()
        self.gen_forbidden()
        self.gen_threaten()
        self.gen_target()
        self.ani.show()
        plt.show()

    def gen_start(self):
        rect = plt.Rectangle((self.start_pos[0]-0.5,self.start_pos[1]-0.5), 10, 10, fill=False,angle=0, edgecolor = 'orange',linewidth=1)
        self.ax.add_patch(rect)

    def gen_forbidden(self):
        for i in range(len(self.map.forbidden_area_vertices)):
            patch = patches.PathPatch(self.map.forbidden_area[i], facecolor='g', lw=0)
            self.ax.add_patch(patch)

    def gen_threaten(self):
        self.plotData(plt, self.map.threaten[:self.map.threaten_num[0]], dot_form['threaten'])
        for i in range(self.map.threaten_num[0]):
            self.draw_circle(self.map.threaten[i])
    
    
    def gen_target(self):
        self.plotData(plt, self.map.target[:self.map.target_num[0]], dot_form['target'])
        for i in range(self.map.target_num[0]):
            self.label(self.map.target[i], str(i))
    
    def label(self, xy, text):
        y = xy[1] - 3  # 标签放置在patch下方的0.15位置处
        plt.text(xy[0], y, text, ha="center", size=8)

    def plotData(self, plt, data, dot_form = 'o'):
        x = [p[0] for p in data]
        y = [p[1] for p in data]
        plt.plot(x, y, dot_form)
    
    def draw_circle(self, center):
        circle = plt.Circle(center[0:2], center[2], color='r',fill = False)
        return self.ax.add_artist(circle)
        #remove test



