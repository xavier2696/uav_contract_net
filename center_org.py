from map import*
import numpy as np
from ACO_Algorithm import*


print_ = False
# =============================================================================
# no need to enter extra input
# all the output is printed in the main function at the bottom of the code
# fix the parameter in param in __main__ if you want to see the iterations
# =============================================================================


class Center_ver2:
    
    def __init__(self, map_,tar_range):
        self.map = map_
        self.tar_range = tar_range
        
        
    def clustering(self, param = {'GrpNum':3,'Print':True, 'Cycle':12}):
        self.tar = self.map.target[:self.tar_range]
        tar = self.tar
        self.param = param
#        print(tar)
        dtype_ = [('Angle', float), ('Index', '<i1')]
        toSort = np.array([(np.arctan2(tar[i][0]-self.map.start_pos[0],tar[i][1]-self.map.start_pos[0]),i) for i in range(len(tar))]\
                           ,dtype = dtype_)
        sorted_info = np.sort(toSort, order = 'Angle')
        print(sorted_info)
        sorted_tar = [tar[sorted_info[i][1]] for i in range(len(tar))]
        self.sorted_tar = sorted_tar
        pointer = [int((len(tar)+1)/param['GrpNum']*(i)) for i in range(param['GrpNum'])]
#        print(pointer)
        self.res_gen(pointer)
        res = self.res
        path, value = res[:,0], res[:,1]
#        path  = [res[i][0] for i in range(param['GrpNum'])]
        if param['Print']==True:
            print("the first value:")
            print(pointer)
            print(value)
            print(path)
        path_res, value_res = [], [np.inf for i in range(param['GrpNum'])]
        t=0
        rate = self.map.map_size[0]*8/15
        while t<param['Cycle']:
            if abs(value[0] - value[1])>10 and t%3==0:
                pointer[1]+=int(((value[1] - value[0])/rate)//1)
            if abs(value[1] - value[2])>10 and t%3==1:
                pointer[2]+=int(((value[2] - value[1])/rate)//1)
            if abs(value[2] - value[0])>10 and t%3==2:
                pointer[0]+=int(((value[0] - value[2])/rate)//1)
            self.res_gen(pointer)
            path, value = self.res[:,0], self.res[:,1]
#            print(len(path[0]),len(path[1]),len(path[2]))
            if param['Print']==True:
                print(pointer)
                print(value)
#            print()
            if max(value) <= max(value_res)+5:
                if max(value) < max(value_res) or sum(value) < sum(value_res)-15:
                    value_res = [value[i] for i in range(len(value))]
                    path_res = [path[i] for i in range(len(path))]
            t+=1
        print('final cost:\n',value_res)
        return path_res
        """
        value_res: the best costs
        """
        
    def res_gen(self,pointer):
    
        sorted_tar = self.sorted_tar
        res = []
        if pointer[0] < 0:
            res.append(self.min_dist(sorted_tar[pointer[0]:]+sorted_tar[0:pointer[1]]))
        elif pointer[0] >= 0:
            res.append(self.min_dist(sorted_tar[pointer[0]:pointer[1]]))
        
        for i in range(1,self.param['GrpNum']-1):
            res.append(self.min_dist(sorted_tar[pointer[i]:pointer[i+1]]))
        
        if pointer[0] < 0:
            res.append(self.min_dist(sorted_tar[pointer[-1]:(pointer[0]+len(self.tar))]))
        elif pointer[0] >= 0:
            res.append(self.min_dist(sorted_tar[pointer[-1]:(len(self.tar)+1)]+sorted_tar[0:pointer[0]]))
#            print(self.min_dist(sorted_tar[pointer[-1]:(len(self.tar)+1)]+sorted_tar[0:pointer[0]]))
#            print("==========================================================")
        self.res = np.array(res)
        
    def min_dist(self,array):
        targets = array
        targets.append(list(self.map.start_pos))
#        print(targets)
        cities = []
        points = []
        
        cost_matrix = []
        rank = len(targets) #number of targets
        for i in range(rank):
            row = []
            for j in range(rank):
                row.append(distance(targets[i], targets[j]))
            cost_matrix.append(row)
            
            
        aco = ACO(20, 50, 1.0, 10.0, 0.5, 100, 2)
        graph = Graph(cost_matrix, rank)
        path_idx, cost = aco.solve(graph)
        zero_idx = 99
        for i in range(rank):
            if path_idx[i] == rank-1:
                zero_idx = i
#        print(zero_idx==rank-1, zero_idx,path_idx[-1])
        path = [targets[path_idx[(zero_idx+i)%rank]] for i in range(rank+1)]
        return path, cost




if __name__ == '__main__':
    m = map_gen()
    m.gen()

    param = {'GrpNum':3,'Print':print_,'Cycle':12}
    #use 'Print' to print the cycles of clustering
    #if the eval takes too much time please reduce the cycle
    print("enter the center_ver2 process")
    center = Center_ver2(m)
    path = center.clustering(param)
    tar = center.tar
    
    
    path_idx = [[999 for j in range(len(path[i])-2)] for i in range(len(path))]
    for i in range(len(path)):
        for j in range(1,len(path[i])-1):
            for k in range(len(tar)):
                if path[i][j]==tar[k]:path_idx[i][j-1]=k
    print("\n","index of path:","\n",path_idx)
    print("\n","pos of path:","\n",path)# the value for the next step(no need to print)
