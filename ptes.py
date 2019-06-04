

import uavs


# the txt fail mapdata would follow the first setting which is a cent_org of target_num = (35,5),threatnum = (17,3)

## case 1  with template that would not handle anything

#uavs.m.target_num = (40,0)
#u = uavs.uav()
# or  the follow call if the mapdata didn't follow the original setting
#    u = uav(replan = True, target_num = (40,0))

#u.run(visual = False)
## case 2
u = uavs.uav(replan = True, target_num = (40,0))
u.run(visual = False)
## case 3
#u = uavs.uav()
#u.run(visual = False)





#wr(1)
#wr(2)
