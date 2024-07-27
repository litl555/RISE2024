import sys
import gtsam
from gtsam import Pose2
from gtsam import noiseModel
import numpy as np
print(sorted(gtsam.__dir__()))
from gtsam import NonlinearFactor
from gtsam import PriorFactorPoint3
from gtsam import BetweenFactorPoint3
from gtsam.gtsam import Values
from gtsam import LevenbergMarquardtOptimizer
f=gtsam.NonlinearFactorGraph()
v=[1,.1,1]
noise=noiseModel.Diagonal.Sigmas(v)
verts=np.load("/home/clearpath/jackal_ws/src/apriltag_mapping/verts.npy")
ids=list(np.load("/home/clearpath/jackal_ws/src/apriltag_mapping/ids.npy"))
last_tag=np.load("/home/clearpath/jackal_ws/src/apriltag_mapping/final_tag_pos.npy")
all_tags=np.load
start=ids.index(b'18')
p=PriorFactorPoint3(1,[verts[start][0],0.0,verts[start][1]],noise)
f.add(p)
indexNum=1
for i in range(len(verts)-1,0,-1):
    f.add(BetweenFactorPoint3(indexNum,indexNum+1,[verts[i+start-len(verts)][0]-verts[i+start-len(verts)+1][0],0.0,verts[i+start-len(verts)][1]-verts[i+start-len(verts)+1][1]],noise))
    print("making edge from "+str(ids[i+start-len(verts)+1])+" to "+str(ids[i+start-len(verts)]))
    print([verts[i+start-len(verts)][0]-verts[i+start-len(verts)+1][0],0.0,verts[i-start][1]-verts[i+start-len(verts)+1][1]])
    print(indexNum)
    print(len(verts))
    indexNum+=1
    print()
f.add(BetweenFactorPoint3(len(verts),1,[last_tag[0]-verts[start+1][0],0.0,last_tag[1]-verts[start+1][1]],noise))
print("making edge from "+str(ids[start+1])+" to "+str(ids[start]))




val=Values()
insert_ind=1
for i in range(len(verts),0,-1):
    val.insert(insert_ind,[verts[i+start-len(verts)][0],0.0,verts[i+start-len(verts)][1]])
    print(ids[i+start-len(verts)])
    insert_ind+=1
#val.insert(1, [-0.03523958795627748, 0.0,1.569379627284615])
#val.insert(2, [-1.7539974148304145, 0.0,0.3721668578189723])
#val.insert(3, [-1.6593294139697636, 0.0,-0.8459824522267847])
#val.insert(4, [-0.07416112874160274,0.0, -1.7676295873521952])
#val.insert(5, [1.419039087377808, 0.0,-0.7111007004228176])
#val.insert(6, [1.4580077635488922, 0.0,0.5026124010016162])

opt=LevenbergMarquardtOptimizer(f,val).optimize()
print(f.error(opt))
print(opt)