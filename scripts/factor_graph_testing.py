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
v=[.1,.1,.1]
noise=noiseModel.Diagonal.Sigmas(v)

p=PriorFactorPoint3(1,[-0.03523958795627748, 0.0,1.569379627284615],noise)

f.add(p)
x1=BetweenFactorPoint3(1,2,np.array([-1.75+.035,0,.372-1.569]),noise)
x2=BetweenFactorPoint3(2,3,np.array([-1.66+1.754,0,-.846-.372]),noise)
x3=BetweenFactorPoint3(3,4,np.array([-.074+1.659,0.0,-1.768+.846]),noise)
x4=BetweenFactorPoint3(4,5,np.array([1.419+.074,0,-.711+1.768]),noise)
x5=BetweenFactorPoint3(5,6,np.array([1.458-1.419,0.0,.503+.711]),noise)
x6=BetweenFactorPoint3(6,1,np.array([-.035-1.458,0.0,1.569-.503]),noise)

f.add(x1)
f.add(x2)
f.add(x3)
f.add(x4)
f.add(x5)
f.add(x6)
print(sorted(f.__dir__()))
val=Values()
val.insert(1, [-0.03523958795627748, 0.0,1.569379627284615])
val.insert(2, [-1.7539974148304145, 0.0,0.3721668578189723])
val.insert(3, [-1.6593294139697636, 0.0,-0.8459824522267847])
val.insert(4, [-0.07416112874160274,0.0, -1.7676295873521952])
val.insert(5, [1.419039087377808, 0.0,-0.7111007004228176])
val.insert(6, [1.4580077635488922, 0.0,0.5026124010016162])

opt=LevenbergMarquardtOptimizer(f,val).optimize()
print(f.error(opt))
print(opt)