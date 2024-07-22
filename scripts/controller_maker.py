#!/usr/bin/env python3.8

import sys
import rospy
from std_msgs.msg import Float64MultiArray
sys.path.append("/home/clearpath/Mahrooalg")
import control_cal
from scipy.spatial.transform import Rotation as R
import csv
import numpy as np

# Generates the controller gains based on map
# RUNS PYTHON3.8
class controller_maker:
    verts=[]
    def __init__(self):
        self.x=None
        controller_maker.verts=[]
        self.y=None
        self.subx=rospy.Subscriber("verticesx",Float64MultiArray,self.sub_callx,10)
        self.suby=rospy.Subscriber("verticesy",Float64MultiArray,self.sub_cally,10)
        self.finish=False
    def sub_callx(self,msg,args):
        print("HI")
        print(msg)
        self.x=msg.data
    def sub_cally(self,msg,args):
        self.y=msg.data
    def convert_k_values(self):
        K_all =np.array([])
        for clid in range(3):

            K= np.load('/home/clearpath/Mahrooalg/control_gains/K'+str(clid)+'.npy')
            if clid == 0:


                K_all =K
            else:
                K_all = np.vstack((K_all,K))
            print(K)
            #np.savetxt("K"+str(clid)+".csv", K, delimiter=",",fmt="%.4f")

        np.savetxt("/home/clearpath/jackal_ws/src/creates_iros/csv/K_gains.csv", np.array(K_all), delimiter=",",fmt="%.4f")
    def run(self):
        try:
            verts=np.load("/home/clearpath/jackal_ws/src/apriltag_environment_mapper/verts.npy")
            ids=np.load("/home/clearpath/jackal_ws/src/apriltag_environment_mapper/ids.npy")
            orientation=np.load("/home/clearpath/jackal_ws/src/apriltag_environment_mapper/orientations.npy",allow_pickle=True)
            if verts.shape[0]!=(1):
        
                rotation_dict=dict()

                with open("/home/clearpath/jackal_ws/src/creates_iros/csv/landmark_positions.csv",'w+') as f:
                    writer=csv.writer(f)
                    print("hi")
                    verts=list(verts)
                    index=0
                    rotation_dict=dict()
                    for i in verts:
                        l=orientation[index]
                        r=R.from_quat([l.x,l.y,l.z,l.w])
                        print(int(ids[index]))
                        l=r.as_matrix()
                        #switch z and y axes and make z axis negative
                        mat_matrix=[l[0],l[2],[-l[1][0],-l[1][1],-l[1][2]]]
                        rotation_dict[str(int(ids[index]))]=mat_matrix
                        print(mat_matrix)
                        formatted=list(i)
                        formatted.insert(0,int(ids[index]))
                        formatted.append(0)
                        writer.writerow(formatted)
                        index+=1
                    f.close()
                self.calculate_rotation_matrices(ids,rotation_dict)
                control_cal.control_cal(np.array(self.np_to_list(verts)))
                self.convert_k_values()
                self.finish=True
        except Exception as e:
            print(e)
    def np_to_list(self,verts):
        l=[]
        for i in verts:
            print(i[0])
            l.append((i[0],i[1]))
        print(l)
        return(l)
    #save rotation matrices to orientation file based on given dictionary
    def calculate_rotation_matrices(self,tags,rotations={
            '14':[[0,0,-1],[-1,0,0],[0,1,0]],
            '16':[[1,0,0],[0,0,-1],[0,1,0]],
            '18':[[1,0,0],[0,0,-1],[0,1,0]],
            '13':[[0,0,1],[1,0,0],[0,1,0]],
            '15':[[-1,0,0],[0,0,1],[0,1,0]],
            '33':[[-1,0,0],[0,0,1],[0,1,0]]
        }):
        
        with open("/home/clearpath/jackal_ws/src/creates_iros/csv/landmark_orientations.csv",'w+') as f:
            w=csv.writer(f)
            
            for i in range(tags.shape[0]):
                for m in rotations[str(int(tags[i]))]:
                    m=[round(m[0]),round(m[1]),round(m[2])]
                    w.writerow(m)
            f.close()

if __name__=="__main__":
    rospy.init_node("controller_maker")
    m=controller_maker()
    r=rospy.Rate(10)
    while not m.finish and not rospy.is_shutdown():
        m.run()
        r.sleep()