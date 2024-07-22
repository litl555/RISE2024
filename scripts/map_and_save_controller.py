#!/usr/bin/env python
import map_environment
import resetter
import rospy
import math
import os
import sys
from std_msgs.msg import Float64MultiArray
import numpy as np
from geometry_msgs.msg import Twist
sys.path.append("/home/clearpath/Mahrooalg")
class map_and_save():
    def __init__(self):
        self.r=resetter.Resetter()
        try:
            os.remove("/home/clearpath/jackal_ws/src/apriltag_mapping/ids.npy")
        except:
            print("didn't remove")
        try:
            os.remove("/home/clearpath/jackal_ws/src/apriltag_mapping/orientations.npy")
        except:
            print("didn't remove")
        try:
            os.remove("/home/clearpath/jackal_ws/src/apriltag_mapping/verts.npy")
        except:
            print("didn't remove")
    def run(self):
        #resetter program just uses PID to turn towards specified april tag and then makes resetter.finished true
        while self.r.finished==False:
            self.r.pub.publish(self.r.vel)
        #Start mapping
        if self.r.finished:
            self.r.pub.publish(Twist())
            rospy.sleep(rospy.Duration(1))
            # Shape the real world april tags are modelled after
            x=[1,math.cos(math.pi/3.0),math.cos(2.0*math.pi/3.0),-1,math.cos(4.0*math.pi/3.0),math.cos(5.0*math.pi/3.0)]
            y=[0,math.sin(math.pi/3.0),math.sin(2.0*math.pi/3.0),0,math.sin(4.0*math.pi/3.0),math.sin(5.0*math.pi/3.0)]
            # gets the ordered vertices, ids, and orientations and saves them to numpy file
            vertex_coordinates,apriltag_ids,orientations=map_environment.get_vertices(x,y)
            print(vertex_coordinates)
            np.save("/home/clearpath/jackal_ws/src/apriltag_mapping/verts.npy",np.array(vertex_coordinates))
            np.save("/home/clearpath/jackal_ws/src/apriltag_mapping/ids.npy",np.array(apriltag_ids))
            np.save("/home/clearpath/jackal_ws/src/apriltag_mapping/orientations.npy",np.array(orientations))

            print(vertex_coordinates)
if __name__=="__main__":
    rospy.init_node("map_and_save")
    m=map_and_save()
    m.run()