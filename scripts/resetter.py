import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import math
import copy
#program that aligns to the specified april tag so I don't have 
#to walk into the room everytime
class Resetter():
    def __init__(self):
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.vel=Twist()
        self.left=None
        self.sub=rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.sub_callback,buff_size=10)
        self.vel.angular.z=-.5
        self.p=3
        self.last_error=0.
        self.d=.2
        self.finished=False
        
    
    def sub_callback(self,msg):
        if msg.detections:
            if self.left is None:
                for i in msg.detections:
                    r=i.pose.pose.pose.orientation
                    t=i.pose.pose.pose.position
                    x,y,z=self.euler_from_quaternion(r.x,r.y,r.z,r.w)
                    self.left=18
            if self.left is not False:
                for i in range(len(msg.detections)):
                    print(msg.detections[i].id)
                    if msg.detections[i].id[0]==self.left:
                        print("is id")
                        r=msg.detections[i].pose.pose.pose.orientation
                        t=msg.detections[i].pose.pose.pose.position
                        x,y,z=self.euler_from_quaternion(r.x,r.y,r.z,r.w)
                        print(y)
                        self.vel.angular.z=self.PID(y)
                        if abs(y)<.25*math.pi/180.0:
                            self.vel.angular.z=0.0
                            self.finished=True
                            self.sub.unregister()
    def PID(self, error):
        output=-error*self.p-self.d*(error-self.last_error)
        self.last_error=error
        return max(min(output,.5),-.5)
        
    def euler_from_quaternion(self,x, y, z, w):
    
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians
    

