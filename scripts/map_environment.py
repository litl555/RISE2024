import rospy
import copy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import geometry_msgs.msg

from geometry_msgs.msg import Twist
import math
import tf.transformations as tr
import numpy as np

import procrustes_analysis
"""
General flow of code:

Broadcaster class takes apriltagdetections and broadcasts them as Transforms
in the robot's reference frame.

TagCalculator broadcasts the transforms between tags and uses initial transform of the
first tag in the robot frame to define world frame axes

GlobalPositions takes the transforms between tags and uses them to calculate each tag's
position in the world frame.
"""
# Takes AprilTags and broadcasts them as transforms relative to camera
class Broadcaster():
    current_tags=[] # Sorted list of every tag that is in camera view
    
    can_add_first=False
    def __init__(self):

        self.all_tags=[] #contains ids of every tag in the order that they are seen
        self.first_buffer=0
        self.is_first=True
        self.first_tag=None

    def initialize(self):
        self.sub=rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.sub_callback,buff_size=10)
        self.br=tf2_ros.TransformBroadcaster()

        
    def get_all_tags(self):
        return self.all_tags
    # Processes tags and broadcasts them as transforms relative to robot
    def sub_callback(self,msg):
        if msg.detections:
            
            
            d=self.sort_tags(msg.detections)
            # Add currently viewed ids to current_tags and publish
            Broadcaster.current_tags=[]
            for i in range(len(msg.detections)):
                
                if d[i].id[0] not in self.all_tags:
                    self.all_tags.append(d[i].id[0])
                try:
                    self.stamp_tag(d[i],d[i].id[0])
                except rospy.ROSException:
                    continue
                Broadcaster.current_tags.append(d[i].id[0])
            #saves the pose of the first tag so it can be used as the world position
            if self.is_first:
                t=geometry_msgs.msg.Transform()
                pos=d[0].pose.pose.pose.position
                t.translation.x=pos.x
                t.translation.y=pos.y
                t.translation.z=pos.z
                q=d[0].pose.pose.pose.orientation
                t.rotation.x=q.x
                t.rotation.y=q.y
                t.rotation.z=q.z
                t.rotation.w=q.w
                self.first_tag=t
                self.is_first=False
                TagCalculator.vel.angular.z=-.1
        else:
            Broadcaster.current_tags=[]
        if len(self.all_tags)!=0:
                if self.all_tags[0] not in Broadcaster.current_tags:
                    self.first_buffer+=1
                if self.first_buffer>=10:
                    Broadcaster.can_add_first=True
                if self.all_tags[0] in Broadcaster.current_tags and Broadcaster.can_add_first:
                    self.all_tags.append(d[len(d)-1].id[0])
    
    def get_first_tag(self):
        return self.first_tag
    def stamp_tag(self,apriltag,id):
        pose=apriltag.pose.pose.pose
        #pose.orientation.x,y,z,w
        
        t=geometry_msgs.msg.TransformStamped()
        t.header.stamp=rospy.Time.now()
        t.header.frame_id="robot"
        t.child_frame_id =str(id)
        pos=pose.position
        t.transform.translation.x=pos.x
        t.transform.translation.y=pos.y
        t.transform.translation.z=pos.z
        q=pose.orientation
        t.transform.rotation.x=q.x
        t.transform.rotation.y=q.y
        t.transform.rotation.z=q.z
        t.transform.rotation.w=q.w
        self.br.sendTransform(t)
        return t
    #make sure tags are in order from left to right on screen
    def sort_tags(self,msg):
        for i in range(len(msg)-1):
            
            for x in range(i+1,len(msg)):
                
                if msg[x].pose.pose.pose.position.x<msg[i].pose.pose.pose.position.x:
                    msg=self.swap(msg,x,i)

        return msg


    def swap(self, list,index1,index2):
        list1=copy.deepcopy(list)
        list[index1]=list[index2]
        list[index2]=list1[index1]
        return list

class TagCalculator():
    transform_list=[]
    vel=Twist()
    def __init__(self):
        self.tag_rotations=[]
        self.transform_iter=[0,0,0,0,0,0,0,0,0]
        self.distance_tag_one=0
        self.br=tf2_ros.TransformBroadcaster()
        self.complete_loop=False
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        
        
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def publish_transform_list(self):
        l=TagCalculator.transform_list
        for i in l:
            
            i.header.stamp=rospy.Time.now()
            
            self.br.sendTransform(i)
    def calc_score(self,rotation1,rotation2,r_inte):
        procrsustes_sum=0.
        
        for i in range(len(rotation1)):
            r2=rotation2[i]
            r1=rotation1[i]
            r1=tr.euler_from_quaternion([r1.x,r1.y,r1.z,r1.w])
            r2=[r2.x,r2.y,r2.z,r2.w]
            #print("r2")
            #print(tr.euler_from_quaternion(r2))
            #print("r_inter")
            #print(tr.euler_from_quaternion(r_inte))
            #print("r1")
            #print(r1)
            angle_image=np.matmul(tr.quaternion_matrix(r_inte),tr.quaternion_matrix(r2))
            angle_image=tr.euler_from_matrix(angle_image)
            procrsustes_sum+=(angle_image[0]-r1[0])**2+(angle_image[1]-r1[1])**2+(angle_image[2]-r1[2])**2
        return procrsustes_sum
    def average_rotation_modifier(self):
        tag_index=0
        
        for rotation_lists in self.tag_rotations:
            min_score=1000000
            min_rotation=None
            for i in range(len(rotation_lists[0])):
                r_inter=rotation_lists[2][i]
                r2=rotation_lists[1]
                r1=rotation_lists[0]
                score=self.calc_score(r1,r2,[r_inter.x,r_inter.y,r_inter.z,r_inter.w])
                if score<min_score:
                    min_score=score
                    min_rotation=r_inter
            TagCalculator.transform_list[tag_index].transform.rotation=min_rotation
            tag_index+=1
        print("################## TRANSFORM LIST ####################")
        print(TagCalculator.transform_list)
        
    #Call this every loop to publish transforms from one tag to the adjacent
    def publish_adjacent_transforms(self,first_tag):
        
        #if len(Broadcaster.current_tags) is 0:
         #   TagCalculator.vel.angular.z=-.75
        
        #If loop complete add the first tag's pose
        if b.get_all_tags()[len(b.get_all_tags())-1]==b.get_all_tags()[0] and Broadcaster.can_add_first and self.complete_loop==False:
            self.average_rotation_modifier()
            print("first tag")
            print(str(b.get_all_tags()[0])+"_1")
            self.complete_loop=True
            t=geometry_msgs.msg.TransformStamped()
            tr=geometry_msgs.msg.Transform()
            tr=first_tag
            t.transform=tr
            t.child_frame_id=str(b.get_all_tags()[0])+"_1"
            t.header.frame_id="world"
            t.header.stamp=rospy.Time.now()
            TagCalculator.transform_list.append(t)
        #Constantly publishes the transforms
        if self.complete_loop:
            self.publish_transform_list()
            TagCalculator.vel.angular.z=0.0
            
            
        
        if not self.complete_loop:
            #keeps running average of the transforms from one tag to the other
            for i in range(len(Broadcaster.current_tags)-1):
                itag=Broadcaster.current_tags[i]
                itag1=Broadcaster.current_tags[i+1]
                insert_index=b.get_all_tags().index(Broadcaster.current_tags[i])
                self.transform_iter[insert_index]+=1
                output=self.tf_buffer.lookup_transform(str(itag),str(Broadcaster.current_tags[i+1]),rospy.Time(0),rospy.Duration(10.00))
                output.header.frame_id=output.header.frame_id+"_1"
                output.child_frame_id=output.child_frame_id+"_1"
                rotation1=self.tf_buffer.lookup_transform("robot",str(itag),rospy.Time(0),rospy.Duration(2)).transform.rotation
                rotation2=self.tf_buffer.lookup_transform("robot",str(itag1),rospy.Time(0),rospy.Duration(2)).transform.rotation

                try:
                    index=b.get_all_tags().index(Broadcaster.current_tags[i])
                    self.tag_rotations[index][0].append(rotation1)
                    self.tag_rotations[index][1].append(rotation2)
                    self.tag_rotations[index][2].append(output.transform.rotation)
                    #print("######################################################################################################")
                    #print("appending at "+str(b.get_all_tags().index(Broadcaster.current_tags[i])))
                except Exception as e:
                    self.tag_rotations.insert(b.get_all_tags().index(itag),[[rotation1],[rotation2],[output.transform.rotation]])
               
                if len(TagCalculator.transform_list)>insert_index:
                    output.transform=self.average_transform(TagCalculator.transform_list[insert_index],output,self.transform_iter[insert_index])
                    
                if len(TagCalculator.transform_list)>insert_index:
                    TagCalculator.transform_list.pop(insert_index)
                
                TagCalculator.transform_list.insert(insert_index,output)
    def average_transform(self,t1,t2,weight):
        t=geometry_msgs.msg.Transform()
        
        t.translation.x=(t1.transform.translation.x*weight+t2.transform.translation.x)/(weight+1)
        t.translation.y=(t1.transform.translation.y*weight+t2.transform.translation.y)/(weight+1)
        t.translation.z=(t1.transform.translation.z*weight+t2.transform.translation.z)/(weight+1)
        t.rotation.x=t2.transform.rotation.x
        t.rotation.y=t2.transform.rotation.y
        t.rotation.z=t2.transform.rotation.z
        t.rotation.w=t2.transform.rotation.w
        return t

class GlobalPositions():

    def __init__(self):
        self.buffer=0
        self.br=tf2_ros.TransformBroadcaster()

        self.tf_buffer1 = tf2_ros.Buffer(cache_time=rospy.Duration(1.0))
        self.listener1 = tf2_ros.TransformListener(self.tf_buffer1)
    
    def get_global_positions(self):
        
        global_pos=[]
        for i in range(len(b.get_all_tags())-1):
            if b.get_all_tags()[i]!=b.get_all_tags()[0] or i==0:
                out=self.tf_buffer1.lookup_transform("world",str(b.get_all_tags()[i])+"_1",rospy.Time(0),rospy.Duration(3.0))
            
            
                
                global_pos.append(out)
            else:
                break
        #publishes global positions for visualization in rviz
        
        for i in global_pos:
            l=i
            l.child_frame_id=l.child_frame_id.split("_")[0]+"_1_1"
            self.br.sendTransform(l)
        return(global_pos)
# Maps tags and returns vertices as list of tuples (x,y,z) and ids as list
# z axis is forward relative to initial position of robot, x axis is right 
# relative to initial position, and y axis faces up
# origin of world frame is at camera
#x and y parameters are the model points and only used for procrustes analysis
b=Broadcaster()
def get_vertices(x,y):

    b.initialize()
    c=TagCalculator()
    r=rospy.Rate(10)
    l=GlobalPositions()
    pr=procrustes_analysis.procrustes(x,y)
    #buffer because otherwise it calls nonexistent transform
    buffer=0
    global_positions=None
    while not rospy.is_shutdown():
        
        if len(b.get_all_tags()) is not 0:
            c.publish_adjacent_transforms(b.get_first_tag())
            c.pub.publish(TagCalculator.vel)
            if c.complete_loop:
                #needs to buffer for the transform listeners
                if buffer<2:
                    buffer+=1
                else:
                    global_positions=l.get_global_positions()
                    break   
        r.sleep()
    #procrustes analysis
    global_positions=pr.get_ordered_tags(global_positions)
    
    vertices=[]
    orientations=[]
    tags=[]
    for i in global_positions:
        tags.append(i.child_frame_id.split("_")[0])
        vertices.append((i.transform.translation.x,i.transform.translation.z))
        orientations.append(i.transform.rotation)
    return vertices,tags,orientations
    
if __name__=="__main__":

    rospy.init_node("map_environment")
    
    x=[1,math.cos(math.pi/3.0),math.cos(2.0*math.pi/3.0),-1,math.cos(4.0*math.pi/3.0),math.cos(5.0*math.pi/3.0)]
    y=[0,math.sin(math.pi/3.0),math.sin(2.0*math.pi/3.0),0,math.sin(4.0*math.pi/3.0),math.sin(5.0*math.pi/3.0)]
    
    vertex_coordinates,apriltag_ids=get_vertices(x,y)
    print("Vertex Coordinates:")
    print(vertex_coordinates)
    print("AprilTag IDs:")
    print(apriltag_ids)