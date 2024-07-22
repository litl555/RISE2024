import rospy
import tf2_ros
import map_environment as map_environment
import math
import copy
import itertools


class procrustes():
    def __init__(self,x1,y1):
        
        self.w=None
        self.z=None
        self.xi=copy.deepcopy(x1)
        self.yi=copy.deepcopy(y1)
        self.x=x1
        self.y=y1
        
        
    def get_transform(self,perm):
        self.w=[]
        self.z=[]
        for i in perm:
            self.w.append(i.transform.translation.x)
            self.z.append(i.transform.translation.z)
        self.x,self.y=self.shift_points(self.xi,self.yi)
        self.w,self.z=self.shift_points(self.w,self.z)
        scale_desired=self.calculate_scale(self.x, self.y)
        scale_current=self.calculate_scale(self.w,self.z)
        scale_factor=scale_desired/scale_current
        self.x,self.y=self.scale_by_factor(scale_desired/scale_current,self.x,self.y)
        self.w,self.z=self.scale_by_factor(scale_desired/scale_current,self.w,self.z)

        angle=self.newtons_method(self.x,self.y,self.w,self.z)
        self.w,self.z=self.rotate_list(angle,self.w,self.z)
        procrustes_distance=0
        for i in range(len(self.w)):
            procrustes_distance+=math.pow(self.w[i]-self.x[i],2)+math.pow(self.z[i]-self.y[i],2)
        procrustes_distance=math.sqrt(procrustes_distance)

        
        return scale_factor, angle,procrustes_distance
    def get_ordered_tags(self,global_transform_stamped):
        
        
        perms=list(itertools.permutations(global_transform_stamped))
        min_score=1000000
        min_order=[]
        for i in perms:

            _,rotate,score=self.get_transform(i)
            if score<min_score:
                min_score=score
                min_order=[]
                for x in i:
                    
                    min_order.append(x)
        
        
        return(min_order)
    def rotate_list(self,angle,x,y):
        return [math.cos(angle)*j-math.sin(angle)*y[i] for i,j in enumerate(x)],[math.sin(angle)*b+math.cos(angle)*y[a] for a,b in enumerate(x)]
    
    def mean(self,x):
        sum=0
        for i in range(len(x)):
            sum+=x[i]
        return(sum/len(x))
    def scale_by_factor(self,factor,x,y):
        for i in range(len(x)):
            x[i]=x[i]*factor
            y[i]=y[i]*factor
        return x,y
    def calculate_scale(self,x,y):
        sum=0
        for i in range(len(x)):
            sum+=x[i]*x[i]+y[i]*y[i]
        return math.sqrt(sum)/len(x)
    def rotate_derivative_one_term(self,x,y,a,w,z):
        x_der=2.0*((math.cos(a)*(w)-math.sin(a)*(z))-x)*(-math.sin(a)*(w)-math.cos(a)*(z))
        y_der=2.0*((math.sin(a)*(w)+math.cos(a)*(z))-y)*(math.cos(a)*(w)-math.sin(a)*(z))

        return x_der+y_der
    def equation(self,x,y,w,z,a):
        xeq=0
        yeq=0
        for i in range(len(x)):
            try:
                xeq+=math.pow((math.cos(a)*w[i]-math.sin(a)*z[i]-x[i]),2)
                yeq+=math.pow((math.sin(a)*w[i]+math.cos(a)*z[i]-y[i]),2)
            except OverflowError:
                return 10000000
        return(xeq+yeq)
    def derivative(self,x,y,a,w,z):
        der=0
        for i in range(len(x)):
            der+=self.rotate_derivative_one_term(x[i],y[i],a,w[i],z[i])
        return(der)
    #I'm dumb this doesn't use Newton's method anymore it just iterates
    def newtons_method(self,x,y,w,z):
        angle=-math.pi
        res=100
        best_angle=0
        min_equation=1000
        for i in range(res):
            eq=self.equation(x,y,w,z,angle)
            if abs(eq)<min_equation:
                min_equation=abs(eq)
                best_angle=angle
            angle+=math.pi/float(res)*2.0
        x_val=best_angle
        
        return(x_val)
    def get_x_intercept(self,slope,y_intercept):
        return(-y_intercept/slope)
    def shift_points(self,x1,y1):
        x=copy.deepcopy(x1)
        y=copy.deepcopy(y1)
        sumx=0
        sumy=0
        for i in range(len(x)):
            sumx+=x[i]
            sumy+=y[i]
        mean=(sumx/len(x),sumy/len(y))
        for i in range(len(x)):
            x[i]-=mean[0]
            y[i]-=mean[1]
        return x,y
