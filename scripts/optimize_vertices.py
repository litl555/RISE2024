from gtsam import *
import numpy as np
import copy
import matplotlib.pyplot as plt
#print(help(gtsam.Pose3))
class optimize_verts():
    def __init__(self):
        self.verts_ordered=list(np.load("/home/clearpath/jackal_ws/src/apriltag_mapping/verts_viewing_order.npy"))
        self.ids_ordered_viewing=list(np.load("/home/clearpath/jackal_ws/src/apriltag_mapping/ids_viewing_order.npy"))
        self.ids_ordered=list(np.load("/home/clearpath/jackal_ws/src/apriltag_mapping/ids.npy"))

        self.ids_ordered_viewing=self.ids_ordered_viewing[:len(self.ids_ordered)]

    def create_factor_graph(self,v):
        
        noise=noiseModel.Diagonal.Sigmas(v)
        factor_graph=NonlinearFactorGraph()
        factor_graph.add(PriorFactorPoint3(1, self.verts_ordered[0],noise))
        x=self.verts_ordered[0][0]
        y=self.verts_ordered[0][2]
        index=1
        for i in range(len(self.verts_ordered)-1):
            edge_vector=self.get_edge(self.verts_ordered[i],self.verts_ordered[i+1])
            x+=edge_vector[0]
            y+=edge_vector[2]
            if index==len(self.verts_ordered)-1:
                factor=BetweenFactorPoint3(index,1,edge_vector,noise)

            else:
                factor=BetweenFactorPoint3(index,index+1,edge_vector,noise)
                #print("generating factor between "+str(self.ids_ordered_viewing[i])+" and " + str(self.ids_ordered_viewing[i+1]))
            factor_graph.add(factor)
            
            index+=1
        print(factor_graph)
        print(x)
        print(y)
        return factor_graph
    def generate_values(self):
        values=Values()
        for ind,vert in enumerate(self.verts_ordered[:len(self.verts_ordered)-1]):
            values.insert(ind+1,vert)
        return values
    def get_edge(self,start,end):
        return([end[0]-start[0],end[1]-start[1],end[2]-start[2]])
    def get_optimized_points(self,factor_graph,values):
        optimized=LevenbergMarquardtOptimizer(factor_graph,values).optimize()
        vertices=[]
        for i in range(len(self.verts_ordered)-1):

            vertices.append(list(optimized.atPoint3(i+1)))
        #print(factor_graph.error(values))
        return vertices,factor_graph.error(values)
    def get_ordered_optimized_vertices(self,v):
        f=self.create_factor_graph(v)
        values=self.generate_values()
        vertices_optimized,score=self.get_optimized_points(f,values)
        
        vert_list_ordered=[None]*(len(self.verts_ordered)-1)
        #print(vertices_optimized)
        for index,i in enumerate(self.ids_ordered):
            
            for index_ordered,x in enumerate(self.ids_ordered_viewing):
                
                if str(x)==str(i):

                    vert_list_ordered[index]=vertices_optimized[index_ordered]
        return(vert_list_ordered,score)

    def get_best_matrix(self):
        best_matrix=[]
        min_score=10000
        for x in range(100):
            for y in range(1):
                for z in range(100):
                    v=optimize_verts()
                    _,score=v.get_ordered_optimized_vertices([float(x)/10.0,float(y)/10.0,float(z)/10.0])
                    if score<min_score:
                        min_score=copy.deepcopy(score)
                        best_matrix=copy.deepcopy([float(x)/10.0,float(y)/10.0,float(z)/10.0])
            
        print("best matrix")
        print(best_matrix)
        print(min_score)
    def plot(self,verts):
        x=[]
        y=[]
        x1=[]
        y1=[]
        
        self.verts_ordered=[list(self.verts_ordered[i]) for i in range(len(self.verts_ordered))]
        print(self.verts_ordered)
        for i in self.verts_ordered:
            print(i)
            x.append(i[0])
            y.append(i[2])
        for i in verts:
            
            x1.append(i[0])
            y1.append(i[2])
        print(x)
        #x.pop(len(x)-1)
        #y.pop(len(y)-1)
        x.append(x[0])
        y.append(y[0])
        x1.append(x1[0])
        y1.append(y1[0])
        fig=plt.figure()
        
        plt.plot(x,y,color="blue",label="Before Map Correction")
        plt.plot(x1,y1,color="red",label="After Map Correction")
        plt.grid()
        fig.legend()
        plt.savefig("/home/clearpath/jackal_ws/src/apriltag_mapping/plots/map_correction.png",bbox_inches="tight")
        plt.show()
v=optimize_verts()
verts,score=v.get_ordered_optimized_vertices([.3,0.0,.3])
print(score)
v.plot(verts)