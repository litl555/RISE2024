import numpy as np
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