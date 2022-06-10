import numpy as np
import modern_robotics as mr
from UR5_parameter import *

dt = 0.1 #timestep
intRes = 200 # integer Euler integration steps
thetalist1 = [0,0,0,0,0,0] #simulation 1 zero configuration
thetalist2 = [0,-1,0,0,0,0] #simulation 2 -1rad joint2
dthetalist = [0,0,0,0,0,0] #initial joint velocities
taumat = np.zeros((200,6 )) #joint/forces matrix
g =  np.array([0, 0, -9.8]) #gravity
Ftipmat = np.zeros((200,6 )) #tip forces

#trajectory for simulation1
thetamat1,dthetamat1 \
        = mr.ForwardDynamicsTrajectory(thetalist1, dthetalist, taumat, g, \
                                       Ftipmat, Mlist, Glist, Slist, dt, \
                                       intRes)
#trjectory for simulation2
thetamat2,dthetamat2 \
        = mr.ForwardDynamicsTrajectory(thetalist2, dthetalist, taumat, g, \
                                       Ftipmat, Mlist, Glist, Slist, dt, \
                                       intRes)

print("thetamat1:",thetamat1,end='\n')
print("thetamat2:",thetamat2,end='\n')

#saved cvs files
np.savetxt('csv/simulation1.csv', thetamat1, delimiter=',', fmt='%f')
np.savetxt('csv/simulation2.csv', thetamat2, delimiter=',', fmt='%f')
