"""
Filename: IK_SMMV_RTB.py
Created on Wednesday 25/05/2022
Title: Inverse of Spherical Manipulator - MV - Robotics Toolbox
Author: Aaron Joshua M. Apolonia
Team: Group 12-Block C
a1 = 10mm
a2 = 40mm
a3 = 40mm
X = 62.831mm
Y = 62.831mm 
Z = 20.226mm
"""

import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3

a1 = float(10) 
a2 = float(40) 
a3 = float(40) 

def mm_to_meter(a):
    m = 1000
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)
lm = float(40)
lm = mm_to_meter(lm)

Sphe_MV = DHRobot([
    RevoluteDH(a1,0,(90/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    RevoluteDH(0,a2,(90/180)*np.pi,(90/180)*np.pi,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    PrismaticDH(0,0,0,a3,qlim=[0,lm]),
], name='Spherical Manipulator MV')

# Inverse Kinematics based on Forward Kinematics
T1 = float(45) 
T2 = float(-20) 
d3 = float(40) 
T1 = (T1/180.0)*np.pi 
T2 = (T2/180.0)*np.pi 
FK = Sphe_MV.fkine([T1,T2,d3])
IK = Sphe_MV.ikine_min(FK)
print('IK = ')
print(IK)

"""
# Inverse Kinematics based on Position Vector
T= SE3()
IKine = Sphe_Modern.ikine_LM(T)
print('IKine = ')
print(IKine)
"""