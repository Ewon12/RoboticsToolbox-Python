"""
Filename: FK_SMMV_RTB.py
Created on Wednesday 25/05/2022
Title: Forward Kinematics of Spherical Manipulator - MV - Robotics Toolbox
Author: Aaron Joshua M. Apolonia
Team: Group 12-Block C
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

J = Sphe_MV.jacob0([(float(45)/180.0)*np.pi,(float(-20)/180.0)*np.pi,float(400)])
print('J = ')
print(np.around(J,3))

JS = J[0:3,0:]
Det_J = np.linalg.det(JS)
print('Det(J) = ',Det_J)

Inv_J = np.linalg.inv(JS)
print('Inv = ')
print(np.around(Inv_J,3))

Transpose_J = np.transpose(J)
print('Transpose_J = ')
print(np.around(Transpose_J,3))