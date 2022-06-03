"""
Filename: Visual_Rep_SMMV_RTB.py
Created on Wednesday 25/05/2022
Title: Visual Representation of Spherical Manipulator - MV - Robotics Toolbox
Author: Aaron Joshua M. Apolonia
Team: Group 12-Block C
a1 = 10mm
a2 = 40mm
a3 = 40mm
"""

import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import matplotlib.pyplot as plt

a1 = float(100) 
a2 = float(400)  
a3 = float(400) 

def mm_to_meter(a):
    m = 1000 
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)

lm = float(400) 
lm = mm_to_meter(lm)

Sphe_MV = DHRobot([
    RevoluteDH(a1,0,(90/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    RevoluteDH(0,a2,(90/180)*np.pi,(90/180)*np.pi,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    PrismaticDH(0,0,0,a3,qlim=[0,lm]),
], name='Spherical Manipulator MV')

print(Sphe_MV)

Sphe_MV.teach(jointlabels=1)