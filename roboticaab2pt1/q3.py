import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH

PLOT = True
PI = np.pi

def inverse_kinematics_Scara(x, y, z, L1=1, L2=1, D2=0.2, D4=0.2):
    Co = (y**2 + x**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(Co) > 1:
        print("-1 < Cos theta > 1")
        return None

    So = m.sqrt(1 - Co**2)  # Positive root
    o2 = m.atan2(So, Co)

    So = -m.sqrt(1 - Co**2)  # Negative root
    o21 = m.atan2(So, Co)

    b = m.atan2(y, x)
    r = m.sqrt(y**2 + x**2)

    if r == 0:
        Cp = 0
    else:
        Cp = (y**2 + x**2 + L1**2 - L2**2) / (2 * L1 * r)

    if abs(Cp) > 1:
        print("-1 < Cos phi > 1")
        return None

    Sp = m.sqrt(1 - Cp**2)
    p = m.atan2(Sp, Cp)

    if o2 > 0:
        o1 = b - p
        o11 = b + p
    else:
        o1 = b + p
        o11 = b - p

    d3 = -z - D4 + D2

    o4 = 0

    print("Possible solutions:")

    print('θ1=', o1, 'θ2=', o2, 'D3=', d3, 'θ4=', o4)
    q = [o1, o2, d3, o4]
    robot_Scara(q=q)

    print('θ1=', o11, 'θ2=', o21, 'D3=', d3, 'θ4=', o4)
    q = [o11, o21, d3, o4]
    robot_Scara(q=q)


def robot_Scara(q=[0, 0, 0.5, 0], L1=1, L2=1, D1=0.2, D3=[0, 1], D4=0.2):
    e1 = RevoluteDH(a=L1, d=D1)
    e2 = RevoluteDH(a=L2, alpha=PI)
    e3 = PrismaticDH(qlim=D3)
    e4 = RevoluteDH(d=D4)
    rob = DHRobot([e1, e2, e3, e4], name='RRPR')

    rob.teach(q)


def main():
    inverse_kinematics_Scara(x=1, y=1, z=-1)
    inverse_kinematics_Scara(x=0, y=0.5, z=-1)
    inverse_kinematics_Scara(x=0.5, y=0, z=-0.5)
