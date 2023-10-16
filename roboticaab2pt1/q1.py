import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH

# Encontrar a configuração das juntas
def inverse_kinematics_RRR(x, y, z, L1=1, L2=1):
    Co = (y**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(Co) > 1:
        print("-1 < Cos theta > 1")
        return None

    So = m.sqrt(1 - Co**2)
    o3 = m.atan2(So, Co)

    So1 = -m.sqrt(1 - Co**2)
    o31 = m.atan2(So1, Co)

    b = m.atan2(z, y)

    Cp = (y**2 + z**2 + L1**2 - L2**2) / (2 * L1 * m.sqrt(y**2 + z**2))
    if abs(Cp) > 1:
        print("-1 < Cos phi > 1")
        return None

    Sp = m.sqrt(1 - Cp**2)
    p = m.atan2(Sp, Cp)

    if o3 > 0:
        o2 = b - p
        o21 = b + p
    else:
        o2 = b + p
        o21 = b - p

    o1 = m.atan2(y / m.sqrt(x**2 + y**2), x / m.sqrt(x**2 + y**2))

    print("Possíveis soluções:")
    if abs(o1 - m.pi / 2) > m.pi / 2 or abs(o2) > m.pi / 2 or abs(o3 + m.pi / 2) > m.pi / 2:
        print('θ1=', o1 - m.pi / 2, 'θ2=', o2, 'θ3=', o3 + m.pi / 2)
        q = [o1 - m.pi / 2, o21, o31 + m.pi / 2]
        robot_RRR(q=q)
        if abs(o1) > m.pi / 2 or abs(o21) > m.pi / 2 or abs(o31) > m.pi / 2:
            print('θ1=', o1 - m.pi / 2, 'θ2=', o21, 'θ3=', o31 + m.pi / 2)
            q = [o1 - m.pi / 2, o2, o3 + m.pi / 2]
            robot_RRR(q=q)
    else:
        print("Não foi encontrada uma solução dentro do intervalo")
        return None

# Definir o robô RRR
def robot_RRR(q=[0, 0, 0], L1=1, L2=1):

    e1 = RevoluteDH(d=0, alpha=m.pi / 2, offset=m.pi / 2)
    e2 = RevoluteDH(a=L1)
    e3 = RevoluteDH(a=L2, offset=-m.pi / 2)

    rob = DHRobot([e1, e2, e3], name='RRR')
    rob.teach(q)
    return rob

# Plotar o espaço de trabalho
def workspace(L1=1, L2=1):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    theta = np.linspace(0, np.pi, 100)
    theta2 = np.linspace(-np.pi/2, np.pi, 100)
    r = L1
    r2 = L1 + L2

    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.zeros_like(theta)
    ax.plot(x, y, z, label='Junta 1')

    x1 = r2 * np.cos(theta2)
    y1 = r2 * np.sin(theta2)
    z1 = np.zeros_like(theta2)
    ax.plot(x1, y1, z1, label='Junta 2')

    ax.axis('equal')
    ax.set_xlabel('Eixo X')
    ax.set_ylabel('Eixo Y')
    ax.set_zlabel('Eixo Z')

    ax.legend()
    plt.show()

    q1_values = np.linspace(0, np.pi, 100)
    q2_values = np.linspace(-np.pi/2, np.pi, 100)

    x_coords = []
    y_coords = []

    for q1 in q1_values:
        for q2 in q2_values:
            x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
            y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
            x_coords.append(x)
            y_coords.append(y)

    plt.figure(figsize=(8, 6))
    plt.scatter(x_coords, y_coords, s=1, c='b', marker='.')
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.title('Espaço de Trabalho do Manipulador RR Planar')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Função principal
def main():
    rob = robot_RRR()

    inverse_kinematics_RRR(x=0, y=1, z=0)
    print(rob.ikine_LM(transl(x=0, y=1, z=0)))

    rob = robot_RRR(q=[-0.1495, 0.9527, -0.6337])

    inverse_kinematics_RRR(x=0, y=0.5, z=-0.5)
    print(rob.ikine_LM(transl(x=0, y=0.5, z=-0.5)))

    rob = robot_RRR(q=[-1.368, -2.779, 4.182])

if __name__ == "__main__":
    main()
