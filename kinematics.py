import math

import numpy

D0 = 60
L = 125
LG = 75


# takes in joint positions array, returns homogenous transformation matrix to EE
def FK(q_array):
    q = [math.radians(q_array[0]), math.radians(q_array[1]), math.radians(q_array[2])]

    T1 = [[1, 0, 0, 0],
          [0, 1, 0, 0],
          [0, 0, 1, D0],
          [0, 0, 0, 1]]

    T2 = [[1, 0, 0, 0],
          [0, 0, -1, 0],
          [0, 1, 0, q[0]],
          [0, 0, 0, 1]]

    T3 = [[math.cos(q[1] + math.pi/2), 0, math.sin(q[1] + math.pi/2), 0],
          [math.sin(q[1] + math.pi/2), 0, -(math.cos(q[1] + math.pi/2)), 0],
          [0, 1, 0, L],
          [0, 0, 0, 1]]

    T4 = [[math.cos(q[2] + math.pi/2), -(math.sin(q[2] + math.pi/2)), 0, LG*math.cos(q[2] + math.pi/2)],
          [math.sin(q[2] + math.pi/2),   math.cos(q[2] + math.pi/2),  0, LG*math.sin(q[2] + math.pi/2)],
          [0, 0, 1, 0],
          [0, 0, 0, 1]]

    # T0e = T1*T2*T3*T4
    T02 = numpy.matmul(T1,T2)
    T03 = numpy.matmul(T02,T3)
    T0e = numpy.matmul(T03,T4)

    return T0e


# takes in EE position, returns joint positions array
def IK(o):
    q = []
    oX = o[0]
    oY = o[1]
    oZ = o[2]

    q[2] = math.asin((oY+L)/LG) - math.pi/2

    a = math.fabs(q[2])

    while a > math.pi:
        a = a - math.pi

    if -0.001 < a < 0.001:
        q[1] = 0
    else:
        q[1] = math.acos(oX/(LG*math.cos(q[2] +math.pi/2))) - math.pi/2
    q[0] = oZ - D0 - LG*math.sin(q[1] +math.pi/2)*math.cos(q[2] +math.pi/2)

    q = [math.degrees(q[0]), math.degrees(q[1]), math.degrees(q[2])]
    return q


# takes in joint pos and vel arrays, returns EE velocity
def velocity_FK(q, q_dot):
    # J(1,:) = [0, -lg*cos(q3 + pi/2)*sin(q2 + pi/2), -lg*cos(q2 + pi/2)*sin(q3 + pi/2)];
    # J(2,:) = [0, 0, -lg*cos(q3 + pi/2)];
    # J(3,:) = [1, lg*cos(q2 + pi/2)*cos(q3 + pi/2), -lg*sin(q2 + pi/2)*sin(q3 + pi/2)];
    # J(4,:) = [0,  0, sin(q2 + pi/2)];
    # J(5,:) = [0, -1, 0];
    # J(6,:) = [0,  0, -cos(q2 + pi/2)];

    jacobian = [[0, -LG*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2), -LG*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)],
                [0, 0, -LG*math.cos(q[2] + math.pi/2)],
                [1, LG*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2), -LG*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)],
                [0,  0, math.sin(q[1] + math.pi/2)],
                [0, -1, 0],
                [0,  0, -math.cos(q[1] + math.pi/2)]]
    e_vel = numpy.matmul(jacobian, numpy.transpose(q_dot))

    return e_vel

# takes in joint pos and EE vel arrays, returns joint velocities
def velocity_IK(q, e_vel):
    jacobian = [[0, -LG*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2), -LG*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)],
                [0, 0, -LG*math.cos(q[2] + math.pi/2)],
                [1, LG*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2), -LG*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)],
                [0,  0, math.sin(q[1] + math.pi/2)],
                [0, -1, 0],
                [0,  0, -math.cos(q[1] + math.pi/2)]]

    val = numpy.linalg.lstsq(jacobian, numpy.transpose(e_vel))
    q_dot = numpy.linalg.pinv(val)

    return q_dot





