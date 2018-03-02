#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 22:35:58 2017

@author: shane
"""

from sympy import *
import tf

def R_x(q):
    M_x = Matrix([[ 1,      0,       0],
                  [ 0, cos(q), -sin(q)],
                  [ 0, sin(q),  cos(q)]])
    return M_x

def R_y(q):
    M_y = Matrix([[ cos(q), 0, sin(q)],
                  [      0, 1,       0],
                  [-sin(q), 0, cos(q)]])
    return M_y

def R_z(q):
    M_z = Matrix([[ cos(q), -sin(q), 0],
                  [ sin(q),  cos(q), 0],
                  [ 0,            0, 1]])
    return M_z

def transMat(q, alpha, d, a):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                0,           0,             1]])
    return T

### Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

### Populate fixed DH parameter table
s = {alpha0:     0,    a0:      0,    d1:  0.75,
     alpha1: -pi/2,    a1:   0.35,    d2:     0,    q2: (q2 - pi/2),
     alpha2:     0,    a2:   1.25,    d3:     0,
     alpha3: -pi/2,    a3: -0.054,    d4:   1.5,
     alpha4:  pi/2,    a4:      0,    d5:     0,
     alpha5: -pi/2,    a5:      0,    d6:     0,
     alpha6:     0,    a6:      0,    d7: 0.303,    q7: 0}

### Create homogeneous transfroms between neighbouring links
T0_1 = transMat(q1, alpha0, d1, a0)
T1_2 = transMat(q2, alpha1, d2, a1)
T2_3 = transMat(q3, alpha2, d3, a2)
T3_4 = transMat(q4, alpha3, d4, a3)
T4_5 = transMat(q5, alpha4, d5, a4)
T5_6 = transMat(q6, alpha5, d6, a5)
T6_7 = transMat(q7, alpha6, d7, a6)

### Substitute DH parameters into the homogeneous transforms
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_7 = T6_7.subs(s)

### Incrementally build the transform from the base link to the
### gripper link (intrinsic rotation = post-mult)
T0_2 = T0_1*T1_2
T0_3 = T0_2*T2_3
T0_4 = T0_3*T3_4
T0_5 = T0_4*T4_5
T0_6 = T0_5*T5_6
T0_7 = T0_6*T6_7

### Create the correction matrrix used to transform from urdf to DH
R_Z = Matrix([[-1,  0, 0, 0],
              [ 0, -1, 0, 0],
              [ 0,  0, 1, 0],
              [ 0,  0, 0, 1]])

R_Y = Matrix([[0, 0, -1, 0],
              [0, 1,  0, 0],
              [1, 0,  0, 0],
              [0, 0,  0, 1]])

T_corr = R_Z*R_Y # intrinsic rotation = post

### Numerically evaluate transforms
s_dict_1 = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0} #test_initial
s_dict_2 = {q1: -0.65, q2: 0.45, q3: -0.37, q4: 0.96, q5: 0.78, q6: 0.46} #test_1
s_dict_3 = {q1: -0.79, q2: -0.11, q3: -2.34, q4: 1.96, q5: 1.14, q6: -3.69} #test_2
s_dict_4 = {q1: -2.99, q2: -0.12, q3: 0.94, q4: 4.06, q5: 1.29, q6: -4.15} #test_3

### Print out the results for the test cases 1 to 4
T_total = T0_7*T_corr

print('\nT0_7_corrected test case 1 =\n')
pprint(T_total.evalf(subs=s_dict_1))

print('\nT0_7_corrected test case 2 =\n')
pprint(T_total.evalf(subs=s_dict_2))

print('\nT0_7_corrected test case 3 =\n')
pprint(T_total.evalf(subs=s_dict_3))

print('\nT0_7_corrected test case 4 =\n')
pprint(T_total.evalf(subs=s_dict_4))