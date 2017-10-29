from sympy import *
from time import time
from mpmath import radians
import tf
'''Formt of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

def R_x(q):
    M_x = Matrix([[ 1,      0,       0],
                  [ 0, cos(q), -sin(q)],
                  [ 0, sin(q),  cos(q)]])
    return M_x

def R_y(q):
    M_y = Matrix([[ cos(q), 0,  sin(q)],
                  [      0, 1,       0],
                  [-sin(q), 0, cos(q)]])
    return M_y

def R_z(q):
    M_z = Matrix([[ cos(q), -sin(q), 0],
                  [ sin(q),  cos(q), 0],
                  [      0,       0, 1]])
    return M_z

def transMat(q, alpha, d, a):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                0,           0,             1]])
    return T

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##

    ## Insert IK code here!

    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    # Create Modified DH parameters
    s = {alpha0:     0,    a0:      0,    d1: 0.75,
         alpha1: -pi/2,    a1:   0.35,    d2:    0,    q2: (q2 - pi/2),
         alpha2:     0,    a2:   1.25,    d3:    0}

    # Define Modified DH Transformation matrix
    T0_1 = transMat(q1, alpha0, d1, a0)
    T1_2 = transMat(q2, alpha1, d2, a1)
    T2_3 = transMat(q3, alpha2, d3, a2)

    T0_1 = T0_1.subs(s)
    T1_2 = T1_2.subs(s)
    T2_3 = T2_3.subs(s)

    # Create individual transformation matrices
    T0_2 = T0_1*T1_2
    T0_3 = T0_2*T2_3

    pprint(R_z(pi)*R_y(-pi/2))
    R_corr = Matrix([[0,  0, 1],
                     [0, -1, 0],
                     [1,  0, 0]])

    # Extract rotation matrices from the transformation matrices
    R0_3 = T0_3[0:3,0:3] # this expression is fine

    Rrpy = R_z(yaw) * R_y(pitch) * R_x(roll) * R_corr # this expression is fine

    wx = px - (0.303) * Rrpy[0,2] # x-coord of wrist position
    wy = py - (0.303) * Rrpy[1,2] # y-coord of wrist position
    wz = pz - (0.303) * Rrpy[2,2] # z-coord of wrist position

    r = sqrt(wx**2 + wy**2) - 0.35 #implemented okay
    ss = wz - 0.75 #implemented okay

    k1 = 1.25 #implemented okay
    k2 = 1.5 #implemented okay

    D = (k1**2 + k2**2 - (r**2 + ss**2))/(2 * k1 * k2) #implemented okay
    K = (k1**2 + (r**2 + ss**2) - k2**2)/(2*sqrt(r**2 + ss**2)*k1) #implemented okay

    # First three joint variables
    theta1 = atan2(wy,wx).evalf()
    theta2 = (pi/2 - atan2(ss,r) - atan2(sqrt(1 - K**2), K)).evalf()
    theta3 = 1.53484 - atan2(sqrt(1 - D**2), D).evalf()

    R36rpy = (R0_3.transpose() * Rrpy).evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    # Second three joint variables

    theta5 = atan2(sqrt(1 - R36rpy[1,2]**2), R36rpy[1,2]).evalf()
    theta4 = atan2(R36rpy[2,2], -R36rpy[0,2]).evalf()
    theta6 = atan2(-R36rpy[1,1], R36rpy[1,0]).evalf()
    print(theta1)
    print(theta2)
    print(theta3)
    print(theta4)
    print(theta5)
    print(theta6)
    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
