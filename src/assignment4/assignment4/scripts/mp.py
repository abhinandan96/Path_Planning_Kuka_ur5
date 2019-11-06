#!/usr/bin/env python

from __future__ import division
import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    def closest_point(self,q,joint_list):
        dist = []
        for i in range (len(joint_list)):
            distance = 0
            for j in range (self.num_joints):
                distance += (q[j][0] - joint_list[i][0][j])**2
            distance = distance**0.5
            dist.append([distance,joint_list[i][0]])
        closest_point = min(dist)[1]
        return closest_point

    def magnitude(self,a,b):
        # print(a,b)
        distance = 0
        for j in range(self.num_joints):
            distance += (a[j] - b[j])**2
        distance = distance ** 0.5
        return distance

    def new_point(self,q,close_joint):
        distance = self.magnitude(q,close_joint)
        k = 0.1
        newq = numpy.zeros((self.num_joints,1))
        newq1 = []
        for j in range (self.num_joints):
            newq[j] = q[j] + (q[j] - close_joint[j])*k/distance
            newq1.append(newq[j][0])
        return newq1

    def check_path(self,newq,close_pt):
        k = 0.05
        close_pt1 = []
        if len(newq)==1:
            newq = newq[0]
        vector = []
        # print(newq)
        # print(self.num_joints)
        # print(close_pt)
        for j in range(self.num_joints):
            # print(j)
            close_pt1.append(close_pt[j])
        vector = numpy.subtract(newq,close_pt)
        maxd = int(max(abs(vector))/0.03)
        unitd = vector/maxd
        checkpath = []
        check = []
        close_pt = numpy.array(close_pt)
        for i in range (maxd+1):
            check = close_pt + i * unitd
            checkpath.append(check)
        for path in checkpath:
            if self.is_state_valid(path) == False:
                return False
        return True

    def find_path(self,joint_list):
        L = len(joint_list)
        path = []
        path.append(joint_list[L-1][0])
        for i in range(L-1,0,-1):
            l = len(path)
            if joint_list[i][0] == path[l-1]:
                path.append(joint_list[i][1])
        path.append(joint_list[0][0])
        path.reverse()
        return  path

    def optimum_path(self, path_list):
        opt_path = []
        opt_path.append((path_list[0]))
        i = 0
        while i < len(path_list):
            l = len(opt_path)
            if opt_path[l - 1] != path_list[i]:
                if self.check_path(opt_path[l - 1], path_list[i]) == False:
                    opt_path.append(path_list[i - 1])
                else:
                    i += 1
            else:
                i += 1
        opt_path.append(path_list[len(path_list) - 1])
        return opt_path

    def discretize_path(self,opt_q_list):
        discrete_path = []
        for i  in range (len(opt_q_list)-1):
            vector = numpy.subtract(opt_q_list[i+1],opt_q_list[i])
            maxd = int(max(abs(vector)) / 0.03)
            unitd = vector / maxd
            disc = []
            for j in range(maxd + 1):
                disc = opt_q_list[i] + j * unitd
                discrete_path.append(disc)
        return discrete_path

    def trajectory(self, discrete_q):
        trajectory = JointTrajectory()
        trajectory.points=[]
        for i in range(0, len(discrete_q)):
            joint = JointTrajectoryPoint()
            joint.positions = list(discrete_q[i])
            trajectory.points.append(joint)
            trajectory.joint_names = self.joint_names
            self.pub.publish(trajectory)

    def IK_ur5(self,b_T_edes):
        self.mutex.acquire()
        # --------------------------------------------------------------------------
        # FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
        l = self.num_joints
        joint_values = numpy.zeros((l, 1))
        for i in range(l):
            joint_values[i] = random.randint(0, 1000) * 0.001
        t = 0
        a = numpy.zeros((3, 1))
        b = numpy.zeros((3, 1))
        c = 0
        c = time.time()
        while t < 5:
            joint_transforms, b_T_ee = self.forward_kinematics(joint_values)
            ee_T_b = tf.transformations.inverse_matrix(b_T_ee)
            ee_T_eed = numpy.dot(ee_T_b, b_T_edes)
            a[:, 0] = ee_T_eed[0:3, 3]
            angle, axis = self.rotation_from_matrix(ee_T_eed)
            b[:, 0] = numpy.dot(axis, angle)
            delx = numpy.concatenate((a, b), axis=0)
            # print(b,delx)
            J = self.get_jacobian(b_T_ee, joint_transforms)
            Jp = numpy.linalg.pinv(J, 0.01)
            delq = numpy.dot(Jp, delx)
            joint_values = joint_values + delq
            d = time.time()
            t = d - c
        # --------------------------------------------------------------------------
        self.mutex.release()
        return joint_values

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times
    in the motion planning process and it will be easiest to make this a
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish,
    which will also be called from the motion planning function.'''

    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
        q = numpy.zeros((self.num_joints,1))
        ## Calculate T_goal
        T_goal = tf.transformations.quaternion_matrix([ee_goal.rotation.x, ee_goal.rotation.y
                                                            , ee_goal.rotation.z, ee_goal.rotation.w])
        T_goal[0, 3] = ee_goal.translation.x
        T_goal[1, 3] = ee_goal.translation.y
        T_goal[2, 3] = ee_goal.translation.z
        # if self.num_joints == 6:
        #     q_goal = self.IK(T_goal)
        # else:
        q_goal = self.IK(T_goal)
        finalq = []
        for i in range (len(q_goal)):
            finalq.append(q_goal[i])
        joint_list = []
        a = self.q_current
        joint_list.append([a,a])
        # k = 0
        while True:
            for i in range (self.num_joints):
                q[i] = random.uniform(-numpy.pi,numpy.pi)
            close_pt = self.closest_point(q,joint_list)
            newq = self.new_point(q,close_pt)
            if self.check_path(newq,close_pt) == True:
                joint_list.append([newq,close_pt])
                if self.check_path(newq,finalq) == True:
                    joint_list.append([finalq,newq])
                    break
        path_list = self.find_path(joint_list)
        opt_q_list = self.optimum_path(path_list)
        discrete_q = self.discretize_path(opt_q_list)
        print("Nodes Expanded:",len(joint_list))
        print("Nodes after smoothening:", len(opt_q_list))
        print("Number of discrete nodes:", len(discrete_q))
        self.trajectory(discrete_q)
    ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the
    joints of the robot arm, ordered from proximal to distal. If no IK solution
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal.
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list
(your 'tree'). The class has a parent field and a joint position field (q).
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()