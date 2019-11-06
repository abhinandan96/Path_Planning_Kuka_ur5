#!/usr/bin/env python

import numpy
import random
import sys
import time
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
import time
import math
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
	self.pub_msg = JointTrajectory()
	

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
	#print(self.q_current)

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data
	#rospy.loginfo(self.current_obstacle)

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
	while True:	
		command = ee_goal
		goal_trans = numpy.dot(tf.transformations.translation_matrix((command.translation.x, command.translation.y, command.translation.z)), tf.transformations.quaternion_matrix([command.rotation.x, command.rotation.y, command.rotation.z, command.rotation.w]))		
		goal_node = []	
		goal_node = self.IK(goal_trans)	
		self.goal = self.IK(goal_trans)
		if self.is_state_valid(goal_node):
			break
		else:
			print("No IK")
	start_node = []
	parent = []
	child = []		
	start_node = self.q_current
	self.treelen = 2
	self.fin_seg = 0.45
	self.obs_seg = 0.05	
	parent.append(None)
	child.append(start_node)
	current_node = start_node
	timer = self.obstacle_time(self.current_obstacle)
	while True:
		val, res = self.is_segment_valid(current_node, goal_node)
		if res == True:
			break
		print("Node:", len(child))
		random_node = self.random_q()			
		#print("random_node:", random_node)
		near_node, nindex = self.close_node(parent, child, random_node)
		#print("near_node:", near_node)
		#print("Distrandtonear:", self.dist(near_node, random_node))		
		if self.dist(near_node, random_node) < self.treelen:
			new_node = random_node
		else:
			new_node = self.new_int_node(near_node,random_node)
			#new_node = self.new_node_func(near_node,random_node,0.1)	
		new_node, res = self.is_segment_valid(near_node, new_node)
		#print(res)
		#print("new_node:", new_node)
		#print("Distbetnearnew:", self.dist(near_node, new_node))
		parent.append(nindex)
    		child.append(new_node)
    		#print("Parent:", nindex)
   		#print("Child:", new_node)
    		current_node = new_node
    		#print("Dist:", self.dist(current_node, goal_node))

	print("Done adding nodes")
	#print(len(child))	
	child.append(goal_node)			
	parent.append(len(child) - 2)
	
	route_rough = []    
	ind = len(child) - 1
	#print(child)
	#print(parent)
	while True:
    		route_rough.append(child[ind])
    		ind = parent[ind]		
    		if ind == None:
	        	break

	#print(route_rough)
	route_rev = []
	for i in range(0,len(route_rough)):
		route_rev.append(route_rough[(len(route_rough) - 1) - i]) 
	#print("route_rev:", route_rev)
	print("Number of nodes in new route:", len(route_rev))
	print("Smoothning the route")
	#print(len(route_rev))
	node = self.reroute(route_rev)
	#print("node:", node)
	print("Number of nodes in new route:", len(node))
	print("Segmenting for the final path")
	#node = route_rev
	route = self.final_route(node)	
	#print(route)

	self.pub_msg.points = []
	for i in range(0, len(route)):
		traj = []
		node = route[i]
		pub_point_msg = JointTrajectoryPoint()
		for j in range(0, self.num_joints):
			traj.append(node[j])
			#print(traj)
		for j in range(0, self.num_joints):
			pub_point_msg.positions.append(traj[j])
		self.pub_msg.points.append(pub_point_msg)

	self.pub_msg.joint_names = []
	for i in range(0,self.num_joints):	
		self.pub_msg.joint_names.append(self.joint_names[i])

	self.pub.publish(self.pub_msg)	
	#print("start_node:",start_node)
	#print("goal_node:", goal_node)
	print("Done")
        ######################################################
	
    def random_q(self):
	self.qc = []
	for i in range(0,self.num_joints):
		self.qc.append(random.uniform(-numpy.pi,numpy.pi))
	return self.qc

    def dist(self, a, b):
	d = []
	dist = 0
	for i in range(0,self.num_joints):
		d.append((a[i] - b[i]) * (a[i] - b[i]))
	for i in range(0,self.num_joints):
		dist = dist + d[i]
	dist = dist ** 0.5
	return dist

    def dist_heur(self, a, b):
	d = []
	dist = 0
	for i in range(0,self.num_joints):
		d.append(((a[i] - b[i]) * (a[i] - b[i])) + ((self.goal[i] - b[i]) * (self.goal[i] - b[i])))
	for i in range(0,self.num_joints):
		dist = dist + d[i]
	dist = dist ** 0.5
	return dist

    def close_node(self, parent, child, a):
	mindist = numpy.zeros((len(child), 2))
	for i in range(0, len(child)):
		mindist[i,0] = self.dist_heur(a,child[i])
		mindist[i,1] = i
	for i in range(0, len(child)):
		for j in range(1, len(child)):
			if mindist[j,0] <= mindist[j - 1,0]:
				mindist[j-1,0], mindist[j,0] = mindist[j,0], mindist[j-1,0]
				mindist[j-1,1], mindist[j,1] = mindist[j,1], mindist[j-1,1]
	
	return child[int(mindist[0,1])], int(mindist[0,1])
    
    def new_node_func(self,a,b,n):
	nv = []
	for i in range(0,self.num_joints):
		nv.append(b[i] - a[i])
	mag = 0
	for i in range(0,self.num_joints):
		mag = mag + (nv[i] * nv[i])
	mag = mag ** 0.5
	new_n = []
	for i in range(0,self.num_joints):
		new_n.append((nv[i]*n)/mag)

	return new_n

    def unit_vect(self,a,b):
	nv = []
	for i in range(0,self.num_joints):
		nv.append(b[i] - a[i])
	mag = 0
	for i in range(0,self.num_joints):
		mag = mag + (nv[i] * nv[i])
	mag = mag ** 0.5
	new_n = []
	for i in range(0,self.num_joints):
		new_n.append(nv[i]/mag)

	return new_n

    def new_int_node(self,a,b):
	nv = []
	dis = self.dist(a,b)
	num = self.treelen/dis
	for i in range(0,self.num_joints):
		nv.append(a[i] + num * (b[i] - a[i]))
	return nv

    def smooth_traj(self, i, route):
	#print(i)
	for j in range(i + 1,len(route)):
		val, res = self.is_segment_valid(route[i], route[j])
		#print(j)
		#print(res)
		if res == False:
			break
	if j == len(route) - 1 and res == True:
		return j
	else:		
		return j-1

    def reroute(self,route):
	new_route = []
	i = 0
	nod = 0
	#print("nod:",nod)
	new_route.append(route[0])
	while nod != len(route) - 1:
		nod = self.smooth_traj(i,route)
		new_route.append(route[nod])
		i = nod
		#print("nod:",i)
		#print(len(route))
	#new_route.append(route[len(route) - 1])
	return new_route
   
    """
    def final_route(self, fin):
	for j in range(0,len(fin) - 1): 
		d = self.dist(fin[j],fin[j + 1])
		ints = math.floor(d/self.fin_seg) + 1
		n = 0
		mat = []
		while n < ints:
			nv = []	
			for i in range(0,self.num_joints):
				nv.append((fin[j + 1][i]*(n/ints)) + (fin[j][i]*(1-(n/ints))))
			mat.append(nv)
			n = n + 1
		
	mat.append(fin[len(fin) - 1])	
	return mat
    """

    def final_route(self, fin):
	mat = []
	n = 0
	#print(n)
	mat.append(fin[0])
	for j in range(0,len(fin) - 1): 
		for k in range(1,16):	
			nv = []
			for i in range(0,self.num_joints):
				nv.append(fin[j][i] + ((fin[j + 1][i] - fin[j][i]) * (k/15.0)))
			mat.append(nv)	
			n = n + 1
			#print(n)
		#print(j)
	#mat.append(fin[len(fin) - 1])	
	return mat
	
	
    def obstacle_time(self, a):
	b = 0
	if a == "None":
		b = 10
	elif a == "Simple":
		b = 60
	elif a == "Hard":
		b = 120
	elif a == "Super":
		b = 200
	return b
    

    def is_segment_valid(self, a, b):
	d = self.dist(a,b)
	ints = math.ceil(d/self.obs_seg) 
	n = 0
	mat = []
	while n <= ints:
		nv = []			
		for i in range(0,self.num_joints):
			nv.append((b[i]*(n/ints)) + (a[i]*(1-(n/ints))))
		mat.append(nv)
		flag = self.is_state_valid(nv)
		if flag == False: 
			break
      		n = n + 1		
	#print(n)
	#print(ints)
	#if n == ints and flag == True:
	return mat[n - 1],flag
 
    
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

