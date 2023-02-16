#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import csv

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator
from geometry_msgs.msg import TransformStamped, Twist
from tf.transformations import euler_from_quaternion

class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator
    def __init__(self):
        #Initializes rospy publisher at desired_positions and sets rate
        self.pub_traj = rospy.Publisher('/desired_positions', Twist, queue_size = 500)
        self.rate = rospy.Rate(20)
        #Subscribes to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
        self.vicon_topic = '/vicon/ARDroneCarre/ARDroneCarre'
        self._vicon_msg = TransformStamped()
        self.sub_vicon = rospy.Subscriber(self.vicon_topic, TransformStamped, self._vicon_callback)
        #Current desired position index. Will increment as the drone passes desired positions.
        #Resets when trajectory is complete
        self.idx = 0
        #Intializes position array and desired position values
        self.position = np.zeros(6)
        self.x_des = []
        self.y_des = []
        self.z_des = []
        self.yaw_des = []
        #Describes how many desired points/waypoints are in each trajectory before looping
        self.desired_resolution = 15*2
        #Creates the lists of points using the chosen trajectory method
        #Uncomment for circular trajectory
        #self.linear_traj(-1.5,-1.5,1,1.5,1.5,2)
        self.circular_traj(1.5,0,0)
        #Initializes an empty log
        self.log = []
  
    #Vicon callback to get position/attitude message
    def _vicon_callback(self,msg):
        self._vicon_msg = msg
        return None        

    #Adds lists of desired x,y,z and yaw values to form a linear trajectory
    def linear_traj(self,x1,y1,z1,x2,y2,z2):
        #Specify desired start and end positions
        #Uses desired resolution for linear interpolation. Divided by two to include return trajectory
        n = self.desired_resolution//2
        #Uses linspace to linearly interpolate between a start and end for x,y,z
        #Also adds a reversed list to each trajectory to return to the start
        x_list = np.linspace(x1,x2,n)
        self.x_des = np.concatenate((x_list,np.flipud(x_list)))
        y_list = np.linspace(y1,y2,n)
        self.y_des = np.concatenate((y_list,np.flipud(y_list)))
        z_list = np.linspace(z1,z2,n)
        self.z_des = np.concatenate((z_list,np.flipud(z_list)))
        #Constant 0 yaw
        self.yaw_des = np.zeros(2*n)
        return None

    #Adds lists of desired x,y,z and yaw values to form a circular trajectory
    #Desired circle radius specified with r, and x and y offset with x and y
    def circular_traj(self,r,x_offset,y_offset):
        #Uses desired resolution for linear interpolation. Divided by two to include return trajectory
        n = self.desired_resolution//2
        #Uses trig and linear interp via linspace to calculate a list of positions within circle
        rad = np.linspace(0,2*np.pi,2*n)
        self.x_des = r*np.cos(rad)+x_offset
        self.y_des = r*np.sin(rad)+y_offset
        z_list = np.linspace(0.5,1.5,n)
        self.z_des = np.concatenate((z_list,np.flipud(z_list)))
        #List of angles such that the drone faces inwards
        self.yaw_des = np.linspace(-np.pi,np.pi,2*n)
        return None

    #Publishes message containing desired x,y,z and yaw
    def publish_desired(self):
        msg = Twist()
        msg.linear.x = self.x_des[self.idx]
        msg.linear.y = self.y_des[self.idx]
        msg.linear.z = self.z_des[self.idx]
        msg.angular.z = self.yaw_des[self.idx]
        self.pub_traj.publish(msg)
        return None

    #Updates the position (linear + angular), from the vicon subscription
    def update_state(self): 
        #Extracts vectors for linear and rotational position
        vicon_lin = self._vicon_msg.transform.translation
        vicon_rot = self._vicon_msg.transform.rotation
        #Reformats linear/rotational position
        position_array = [vicon_lin.x,
                          vicon_lin.y,
                          vicon_lin.z]
        rotation_array = [vicon_rot.x,
                          vicon_rot.y,
                          vicon_rot.z,
                          vicon_rot.w]
        rotation_array_euler = list(euler_from_quaternion(rotation_array))
        #Format returned is [x,y,z,roll,pitch,yaw]
        self.position = np.array(position_array + rotation_array_euler) 
        return None

    #Updates the desired position if within tolerance
    def update_desired(self,pos_tol,ang_tol):
        #Extracts position and desired position
        x_des,y_des,z_des = self.x_des[self.idx],self.y_des[self.idx],self.z_des[self.idx]
        yaw_des = self.yaw_des[self.idx]
        x,y,z = self.position[0], self.position[1],self.position[2]
        yaw = self.position[5]
        distance_error = ((x-x_des)**2)+((y-y_des)**2)+((z-z_des)**2)**0.5
        #If both position and angle error are within position tolerance and angle tolerance
        if distance_error < pos_tol and abs(yaw-yaw_des) < ang_tol:
            #Updates to next desired position. Loops when all waypoints are complete
            self.idx = (self.idx + 1) % self.desired_resolution
        return None

    #Updates and regularly stores the data for later plotting
    def update_log(self):
        #Saves the important position and desired position data into a log list
        idx = self.idx
        pos = self.position
        self.log.append([pos[0],
                        pos[1],
                        pos[2],
                        pos[5],
                        self.x_des[idx],
                        self.y_des[idx],
                        self.z_des[idx],
                        self.yaw_des[idx],
			            rospy.get_time()])
        #After 3000 log entries
        if len(self.log) > 2500:
            #A csv file is created. ENTER FILENAME WITH FULLPATH WITHIN QUOTES TO SPECIFY WHERE LOGS SHOULD BE SAVED
            with open("/home/zirui/aer1217/labs/src/aer1217_ardrone_simulator/scripts/circular.csv",'w') as file:	
                writer = csv.writer(file)
                #The first line describes what is in each column
                writer.writerow(["x","y","z","yaw","x desired","y desired","z desired","yaw desired","time"])
                #Each rounded log entry is recorded in the csv
                writer.writerows(np.around(self.log,5))
            #The log is reset
            self.log = []
        return None

#Run by ROS:
if __name__ == '__main__':
    #Initializes rospy node and position generator object
    rospy.init_node('desired_positions')
    position_generator = ROSDesiredPositionGenerator()
    #Loops while rospy is running
    while not rospy.is_shutdown():
        #Updates the state variables in position generator
        position_generator.update_state()
        #Updates desired position if the position is within position/radian angle tolerance
        position_generator.update_desired(0.1,0.0872665)
        #Publishes current waypoint
        position_generator.publish_desired()
        #Updates the log files of the position generator for later analysis
        position_generator.update_log()
        position_generator.rate.sleep()
    rospy.spin()

