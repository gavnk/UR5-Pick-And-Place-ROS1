#!/usr/bin/env python3
'''
           pick and place with UR5 in MoveIt by Gavin Kenny
           this file has the classes 'Movements' & 'GripperController' for main.py
'''
import sys
import rospy
import copy
import numpy as np
import moveit_commander
import math
import geometry_msgs.msg
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64
from moveit_msgs import msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose ,PoseStamped, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place', anonymous=True)

# this class handles the different robot poses for pick and placement
class Movements:
    def __init__(self):
      # Initialization
    
      
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander("ur5_arm")  
     

    # Task Space
    #general pose (inverse kinematics) function
    # takes in an array of orientations as RPY, and xyz positions 
    def plan_IK(self,arr):
    # Define rotation in radians (roll, pitch, yaw)
        roll  = arr[0]
        pitch = arr[1]
        yaw   = arr[2]

        # Create quaternion from roll, pitch, yaw
        q_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        target_pose = Pose()
        target_pose.orientation.x = q_rot[0]
        target_pose.orientation.y = q_rot[1]
        target_pose.orientation.z = q_rot[2]
        target_pose.orientation.w = q_rot[3]
        target_pose.position.x = arr[3]
        target_pose.position.y = arr[4]
        target_pose.position.z = arr[5]
        
        self.group.set_pose_target(target_pose)

        # Create a Cartesian path
        waypoints = [target_pose]

        # Plan the Cartesian path
        plan, fraction = self.group.compute_cartesian_path(waypoints, 
                                                        0.01,  # eef_step
                                                        0.0)   # jump_threshold

        rospy.loginfo("Visualizing plan (cartesian path) (%.2f%% achieved)", fraction * 100.0)

        # Execute the Cartesian path
        self.group.execute(plan)
        self.group.go()
        
        self.group.stop()
        self.group.clear_pose_targets()
        return

    # Joint Space
    # the home position of the robot is below
    def go_to_home(self):

        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] =  0
        joint_goal[1] = -math.pi/2
        joint_goal[2] =  math.pi/2
        joint_goal[3] = -math.pi/2
        joint_goal[4] = -math.pi/2 
        joint_goal[5] = 0

        self.group.go(joint_goal, wait=True)
        self.group.stop()

        return 0

    # general direct kinematics function, takes in an array of angle values in degrees
    # note: the first joint of the robot aka "joint 1" is equal to arr[0]
    def plan_DK(self,arr):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = math.radians(arr[0])
        joint_goal[1] = math.radians(arr[1])
        joint_goal[2] = math.radians(arr[2])
        joint_goal[3] = math.radians(arr[3])
        joint_goal[4] = math.radians(arr[4])
        joint_goal[5] = math.radians(arr[5])
    
        self.group.go(joint_goal, wait=True)
        self.group.stop()

        return 0

    # Joint Space end

    # this function prints about the pose (position and orientation), and the joint values
    def print_text(self):
        # Get the current pose of the end effector
        end_effector_pose = self.group.get_current_pose().pose

        # Print the position
        rospy.loginfo("End Effector Position:")
        rospy.loginfo("x: {:.3f}".format(end_effector_pose.position.x))
        rospy.loginfo("y: {:.3f}".format(end_effector_pose.position.y))
        rospy.loginfo("z: {:.3f}".format(end_effector_pose.position.z))

        # Print the orientation (quaternion)
        rospy.loginfo("End Effector Orientation (Quaternion):")
        rospy.loginfo("x: {:.3f}".format(end_effector_pose.orientation.x))
        rospy.loginfo("y: {:.3f}".format(end_effector_pose.orientation.y))
        rospy.loginfo("z: {:.3f}".format(end_effector_pose.orientation.z))
        rospy.loginfo("w: {:.3f}".format(end_effector_pose.orientation.w))

        # Optionally, print the orientation as Euler angles
        euler = tf.transformations.euler_from_quaternion([
            end_effector_pose.orientation.x,
            end_effector_pose.orientation.y,
            end_effector_pose.orientation.z,
            end_effector_pose.orientation.w
        ])
        
        rospy.loginfo("End Effector Orientation (Euler):")
        rospy.loginfo("Roll: {:.3f}, Pitch: {:.3f}, Yaw: {:.3f}".format(euler[0], euler[1], euler[2]))
        
        # Get the current joint values
        joint_values = self.group.get_current_joint_values()
        rospy.loginfo("Joint Values (Radians):")
        
        for i, joint_angle in enumerate(joint_values):
            rospy.loginfo("Joint {}: {:.3f} rad".format(i + 1, joint_angle))

        # Convert joint values to degrees
        joint_values_degrees = [math.degrees(angle) for angle in joint_values]
        rospy.loginfo("Joint Values (Degrees):")
        
        for i, joint_angle_deg in enumerate(joint_values_degrees):
            rospy.loginfo("Joint {}: {:.3f} deg".format(i + 1, joint_angle_deg))





# this class handles the opening and closing of the gripper, which is a plugin in this project
# the size of how far to close the gripper can be adjusted
class GripperController:
    def __init__(self):
        self.d_finger_pub = rospy.Publisher('/d_finger_controller/command', Float64, queue_size=10)
        self.u_finger_pub = rospy.Publisher('/u_finger_controller/command', Float64, queue_size=10)
        self.left_inner_pub = rospy.Publisher('/left_inner_finger_controller/command', Float64, queue_size=10)
        self.right_inner_pub = rospy.Publisher('/right_inner_finger_controller/command', Float64, queue_size=10)
        self.left_pub = rospy.Publisher('/left_finger_controller/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_finger_controller/command', Float64, queue_size=10)
        self.srv = rospy.Service('toggle_gripper', SetBool, self.handle_toggle_gripper)
        self.close_size = 0.55
        self.open_size = 0

    def handle_toggle_gripper(self, req):
        if req.data:  # open the gripper
            self.close_gripper()
        else:  # close the gripper
            self.open_gripper()
        return SetBoolResponse(success=True, message="Gripper action completed.")

    def close_gripper(self, size=None):
        if size is not None:
            self.close_size = size  # Update the close size if a new value is provided
        self.d_finger_pub.publish(self.close_size)
        self.u_finger_pub.publish(-(self.close_size))
        self.left_inner_pub.publish(-(self.close_size))
        self.right_inner_pub.publish(-(self.close_size))
        self.left_pub.publish(self.close_size)
        self.right_pub.publish(self.close_size)
        print(f"Gripper closed to size {self.close_size}.")

    def open_gripper(self):
        self.d_finger_pub.publish(self.open_size)
        self.u_finger_pub.publish(self.open_size)
        self.left_inner_pub.publish(self.open_size)
        self.right_inner_pub.publish(self.open_size)
        self.left_pub.publish(self.open_size)
        self.right_pub.publish(self.open_size)
        print("Gripper action completed.")










    
   
     

