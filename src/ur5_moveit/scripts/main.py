#!/usr/bin/env python3
'''
           pick and place with UR5 in MoveIt by Gavin Kenny
           this file is the main.py file for the project
           to run first launch 'demo_gazebo.launch' in a separate termainal with:
           'roslaunch ur5_moveit demo_gazebo.launch'
           then in run in the scripts folder directory with: python3 main.py

'''
import sys
import rospy
import p_and_p
from p_and_p import GripperController
from p_and_p import Movements


def main():

    # create the objects for the robot movements and gripper control
    gripper_controller = GripperController()
    movements = Movements()

    print ("============ Reference frame: %s" % movements.group.get_planning_frame())
    print ("============ Reference frame: %s" % movements.group.get_end_effector_link())
    print ("============ Robot Groups:")
    print (movements.robot.get_group_names())
    #current_pose = group.get_current_pose().pose.position
  
    movements.group.set_max_velocity_scaling_factor(0.6) # speed of movements
    movements.group.clear_pose_targets()

    # below are arrays storing the angle values for each of poses
    # to add more poses simply create a new array, add angles for the poses
    # and then put the array name as an argument to the planIK (inverse kinematics), 
    # or planDK (direct kinematics) functions, as below.
    # the function definitions are in the class 'Movements' in the p_and_p.py file

    # function arguments for plan_IK roll  ,pitch  ,yaw   ,x     ,y     ,z
    goToPickUpRedPegIK   = [-3.124, -0.003, 3.141, 0.591, 0.015, 1.262]
    goToPlaceRedPegIK = [1.588,1.536,1.642,0.214,-0.305,1.215]
    #goToPickUpGreenPeg = [3.140, -0.017, -2.984, 0.588, 0.140, 1.259]
    goToPickUpGreenPegIK = [-3.141, -0.003, 3.3, 0.588, 0.140, 1.259]
    goToPlaceGreenPegIK = []

    # function arguments for plan_DK joint angles in degress
    goToPickUpRedPegDK = [-9,18,97,-115,-91,-9]
    goToPlaceRedPegDK = [-86,-0,125,-33,1,-180]
    goToPickUpGreenPegDK = [3,19,97,-115,-90,-6]
    goToPlaceGreenPegDK = [-92,4,123,-40,-13,-175]


    rospy.sleep(1) # starting up..
    movements.go_to_home() ## make sure to start from home position
 

   
    #------------------------- pick up red peg ---------------------------------------
    
    movements.plan_IK(goToPickUpRedPegIK)
    #movements.plan_DK(goToPickUpRedPegDK)
    movements.print_text()                       # print position and orientation
    rospy.sleep(2)                     # wait
    gripper_controller.close_gripper() # Close the gripper 
    rospy.sleep(4)                     # wait
    movements.go_to_home()                       # back to home position
    rospy.sleep(1)                     # wait
    
    #------------------------- place red peg ----------------------------------------
    #movements.plan_IK(goToPlaceRedPegIK)
    movements.plan_DK(goToPlaceRedPegDK)
    movements.print_text()                       # print position and orientation
    rospy.sleep(2)                     # wait
    gripper_controller.open_gripper()  # open gripper
    rospy.sleep(4)                     # wait
    movements.go_to_home()                       # go to home position
    rospy.sleep(1)                     # wait
    
    #----------------------- pick up green peg --------------------------------------
  
    #movements.plan_IK(goToPickUpGreenPegIK)
    movements.plan_DK(goToPickUpGreenPegDK)
    movements.print_text() #print position and orientation
    rospy.sleep(2)                     # wait
    gripper_controller.close_gripper(size=0.5)  # open gripper
    rospy.sleep(4)                     # wait
    movements.go_to_home()                       # go to home position
    rospy.sleep(1)                     # wait

    #------------------------ place green peg ---------------------------------------
   
    #movements.plan_IK(goToPlaceGreenPegIK)
    movements.plan_DK(goToPlaceGreenPegDK)
    movements.print_text() #print position and orientation
    rospy.sleep(2)                     # wait
    gripper_controller.open_gripper() # open gripper
    rospy.sleep(4)                     # wait
    movements.go_to_home()                       # go to home position
                        #end
    
    

if __name__ == '__main__':
   main() # call to main function
   
    