#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64

class GripperController:
    def __init__(self):
        self.d_finger_pub = rospy.Publisher('/d_finger_controller/command', Float64, queue_size=10)
        self.u_finger_pub = rospy.Publisher('/u_finger_controller/command', Float64, queue_size=10)
        self.left_inner_pub = rospy.Publisher('/left_inner_finger_controller/command', Float64, queue_size=10)
        self.right_inner_pub = rospy.Publisher('/right_inner_finger_controller/command', Float64, queue_size=10)
        self.left_pub = rospy.Publisher('/left_finger_controller/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_finger_controller/command', Float64, queue_size=10)
        self.srv = rospy.Service('toggle_gripper', SetBool, self.handle_toggle_gripper)
        self.close_size = 1
        self.open_size = 0

    def handle_toggle_gripper(self, req):
        if req.data:  # open the gripper
            self.d_finger_pub.publish(self.close_size)
            self.u_finger_pub.publish(-(self.close_size))
            self.left_inner_pub.publish(-(self.close_size))
            self.right_inner_pub.publish(-(self.close_size))
            self.left_pub.publish(self.close_size)
            self.right_pub.publish(self.close_size)
        else:  # close the gripper
            self.d_finger_pub.publish(self.open_size)
            self.u_finger_pub.publish(self.open_size)
            self.left_inner_pub.publish(self.open_size)
            self.right_inner_pub.publish(self.open_size)
            self.left_pub.publish(self.open_size)
            self.right_pub.publish(self.open_size)
        return SetBoolResponse(success=True, message="Gripper action completed.")

if __name__ == '__main__':
    rospy.init_node('gripper_controller')
    controller = GripperController()
    rospy.spin()

