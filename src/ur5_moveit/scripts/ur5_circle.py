#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_circle_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")  
# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10)
marker = Marker()
marker.header.frame_id = "base_link"
marker.id = 0
marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.01  # Line width
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Define the circle trajectory in y-z plane (quarter circle)
def circle_trajectory(radius, center, num_points):
    waypoints = []
    segment_points = num_points  # Considering only a quarter circle

    for i in range(segment_points):
        theta = 2.0 * math.pi * float(i) / float(num_points)
        x = center[0]  # keep x constant
        y = center[1] + radius * math.cos(theta)
        z = center[2] + radius * math.sin(theta)

        # create a pose
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        waypoints.append(pose)
        publish_marker(pose.position)

    return waypoints

def publish_marker(position):
    point = Point()
    point.x = position.x + 0.2
    point.y = position.y
    point.z = position.z - 1.02
    marker.points.append(point)
    marker_publisher.publish(marker)

# Command the UR5 to follow the trajectory
def execute_trajectory(waypoints):
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,
                            0.01,  # eef_step
                            0.0)   # jump_threshold

    if fraction == 1.0:
        print("Full trajectory computed!")
        group.execute(plan, wait=True)
    else:
        print("Could not compute full trajectory. Only " + str(fraction*100) + "% achieved.")

# Main Execution
if __name__ == '__main__':
    current_pose = group.get_current_pose().pose.position
    while not rospy.is_shutdown():
        waypoints = circle_trajectory(0.1, [current_pose.x, current_pose.y, current_pose.z], 100)
        execute_trajectory(waypoints)

