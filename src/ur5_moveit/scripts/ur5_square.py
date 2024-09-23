#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_square_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")

# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10, latch=True)
marker = Marker()
marker.header.frame_id = "base_link"
marker.id = 0
marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.01  # Line width
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0
marker.points = []

# Define the square trajectory in YZ plane
def square_trajectory(side_length, center):
    waypoints = []
    half_side = side_length / 2.0
    corners = [
        (center[0], center[1] - half_side, center[2] + half_side),  # Bottom Left
        (center[0], center[1] + half_side, center[2] + half_side),  # Top Left
        (center[0], center[1] + half_side, center[2] - half_side),  # Top Right
        (center[0], center[1] - half_side, center[2] - half_side),  # Bottom Right
        (center[0], center[1] - half_side, center[2] + half_side),  # Back to Bottom Left to close the square
    ]

    # Create waypoints for the corners of the square
    for corner in corners:
        pose = group.get_current_pose().pose
        pose.position.x = corner[0]
        pose.position.y = corner[1]
        pose.position.z = corner[2]
        waypoints.append(pose)
        # Publish each corner as a marker
        marker.points.append(Point(pose.position.x + 0.2, pose.position.y, pose.position.z-1.02))

    # Publish the marker
    marker_publisher.publish(marker)
    return waypoints

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
        print("Could not compute full trajectory. Only " + str(fraction * 100) + "% achieved.")

# Main Execution
if __name__ == '__main__':
    current_pose = group.get_current_pose().pose.position
    side_length = 0.1  # Change this to the desired side length of the square
    while not rospy.is_shutdown():
        waypoints = square_trajectory(side_length, [current_pose.x, current_pose.y, current_pose.z])
        execute_trajectory(waypoints)
        marker_publisher.publish(marker)
        rospy.sleep(5)  # Add a delay to observe the drawing if needed
        break

