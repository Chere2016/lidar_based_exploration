#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from utils_lib.online_planning import StateValidityChecker, wrap_angle
from utils_lib import exploration_sampling as es
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

class SamplingExplorer:
    def __init__(self):
        rospy.init_node("sampling_explorer")

        # Flags and data
        self.map_received = False
        self.odom_received = False
        self.goal_sent = False
        self.map = None
        self.robot_pose = None

        # ROS interfaces
        rospy.Subscriber("/goal_reached", Bool, self.goal_reached_callback)
        rospy.Subscriber("/projected_map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/turtlebot/odom_ground_truth", Odometry, self.odom_callback)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.goal_marker_pub = rospy.Publisher("/goal_marker", Marker, queue_size=1)

        # State validity checker
        self.svc = StateValidityChecker(distance=0.2, is_unknown_valid=True)

        rospy.Timer(rospy.Duration(1.0), self.main_loop)
        rospy.loginfo("Sampling Exploration node is ready.")

    def map_callback(self, msg):
        self.map_received = True
        self.resolution = msg.info.resolution
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        width = msg.info.width
        height = msg.info.height
        self.map = np.array(msg.data, dtype=np.int8).reshape((height, width)).T
        self.svc.set(self.map, self.resolution, self.origin)

    def odom_callback(self, msg):
        self.odom_received = True
        self.robot_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw_from_quat(msg.pose.pose.orientation)
        ])

    def get_yaw_from_quat(self, orientation):
        import tf
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw
    def goal_reached_callback(self, msg):
        if msg.data:
            rospy.loginfo("Goal reached! resampling next goal")
            self.goal_sent = False



    def main_loop(self, event):
        if not (self.map_received and self.odom_received):
            rospy.loginfo_throttle(5, "Waiting for map and odom...")
            return

        if self.goal_sent:
            return  # prevents sampling if goal is active

        rospy.loginfo("Sampling candidate goals...")

        # Sample goals
        candidates = es.sample_candidate_goals(self.origin, self.resolution, self.map.shape, num_samples=200)
        valid_goals = [g for g in candidates if self.svc.is_valid(g)]

        if not valid_goals:
            rospy.logwarn("No valid goals found during sampling.")
            return

        info_gains = [
            es.compute_info_gain(self.map, g, self.origin, self.resolution, radius=1.0)
            for g in valid_goals
        ]

        MIN_INFO_GAIN = 0.5  # Tune as needed
        filtered_goals = [g for i, g in enumerate(valid_goals) if info_gains[i] > MIN_INFO_GAIN]
        filtered_gains = [ig for ig in info_gains if ig > MIN_INFO_GAIN]

        if not filtered_goals:
            rospy.logwarn("No informative goals found. Exploration may be complete.")
            return

        best_idx = es.score_goals(filtered_goals, self.robot_pose, filtered_gains)
        best_goal = filtered_goals[best_idx]
        gain = filtered_gains[best_idx]

        rospy.loginfo(f"Selected best goal: {best_goal} (info gain = {gain:.2f})")
        self.publish_goal(best_goal)
        self.goal_sent = True

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = "world_ned"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.publish_goal_marker(goal)
    def publish_goal_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.goal_marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        SamplingExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
