#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from utils_lib.online_planning import StateValidityChecker

class SamplingExplorer:
    def __init__(self):
        rospy.init_node("sampling_explorer")

        # Flags and buffers
        self.map_received = False
        self.odom_received = False
        self.goal_sent = False

        # Robot state and map
        self.map = None
        self.odom = None
        self.resolution = None
        self.origin = None

        # Setup state validity checker
        self.svc = StateValidityChecker(distance=0.2, is_unknown_valid=True)

        # Subscribers
        rospy.Subscriber("/projected_map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/turtlebot/odom_ground_truth", Odometry, self.odom_callback)

        # Publisher to send goal
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # Run main loop at 1 Hz
        rospy.Timer(rospy.Duration(1.0), self.main_loop)

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

    def main_loop(self, event):
        if not self.map_received or not self.odom_received:
            rospy.loginfo_throttle(5, "Waiting for map and odometry...")
            return

        if self.goal_sent:
            return  # Only send one goal for now

        # === STEP 1: Sample goals ===
        for _ in range(100):  # Try up to 100 samples
            goal = self.sample_goal()
            if self.svc.is_valid(goal):
                rospy.loginfo(f"Sampled valid goal: {goal}")
                self.publish_goal(goal)
                self.goal_sent = True
                break

    def sample_goal(self):
        # Uniform sampling in known map bounds
        x = np.random.uniform(self.origin[0], self.origin[0] + self.map.shape[0] * self.resolution)
        y = np.random.uniform(self.origin[1], self.origin[1] + self.map.shape[1] * self.resolution)
        return [x, y]

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = "world_ned"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 0
        msg.pose.orientation.w = 1.0  # Facing forward
        self.goal_pub.publish(msg)

if __name__ == "__main__":
    try:
        node = SamplingExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
