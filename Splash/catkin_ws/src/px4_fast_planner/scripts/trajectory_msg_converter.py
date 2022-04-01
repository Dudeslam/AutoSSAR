#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader

add function to also convert data from other UAVs
Authors: Liping S
"""


# Imports
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler

import tf
from geometry_msgs.msg import PoseStamped



class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'command/trajectory')

        # for multi-uav coordination
        self.tl = tf.TransformListener()
        coord_sub_topic = rospy.get_param('~coord_sub_topic', 'follower_1_other/relay_pose')
        coord_pub_topic = rospy.get_param('~coord_pub_topic', 'follower_1_other/in_world_pose')
        # Subscriber the original pose of the other UAV in global_world
        rospy.Subscriber(coord_sub_topic, PoseStamped, self.other_uav_message_handler)
        # Publish pose of other UAVs in local world frame
        self.other_uav_pose_pub = rospy.Publisher(coord_pub_topic, PoseStamped, queue_size=1)

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def other_uav_message_handler(self, msg): 
        pose_in_world = self.tl.transformPose('world', msg)
        self.other_uav_pose_pub.publish(pose_in_world)


    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # TODO: set vel.angular to msg.yaw_dot

        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)
        self.traj_pub.publish(traj_msg)

if __name__ == '__main__':
    obj = MessageConverter()
    