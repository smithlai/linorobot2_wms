#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# 3rd party
import numpy as np
import pandas as pd
import os
# import csv
# import time
# from ament_index_python.packages import get_package_share_directory

# ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer   #, CancelResponse
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rcl_interfaces.msg import ParameterType


# messages
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseArray
# from nav_msgs.msg import MapMetaData
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from wms_interfaces.action import Discover


# ros2 action send_goal wander explorer_interfaces/action/Wander "{strategy: 1, map_completed_thres: 0.6}"


class DiscovererServer(Node):
    def __init__(self):
        super().__init__('discoverer_server')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('local_candidate_topic', 'local_candidate_topic'),
                ('map_topic', 'map')
            ])
        
        self.group1 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()
        self.group2 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()

        # This Action server
        self._action_server = ActionServer(self, Discover, 'discover', self.execute_callback, callback_group=self.group1)

        # Describe candidate data
        local_candidate_topic = self.get_parameter('local_candidate_topic').value
        self.sorted_local_candidates = []
        self.local_candidate_subscriber = self.create_subscription(PoseArray, local_candidate_topic, self.local_candidate_callback, 10, callback_group=self.group2)

        # Register Nav client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.group3)

        # Global variables
        self.stop_discovering = False
        self.map_completed_thres=1.0 #Initialize threshold to max (100%)
        self.get_logger().info("Discoverer Server is ready")

    def local_candidate_callback(self, msg):   
        # self.get_logger().info("sorted_local_candidates UPDATED")   
        poses = msg.poses
        self.sorted_local_candidates = poses
            
    def execute_callback(self, goal_handle):
        self.get_logger().info("Discoverer Server received a goal")
        self.map_completed_thres=goal_handle.request.map_completed_thres
        self.get_logger().info("Map completed threshold set to: %s" %self.map_completed_thres)
        while not self.stop_discovering:
            self.send_goal()
            self.get_logger().warning('self.send_goal() done')

        self.get_logger().info('Discovering Finished')
        goal_handle.succeed()
        return Discover.Result()

    """
    send_goal -> goal_response_callback -> get_result_callback
    """
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.stop_discovering = True
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived at destination')
        else:
            self.stop_discovering = True
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def create_goal(self):
        # create goal command
        # pop first element from the list, in case the sorted_local_candidates didn't refresh
        goal_msg=None
        try:
            next_candidate = self.sorted_local_candidates.pop(0)
            self.get_logger().warning('new goal = {next_candidate.x},{next_candidate.y}')
            map_topic = self.get_parameter('map_topic').value
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = map_topic
            goal_msg.pose.pose = next_candidate
            # goal_msg.pose.pose.orientation.w = 1.0 # default
        except Exception as e:
            self.get_logger().info('Error while pop sorted_local_candidates')

        return goal_msg

    def send_goal(self):
        self.stop_discovering = False
        self.get_logger().info('Waiting for Navigation server...start')
        self._action_client.wait_for_server()
        self.get_logger().info('Waiting for Navigation server...done')

        goal_msg = self.create_goal()
        if goal_msg is None:
            self.get_logger().warning('No sorted_local_candidates available.')
            return

        # Send goal and wait
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # check goal accept or reject, if accepted, add self.get_result_callback
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        # After complete, request for result
        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

def main(args=None):
    rclpy.init(args=args)

    try:
        # declare the node constructor
        discoverer_server = DiscovererServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(discoverer_server)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            discoverer_server.destroy_node()

    finally:
        # shutdown the ROS communication
        rclpy.shutdown()

if __name__ == '__main__':
    main()
