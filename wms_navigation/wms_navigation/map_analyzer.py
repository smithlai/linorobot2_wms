#! /usr/bin/env python3

# Copyright (c) 2022 Smith Lai from UG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This Node will try to retrieve OccupancyGrid from slam toolbox.

# ros2
import rclpy
from rclpy.node import Node

# messages
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, PoseArray
# 3rd party
import time
import numpy as np
import scipy.signal
import scipy.ndimage
# import scipy.ndimage.label
from skimage import feature
import matplotlib.pyplot as plt

class MapAnalyzer(Node):
    def __init__(self):
        super().__init__('map_analyzer')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('local_radius', 100),
                ('kernal_size', 9),
                ('map_topic', 'map'),
                ('odom_topic', 'odom'),
                ('local_candidate_topic', 'local_candidate_topic')
            ])
        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        
        local_candidate_topic = self.get_parameter('local_candidate_topic').value
        self.occupancy_subscription = self.create_subscription(OccupancyGrid, map_topic, self.occupancy_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.my_local_candidates_publisher = self.create_publisher(
            PoseArray, 
            f'/{local_candidate_topic}', 
            1)
        self.busy_occupancy_callback = False
        self.robot_position = None

        self.fig = plt.figure()

    def convolute(self, full_map, valid_mask, k_size=15, iteration=1):
        k_width = (k_size//2)
        # conv_base = np.array(full_map[y_min:y_max, x_min:x_max])
        
        k = np.ones([k_size, k_size])
        k[k_width, k_width] = 0 

        conv_base = np.array(full_map)
        for i in range(iteration):
            conv_result = scipy.signal.convolve2d(conv_base, k, mode='same', boundary='fill', fillvalue=100)
            conv_result = np.where(valid_mask & (conv_result <= 0), conv_result, 0 )
            conv_base = conv_result
        return conv_result

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible candidates. It sorts them and
        saves them in the self.sorted_accessible_local_candidates variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """
        self.get_logger().info(f"======= occupancy_callback start =========")
        
        if self.busy_occupancy_callback:
            self.get_logger().info(f"------- occupancy_callback cancel (busying)------")
            return
        
        self.busy_occupancy_callback=True
        data = np.array(msg.data)  # download the occupancy grid
        current_map_width = msg.info.width  # get the current map width
        current_map_height = msg.info.height  # get the current map height
        resolution = msg.info.resolution  # get the resolution
        shiftX = msg.info.origin.position.x
        shiftY = msg.info.origin.position.y
        # # reshape the data so it resembles the map shape
        data = np.reshape(data, (current_map_height, current_map_width))
        # image = np.where(data<0 , 50 , data)  # make unknown grid gray
        robot_position_x = robot_position_y = 0
        if self.robot_position:
            robot_position_x = round((self.robot_position.x-shiftX) / resolution)
            robot_position_y = round((self.robot_position.y-shiftY) / resolution)
            # image[robot_position_y-4:robot_position_y+4,robot_position_x-4:robot_position_x+4] = 100
        # for j in range(current_map_height):
        #     for i in range(current_map_width):
        #         if image[j,i] != 0:
        #             self.get_logger().info(f"{image[j,i]}")

        

        # Local Candidate
        if not self.robot_position:
            self.get_logger().info(f"------- occupancy_callback cancel (no robot info)------")
            pass
        else:
            startT = time.time()
            full_map = np.copy(data)
            
            # Dilate all wall points
            wall_mask = scipy.ndimage.grey_dilation(data, size=(25,25))

            # remove wall point
            valid_mask = np.where(wall_mask > 0, False, True)
            
            # remove unknown point
            valid_mask = np.where(full_map < 0, False, valid_mask)
            
            # Union robot range and empty point
            local_radius = self.get_parameter('local_radius').value
            y_min = max(robot_position_y-local_radius, 0)
            y_max = robot_position_y+local_radius
            x_min = max(robot_position_x-local_radius, 0)
            x_max = robot_position_x+local_radius

            radius_mask = np.full_like(data, False)
            radius_mask[y_min:y_max, x_min:x_max] = True
            
            # conv to get most unknown points
            kernal_size = self.get_parameter('kernal_size').value
            conv_result = self.convolute(full_map, valid_mask, k_size=kernal_size, iteration=1)
            
            local_conv = np.where(radius_mask, conv_result, 0)

            global_conv = np.where(np.logical_not(radius_mask), conv_result, 0)
            # equals to: global_conv = np.where(radius_mask, 0, conv_result)
            
            # ==== local
            local_cord = np.where(local_conv < 0)
            local_values = local_conv[local_cord]
            # x,y
            local_candidates = np.asarray(local_cord).T[:,::-1]
            local_sorted_index = local_values.argsort()
            local_values = local_values[local_sorted_index]
            local_candidates = local_candidates[local_sorted_index]
            local_candidates = local_candidates[:3]

            # ==== global
            global_cord = np.where(global_conv < 0)
            global_values = global_conv[global_cord]
            # x,y
            global_candidates = np.asarray(global_cord).T[:,::-1]

            global_sorted_index = global_values.argsort()
            global_values = global_values[global_sorted_index]
            global_candidates = global_candidates[global_sorted_index]
            global_candidates = global_candidates[:3]

            all_candidates = np.concatenate((local_candidates,global_candidates),axis=0)
            deltaT = time.time() - startT
            self.get_logger().info(f"-------- Got {all_candidates.shape[0]} candidates, cost. {deltaT} ------")
            startT = time.time()
            msg = PoseArray()
            for i in range(min(all_candidates.shape[0], 5)):
                candidate = all_candidates[i]
                new_position_x = candidate[0] * resolution + shiftX
                new_position_y = candidate[1] * resolution + shiftY
                new_p = Pose()
                new_p.position.x = new_position_x
                new_p.position.y = new_position_y
                # new_p.position.z = 0.0
                msg.poses.append(new_p)
            self.my_local_candidates_publisher.publish(msg)
            
            # https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib
            # full_map = np.where(full_map < 0, 50, full_map)
            # for i in range(min(all_candidates.shape[0], 5)):
            #     size = 5 - (i)
            #     candidate = all_candidates[i]
            #     # self.get_logger().info(f"candidate{i}. {candidate} -> {candidate[1]}:-size:candidate[1]+size, candidate[0]-size:candidate[0]+size]}")
            #     full_map[candidate[1]-size:candidate[1]+size, candidate[0]-size:candidate[0]+size] = 100
                
            # cmap = 'gray_r'
            # origin='lower'
            # plt.figimage(full_map, cmap=cmap, origin=origin)
            # self.fig.canvas.draw()
            # plt.pause(0.01)
            
        self.busy_occupancy_callback=False

    def odom_callback(self, msg):      
        pose = msg.pose.pose
        position = pose.position
        # orientation = pose.orientation
        self.robot_position = position
        # self.get_logger().info(f'self.robot_position={self.robot_position}')  


def main(args=None):
    rclpy.init(args=args)

    map_analyzer = MapAnalyzer()

    rclpy.spin(map_analyzer)

    map_analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()