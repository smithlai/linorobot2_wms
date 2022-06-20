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
import numpy as np
import matplotlib.pyplot as plt
# Todo :距離已知點太遠也不行
class MapAnalyzer(Node):
    def __init__(self):
        super().__init__('map_analyzer')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('candidates_radius_num', 40),
                ('candidates_pixel_step', 1),
                ('convolution_size', 3),
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
        candidates_radius_num = self.get_parameter('candidates_radius_num').value
        candidates_pixel_step = self.get_parameter('candidates_pixel_step').value
        self.robot_position = None
        # create a 100*100 map, each axis coordinate increased by 0.2m, that is [0.0,0.2,0.4...19.8] 
        # ([(x1,y1),(x2,y2)......])
        self.nearby_candidates = self.generate_list_of_candidates(candidates_radius_num=candidates_radius_num, step=candidates_pixel_step)
        self.sorted_accessible_local_candidates = np.array([])
        self.fig = plt.figure()

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible candidates. It sorts them and
        saves them in the self.sorted_accessible_local_candidates variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """
        if self.busy_occupancy_callback:
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
        image = np.where(data<0 , 50 , data)  # make unknown grid gray
        robot_position_x = robot_position_y = 0
        if self.robot_position:
            robot_position_x = round((self.robot_position.x-shiftX) / resolution)
            robot_position_y = round((self.robot_position.y-shiftY) / resolution)
            image[robot_position_y-4:robot_position_y+4,robot_position_x-4:robot_position_x+4] = 100
        # for j in range(current_map_height):
        #     for i in range(current_map_width):
        #         if image[j,i] != 0:
        #             self.get_logger().info(f"{image[j,i]}")

        # https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib

        # Local Candidate
        if not self.robot_position:
            pass
        else:
            # Here we go through every candidate and save the ones that are accessible.
            # An accessible candidate is one which has no obstacles, and has few or no unknown squares in the vicinity.
            local_accessible_candidates = np.array([], dtype=np.int32) #([(x1,y1),(x2,y2)......])
            occupancy_value = np.array([])
            for candidate in self.nearby_candidates: #([(x1,y1),(x2,y2)......])
                try:
                    x = round(candidate[0]) + robot_position_x
                    y = round(candidate[1]) + robot_position_y
                    if x < 0 or y < 0:
                        continue
                    occupancy_grid_coordinate = [x, y]
                    convolution_size = self.get_parameter('convolution_size').value
                    conv_accessable, worth = self.convolute(data, occupancy_grid_coordinate, size=convolution_size)  # perform convolution

                    # if the convolution returns True, it means the WP is accessible (under threshold), 
                    # so it is stored in local_accessible_candidates
                    if conv_accessable:
                        local_accessible_candidates = np.append(local_accessible_candidates, occupancy_grid_coordinate)
                        occupancy_value = np.append(occupancy_value, worth)
                    else:
                        pass
                # because the candidate array is over-sized, we need to remove the values that are out of range
                except IndexError:
                    pass
            # reshape the accessible candidates array to shape (n, 2)
            local_accessible_candidates = local_accessible_candidates.reshape((-1, 2))  #([(x1,y1),(x2,y2)......])
            # Sorting candidates according to occupancy value. This allows the robot to prioritize the candidates with
            # more uncertainty (it wont access the areas that are completely clear, thus going to the discovery frontier)
            # [::-1] reverses the array
            # https://stackoverflow.com/questions/16486252/is-it-possible-to-use-argsort-in-descending-order
            sorted_local_occupancy_value_idxs = occupancy_value.argsort()[::-1]
            self.sorted_accessible_local_candidates = local_accessible_candidates[sorted_local_occupancy_value_idxs]
            sorted_local_occupancy_value = occupancy_value[sorted_local_occupancy_value_idxs]

            # At the beginning, when all values are uncertain, we add some hardcoded candidates so it begins to navigate
            # and has time to discover accessible areas
            # if np.size(self.sorted_accessible_local_candidates) == 0:
            #     self.sorted_accessible_local_candidates = np.array([[1.5, 0.0], [0.0, 1.5], [-1.5, 0.0], [0.0, -1.5]])

            for i in range(self.sorted_accessible_local_candidates.shape[0]):
                candidate = self.sorted_accessible_local_candidates[i]
                if i == 0:
                    size = 3
                elif i <= 5:
                    size = 2
                elif i <= 10:
                    size = 1
                else:
                    size = 0
                image[candidate[1]-size:candidate[1]+size,candidate[0]-size:candidate[0]+size] = 100 # + round(sorted_local_occupancy_value[i] * 2) + 20  #sorted_local_occupancy_value[i]*2

            cmap = 'gray_r'
            origin='lower'
            plt.figimage(image, cmap=cmap, origin=origin)
            self.fig.canvas.draw()
            plt.pause(0.01)
            # Once we have the new candidates, 
            # they are saved in self.sorted_accessible_local_candidates for use by the Navigator client
            # self.get_logger().info('Accessible candidates have been updated...')

            # To robot coordination
            # self.get_logger().info(f"-----------------------------")
            msg = PoseArray()
            for i in range(min(self.sorted_accessible_local_candidates.shape[0], 10)):
                candidate = self.sorted_accessible_local_candidates[i]
                new_position_x = candidate[0] * resolution + shiftX
                new_position_y = candidate[1] * resolution + shiftY
                new_p = Pose()
                new_p.position.x = new_position_x
                new_p.position.y = new_position_y
                # new_p.position.z = 0.0
                msg.poses.append(new_p)
                # self.get_logger().info(f"-----{robot_position_x},{robot_position_y}")
                # self.get_logger().info(f"-----{candidate[0]},{candidate[1]}")
                # self.get_logger().info(f"-----{self.robot_position.x},{self.robot_position.y}")
                # self.get_logger().info(f"-----{round(new_position_x,2)},{round(new_position_y,2)}")
            
            self.my_local_candidates_publisher.publish(msg)
        # import time
        # time.sleep(3)
        self.busy_occupancy_callback=False

    def odom_callback(self, msg):      
        pose = msg.pose.pose
        position = pose.position
        # orientation = pose.orientation
        self.robot_position = position
        # self.get_logger().info(f'self.robot_position={self.robot_position}')  

    @staticmethod
    def convolute(data, coordinates, size=3): # , threshold=40
        """
        This function calculates the average occupancy probability at 'coordinates' for an area of size (size x size)
        around said point.

        :param data: Occupancy Grid Data (shaped to (x, y) map dimensions)
        :param coordinate: the coordinate (x,y) of the OccupancyGrid to convolute around 
        :param size: size of the kernel
        :param threshold: threshold of accessibility
        :return: True or False, depending on whether the candidate is accessible or not.
        :return: worth: The worth level
        """
        worth = 0
        isKnownCenter = False
        adjacentWall = False
        centerOccupied = data[coordinates[1], coordinates[0]]
        if centerOccupied >= 0 and centerOccupied <= 10:
            isKnownCenter = True

        if isKnownCenter:
            for y in range(round(coordinates[1] - size / 2), round(coordinates[1] + size / 2)):
                for x in range(round(coordinates[0] - size / 2), round(coordinates[0] + size / 2)):
                    # if the area is unknown, we add 100 to worth.
                    if data[y, x] == -1:
                        worth += 100
                    # access areas near walls.
                    elif data[y, x] > 50:
                        adjacentWall = True
                        worth += 0
                        break
                    else:
                        worth += 1 if worth > 0 else 0

        if worth > 0 and isKnownCenter and not adjacentWall:
            return True, worth
        else:
            return False, worth

    def generate_list_of_candidates(self, candidates_radius_num, step):
        """

        Generates a grid of candidates of size ('candidates_radius_num*2' * 'candidates_radius_num*2') and step size 'step'

        :param candidates_radius_num: number of total candidates to generate per side
        :param step: float resolution of the candidates
        :return candidates: 2D numpy array of a list of coordinates of size dim x 2,
        where dim is the number of candidates [x1,y1,x2,y2......]
        """
        diameter = candidates_radius_num * 2 + 1
        candidates = np.zeros((diameter ** 2 , 2))

        i = 0
        radius = candidates_radius_num * step
        # center_shift = edge/2
        for index_y in range(diameter):
            for index_x in range(diameter):
                candidates[i] = [float(index_x) * step - radius, float(index_y)*step - radius]
                i += 1

        self.get_logger().info(f"Grid of {i} candidates has been generated. step={step}")
        return candidates

def main(args=None):
    rclpy.init(args=args)

    map_analyzer = MapAnalyzer()

    rclpy.spin(map_analyzer)

    map_analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()