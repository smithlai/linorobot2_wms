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
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid, Odometry

# 3rd party
import numpy as np
import matplotlib.pyplot as plt

class SlamToolboxSubscriber(Node):
    def __init__(self):
        super().__init__('slamtoolbox_subscriber')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('candidates_per_axis', 40),
                ('step_of_candidates', 0.5),
                ('map_topic', 'map1'),
                ('odom_topic', 'odom1')
            ])
        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.occupancy_subscription = self.create_subscription(OccupancyGrid, map_topic, self.occupancy_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        candidates_per_axis = self.get_parameter('candidates_per_axis').value
        step_of_candidates = self.get_parameter('step_of_candidates').value
        self.robot_position = None
        # create a 100*100 map, each axis coordinate increased by 0.2m, that is [0.0,0.2,0.4...19.8] 
        # ([(x1,y1),(x2,y2)......])
        self.nearby_candidates = self.generate_list_of_candidates(candidates_per_axis=candidates_per_axis, step=step_of_candidates)
        self.sorted_accessible_candidates = np.array([])

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible candidates. It sorts them and
        saves them in the self.sorted_accessible_candidates variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """

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
            robot_position_x = int((self.robot_position.x-shiftX) / resolution)
            robot_position_y = int((self.robot_position.y-shiftY) / resolution)
            image[robot_position_y-4:robot_position_y+4,robot_position_x-4:robot_position_x+4] = 100
        # for j in range(current_map_height):
        #     for i in range(current_map_width):
        #         if image[j,i] != 0:
        #             self.get_logger().info(f"{image[j,i]}")

        # https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib
        cmap = 'gray_r'
        origin='lower'
        # plt.figimage(image, cmap=cmap, origin=origin)
        # plt.draw()
        # plt.pause(0.2)


        if not self.robot_position:
            pass
        else:
            # Here we go through every candidate and save the ones that are accessible.
            # An accessible candidate is one which has no obstacles, and has few or no unknown squares in the vicinity.
            accessible_candidates = np.array([], dtype=np.int32) #([(x1,y1),(x2,y2)......])
            occupancy_value = np.array([])
            for candidate in self.nearby_candidates: #([(x1,y1),(x2,y2)......])
                try:
                    occupancy_grid_coordinate = [int(candidate[0] / resolution) + robot_position_x, int(candidate[1] / resolution)+robot_position_y]
                    conv_accessable, conv_avg= self.convolute(data, occupancy_grid_coordinate, size=9, threshold=40)  # perform convolution

                    # if the convolution returns True, it means the WP is accessible (under threshold), 
                    # so it is stored in accessible_candidates
                    if conv_accessable:
                        accessible_candidates = np.append(accessible_candidates, occupancy_grid_coordinate)
                        occupancy_value = np.append(occupancy_value, conv_avg)
                    else:
                        pass
                # because the candidate array is over-sized, we need to remove the values that are out of range
                except IndexError:
                    pass
            # reshape the accessible candidates array to shape (n, 2)
            accessible_candidates = accessible_candidates.reshape((-1, 2))  #([(x1,y1),(x2,y2)......])
            # Sorting candidates according to occupancy value. This allows the robot to prioritize the candidates with
            # more uncertainty (it wont access the areas that are completely clear, thus going to the discovery frontier)
            # [::-1] reverses the array
            # https://stackoverflow.com/questions/16486252/is-it-possible-to-use-argsort-in-descending-order
            sorted_occupancy_value_idxs = occupancy_value.argsort()[::-1]
            self.sorted_accessible_candidates = accessible_candidates[sorted_occupancy_value_idxs]
            sorted_occupancy_value = occupancy_value[sorted_occupancy_value_idxs]

            # At the beginning, when all values are uncertain, we add some hardcoded candidates so it begins to navigate
            # and has time to discover accessible areas
            # if np.size(self.sorted_accessible_candidates) == 0:
            #     self.sorted_accessible_candidates = np.array([[1.5, 0.0], [0.0, 1.5], [-1.5, 0.0], [0.0, -1.5]])

            for i in range(self.sorted_accessible_candidates.shape[0]):
                candidate = self.sorted_accessible_candidates[i]
                image[candidate[1],candidate[0]] = 100  #sorted_occupancy_value[i]*2

            plt.figimage(image, cmap=cmap, origin=origin)
            plt.draw()
            plt.pause(0.1)
            # Once we have the new candidates, 
            # they are saved in self.sorted_accessible_candidates for use by the Navigator client
            self.get_logger().info('Accessible candidates have been updated...')

    def odom_callback(self, msg):      
        pose = msg.pose.pose
        position = pose.position
        # orientation = pose.orientation
        self.robot_position = position        

    @staticmethod
    def convolute(data, coordinates, size=3, threshold=40):
        """
        This function calculates the average occupancy probability at 'coordinates' for an area of size (size x size)
        around said point.

        :param data: Occupancy Grid Data (shaped to (x, y) map dimensions)
        :param coordinate: the coordinate (x,y) of the OccupancyGrid to convolute around 
        :param size: size of the kernel
        :param threshold: threshold of accessibility
        :return: True or False, depending on whether the candidate is accessible or not.
        :return: average: average occupancy probability of the convolution
        """
        sum = 0
        for y in range(int(coordinates[1] - size / 2), int(coordinates[1] + size / 2)):
            for x in range(int(coordinates[0] - size / 2), int(coordinates[0] + size / 2)):
                # if the area is unknown, we add 100 to sum.
                if data[y, x] == -1:
                    sum += 100
                # if occupancy state is above 50 (occupied), we add 1M to the sum so that the robot DOES NOT
                # access areas near walls.
                elif data[y, x] > 50:
                    sum += 1000000
                elif data[y, x] == 0:
                    sum += (abs(x) + abs(y))*0.01
                # if the occupancy state is below 50 and known, just add the value to sum.
                else:
                    sum += data[y, x]

        # average value for the square is computed
        average = sum / (size * size)
        if average < threshold:
            # if the average of the squares is below the threshold, the candidate is accessible
            return True, average
        else:
            # if the average is above the threshold, the candidate has either too many unknowns, or an obstacle
            return False, average

    def generate_list_of_candidates(self, candidates_per_axis, step):
        """

        Generates a grid of candidates of size ('candidates_per_axis' * 'candidates_per_axis') and step size 'step'

        :param candidates_per_axis: number of total candidates to generate per side
        :param step: float resolution of the candidates
        :return candidates: 2D numpy array of a list of coordinates of size dim x 2,
        where dim is the number of candidates [x1,y1,x2,y2......]
        """

        candidates = np.zeros((candidates_per_axis * candidates_per_axis, 2))

        i = 0
        edge = candidates_per_axis * step
        center_shift = edge/2
        for index_y in range(candidates_per_axis):
            for index_x in range(candidates_per_axis):
                candidates[i] = [float(index_x) * step - center_shift, float(index_y)*step - center_shift]
                i += 1

        self.get_logger().info(f"Grid of {i} candidates has been generated.")
        return candidates

def main(args=None):
    rclpy.init(args=args)

    slamToolboxSubscriber = SlamToolboxSubscriber()

    rclpy.spin(slamToolboxSubscriber)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()