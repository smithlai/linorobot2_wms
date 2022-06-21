# ROS client library for Python
import rclpy
import time # Time library
import subprocess
# Used to create nodes
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration # Handles time for ROS 2
#from nav2_msgs.action import NavigateToPose
from myautomapping.robot_navigator_tst import BasicNavigator, NavigationResult

sorted_local_candidates = []
stop_discovering = False
cand_empty_count = 0


class ReceiveLocalCandidate(Node):
    """
    Subscriber node to the current battery state
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('local_candidate_subscriber')
    
      #self.sorted_local_candidates = []
      # Create a subscriber 
      # This node subscribes to messages of type
      # sensor_msgs/BatteryState
      self.subscription_battery_state = self.create_subscription(
        PoseArray,
        'local_candidate_topic',
        self.local_candidate_callback,
        10)
      
      self.startT = time.time()
      
    def local_candidate_callback(self, msg):
        global sorted_local_candidates
        global cand_empty_count
        global stop_discovering
        poses = msg.poses
        sorted_local_candidates = poses
        if stop_discovering == False:
            if len(sorted_local_candidates) == 0:
                cand_empty_count = cand_empty_count + 1
                if cand_empty_count == 3:
                    #save map
                    stop_discovering = True
                    deltaT = time.time() - self.startT
                    self.get_logger().info(f"Elapsed Time {deltaT},Save the map!!")
                    subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "wms2","--ros-args","-p","save_map_timeout:=10000.0"])
            else:
                cand_empty_count = 0

            deltaT = time.time() - self.startT
            self.get_logger().info(f"sorted_local_candidates UPDATED: {len(sorted_local_candidates)}")
            self.get_logger().info(f"candidates empty UPDATED: {cand_empty_count}")
            self.get_logger().info(f"elapsed: {deltaT}")

class MySendGoal(Node):
    def __init__(self):
  
        # Initialize the class using the constructor
        super().__init__('mygoal')
        timer_period = 1.0
        self.abort_cnt=0
        self.timer = self.create_timer(timer_period, self.send_goal)
        self.startT = time.time()
        
    def send_goal(self):
        global stop_discovering
        global sorted_local_candidates
      
        if stop_discovering == True:
            return
        else:
            self.get_logger().info('create_goal...')
            next_candidate=None
            try:
                next_candidate = sorted_local_candidates.pop(0)
               
                self.get_logger().warning(f'candidate = {len(sorted_local_candidates)}, new goal = {next_candidate.position.x},{next_candidate.position.y}')
                # goal_msg.pose.pose.orientation.w = 1.0 # default
            except Exception as e:
                self.get_logger().warning(f'Error while pop sorted_local_candidates:{e}')
        
            if next_candidate != None:    
                self.get_logger().info('send_goal...')
                # Launch the ROS 2 Navigation Stack
                navigator = BasicNavigator()
                # Wait for navigation to fully activate. Use this line if autostart is set to true.
                navigator.waitUntilNav2Active()
                # Set the robot's goal pose
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose = next_candidate
        
                # Go to the goal pose
                navigator.goToPose(goal_pose)
        
                i = 0
                nav_start = navigator.get_clock().now()
                while not navigator.isNavComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()
                    #if feedback and i % 5 == 0:
                    #    self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                    #    feedback.distance_remaining) + ' meters.')
                    # Some navigation timeout to demo cancellation
                    now = navigator.get_clock().now()
                    if now - nav_start > Duration(seconds=700.0):
                        self.get_logger().info('Goal was canceled due to timeout!!')
                        navigator.cancelNav()
                        stop_discovering = True
              
                result = navigator.getResult()
                if result == NavigationResult.SUCCEEDED:
                    self.get_logger().info('Successfully reached the goal...')
                    self.abort_cnt=0
                elif result == NavigationResult.CANCELED:
                    self.get_logger().info('Goal was canceled!')
                    if stop_discovering == True:
                        deltaT = time.time() - self.startT
                        self.get_logger().info(f'Elapsed Time {deltaT},Save Map')
                        subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "wms2","--ros-args","-p","save_map_timeout:=10000.0"])
                    
                    self.abort_cnt=0
                    
                elif result == NavigationResult.FAILED:
                    self.abort_cnt = self.abort_cnt + 1
                    if self.abort_cnt == 10:
                        stop_discovering = True
                        self.get_logger().info('Goal failed 10 times!!')
                        #save the map
                        deltaT = time.time() - self.startT
                        self.get_logger().info(f'Elapsed Time {deltaT},Save Map')
                        subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "wms2","--ros-args","-p","save_map_timeout:=10000.0"])
                    else:
                        self.get_logger().info('Goal failed!')
                else:
                    self.get_logger().info('Goal has an invalid return status!') 
            
def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)
     
    try:
    # Create the node
        revcade = ReceiveLocalCandidate()
        my_goal = MySendGoal()
 
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    #rclpy.spin(revcade)
    
    # Set up mulithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(revcade)
        executor.add_node(my_goal)
 
        try:
    #  # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
      # Shutdown the nodes
            executor.shutdown()
            revcade.destroy_node()
            my_goal.destroy_node()

    finally:
  #  # Shutdown
        rclpy.shutdown()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #revcade.destroy_node()
     
    # Shutdown the ROS client library for Python
    #rclpy.shutdown()
 
if __name__ == '__main__':
    main()
