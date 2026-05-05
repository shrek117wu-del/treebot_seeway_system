#!/usr/bin/env python3
"""
Nav2 Client Node for Seeway Robot
Sends navigation goals to Nav2 and monitors navigation status
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class Nav2ClientNode(Node):
    """
    Nav2 navigation client
    Sends NavigateToPose goals and monitors feedback
    """

    def __init__(self):
        super().__init__('nav2_client_node')

        # Callback group for action client
        self.callback_group = ReentrantCallbackGroup()

        # Action client
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Declare parameters
        self.declare_parameter('goal_threshold', 0.2)
        self.declare_parameter('planning_timeout', 60.0)
        self.declare_parameter('execution_timeout', 300.0)

        # Get parameters
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value

        # State
        self.nav_state = 'IDLE'  # IDLE, PLANNING, EXECUTING, SUCCEEDED, FAILED
        self.current_goal_id = None
        self.current_goal_pose = None
        self.send_goal_handle = None

        self.get_logger().info(
            f'Nav2 Client Node initialized\n'
            f'  Goal threshold: {self.goal_threshold} m\n'
            f'  Planning timeout: {self.planning_timeout} s\n'
            f'  Execution timeout: {self.execution_timeout} s'
        )

    def wait_for_server(self, timeout: float = 10.0) -> bool:
        """
        Wait for Nav2 action server to be available
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if server is available, False if timeout
        """
        return self.nav_to_pose_client.wait_for_server(timeout_sec=timeout)

    def create_pose_from_xy(self, x: float, y: float, theta: float = 0.0) -> PoseStamped:
        """
        Create a PoseStamped message from x, y coordinates and orientation
        
        Args:
            x: X coordinate
            y: Y coordinate
            theta: Orientation angle in radians (default: 0.0)
            
        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        # Simple conversion: theta -> (0, 0, sin(theta/2), cos(theta/2))
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose

    def send_navigation_goal(self, goal_pose: PoseStamped, goal_id: str = "") -> bool:
        """
        Send a navigation goal to Nav2
        
        Args:
            goal_pose: Target pose as PoseStamped
            goal_id: Unique goal identifier
            
        Returns:
            True if goal was sent, False if failed
        """
        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().error('Nav2 server is not ready')
            self.nav_state = 'FAILED'
            return False

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.behavior_tree = ''

        # Store current goal
        self.current_goal_id = goal_id if goal_id else str(id(goal))
        self.current_goal_pose = goal_pose
        self.nav_state = 'PLANNING'

        # Send goal asynchronously
        self.get_logger().info(
            f'Sending navigation goal to ({goal_pose.pose.position.x:.2f}, '
            f'{goal_pose.pose.position.y:.2f}) - Goal ID: {self.current_goal_id}'
        )

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        """
        Callback for goal response from Nav2
        
        Args:
            future: Future object with goal handle
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.nav_state = 'FAILED'
            return

        self.get_logger().info('Goal accepted by Nav2')
        self.send_goal_handle = goal_handle
        self.nav_state = 'EXECUTING'

        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback from Nav2
        
        Args:
            feedback_msg: Feedback message from navigation
        """
        feedback = feedback_msg.feedback
        
        # Calculate distance to goal
        current_pose = feedback.current_pose.pose
        goal_pose = self.current_goal_pose.pose
        
        distance = math.sqrt(
            (current_pose.position.x - goal_pose.position.x) ** 2 +
            (current_pose.position.y - goal_pose.position.y) ** 2
        )

        self.get_logger().debug(
            f'Navigation feedback: distance to goal = {distance:.2f}m'
        )

    def get_result_callback(self, future):
        """
        Handle navigation result from Nav2
        
        Args:
            future: Future object with navigation result
        """
        result = future.result()

        if result.result:
            self.get_logger().info(
                f'Navigation succeeded for goal {self.current_goal_id}'
            )
            self.nav_state = 'SUCCEEDED'
        else:
            self.get_logger().warn(
                f'Navigation failed for goal {self.current_goal_id}'
            )
            self.nav_state = 'FAILED'

    def cancel_navigation(self) -> bool:
        """
        Cancel current navigation goal
        
        Returns:
            True if cancellation was requested
        """
        if self.send_goal_handle is None:
            self.get_logger().warn('No active goal to cancel')
            return False

        self.get_logger().info(f'Cancelling navigation goal {self.current_goal_id}')
        
        cancel_future = self.send_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

        return True

    def cancel_done_callback(self, future):
        """
        Callback for goal cancellation
        
        Args:
            future: Future object
        """
        cancel_response = future.result()
        
        if cancel_response.return_code == 0:  # SUCCEED
            self.get_logger().info('Goal cancellation successful')
            self.nav_state = 'IDLE'
        else:
            self.get_logger().warn('Goal cancellation failed')

    def check_goal_reached(self) -> bool:
        """
        Check if current goal has been reached within threshold
        
        Returns:
            True if goal is reached within threshold
        """
        if self.current_goal_pose is None or self.send_goal_handle is None:
            return False

        # Get feedback from last navigation
        # Note: This is a simplified check. In real implementation,
        # you should maintain the latest feedback state
        if self.nav_state == 'SUCCEEDED':
            return True

        return False

    def is_navigating(self) -> bool:
        """
        Check if navigation is currently in progress
        
        Returns:
            True if navigating, False otherwise
        """
        return self.nav_state in ['PLANNING', 'EXECUTING']

    def get_navigation_state(self) -> str:
        """
        Get current navigation state
        
        Returns:
            Navigation state string
        """
        return self.nav_state

    def send_navigation_goal_xy(self, x: float, y: float, theta: float = 0.0, 
                                goal_id: str = "") -> bool:
        """
        Convenience method to send navigation goal using x, y coordinates
        
        Args:
            x: Target X coordinate
            y: Target Y coordinate
            theta: Target orientation in radians
            goal_id: Unique goal identifier
            
        Returns:
            True if goal was sent successfully
        """
        goal_pose = self.create_pose_from_xy(x, y, theta)
        return self.send_navigation_goal(goal_pose, goal_id)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ClientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
