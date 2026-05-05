#!/usr/bin/env python3
"""
Task Scheduler Node for Seeway Robot
Manages task queue and coordinates navigation and execution
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from seeway_task_msgs.msg import TaskCommand, TaskStatus
from geometry_msgs.msg import PoseStamped
import json
from collections import deque
from datetime import datetime
import math


class TaskSchedulerNode(Node):
    """
    Core task scheduler that manages task queue and coordinates
    navigation and execution
    
    Task state machine:
    WAITING -> NAVIGATING_TO_TARGET -> EXECUTING -> COMPLETED
                                                  \-> FAILED
    """

    def __init__(self):
        super().__init__('task_scheduler_node')

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # QoS Profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publishers
        self.status_publisher = self.create_publisher(
            TaskStatus,
            '/task_status_feedback',
            qos_profile
        )

        # Subscribers
        self.task_subscriber = self.create_subscription(
            TaskCommand,
            '/sys_task_cmd',
            self.task_callback,
            qos_profile,
            callback_group=self.callback_group
        )

        # Task queue and state
        self.task_queue = deque()
        self.current_task = None
        self.task_states = {}  # {task_id: state}
        
        # Predefined cleaning locations (x, y, theta, duration)
        self.cleaning_locations = {
            'toilet': {
                'start_point': (0.0, 0.0),
                'target_point': (1.0, 1.0),
                'theta': 0.0,
                'duration': 120  # 2 minutes
            },
            'sink': {
                'start_point': (0.0, 0.0),
                'target_point': (2.0, 2.0),
                'theta': 0.0,
                'duration': 90  # 1.5 minutes
            },
            'urinal': {
                'start_point': (0.0, 0.0),
                'target_point': (3.0, 1.0),
                'theta': 0.0,
                'duration': 60  # 1 minute
            },
            'floor': {
                'start_point': (0.0, 0.0),
                'target_point': (1.5, 1.5),
                'theta': 0.0,
                'duration': 180  # 3 minutes
            },
            'wall': {
                'start_point': (0.0, 0.0),
                'target_point': (2.5, 2.5),
                'theta': 0.0,
                'duration': 90  # 1.5 minutes
            }
        }

        # Task statistics
        self.task_stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'start_time': datetime.now()
        }

        # Declare parameters
        self.declare_parameter('state_check_period', 0.1)  # 100ms
        self.declare_parameter('goal_threshold', 0.2)
        self.declare_parameter('nav_timeout', 60.0)

        # Get parameters
        state_check_period = self.get_parameter('state_check_period').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.nav_timeout = self.get_parameter('nav_timeout').value

        # State check timer
        self.create_timer(
            state_check_period,
            self.state_checker,
            callback_group=self.callback_group
        )

        # Task state
        self.task_start_time = None
        self.nav_start_time = None
        self.task_current_progress = 0

        self.get_logger().info(
            f'Task Scheduler Node initialized\n'
            f'  Cleaning locations: {list(self.cleaning_locations.keys())}\n'
            f'  State check period: {state_check_period}s\n'
            f'  Goal threshold: {self.goal_threshold}m\n'
            f'  Navigation timeout: {self.nav_timeout}s'
        )

    def task_callback(self, msg: TaskCommand):
        """
        Handle incoming task commands
        
        Args:
            msg: TaskCommand message
        """
        # Parse task parameters
        try:
            if msg.param and msg.param != "none":
                param = json.loads(msg.param)
            else:
                param = {}
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse task param: {msg.param}")
            param = {}

        # Create task object
        task = {
            'task_id': f"{len(self.task_queue):04d}",
            'own': msg.own,
            'task_name': msg.task,
            'param': param,
            'status': 'WAITING',
            'progress': 0,
            'created_at': datetime.now(),
            'location': param.get('location', 'unknown')
        }

        # Add to queue
        self.task_queue.append(task)
        self.task_stats['total_tasks'] += 1

        self.get_logger().info(
            f"Task added to queue: {task['task_name']} "
            f"at location {task['location']} (Queue size: {len(self.task_queue)})"
        )
        
        # Publish status
        self.publish_task_status(task, 'WAITING', 0, 'Task added to queue')

    def state_checker(self):
        """
        Periodic state checker - main state machine logic
        Runs every 100ms
        """
        # If no current task, try to start next one
        if self.current_task is None:
            if len(self.task_queue) > 0:
                self.start_next_task()
            return

        # Handle current task based on its status
        current_status = self.current_task['status']

        if current_status == 'WAITING':
            # Move to navigation
            self.current_task['status'] = 'NAVIGATING_TO_TARGET'
            self.nav_start_time = datetime.now()
            self.get_logger().info(
                f"Task {self.current_task['task_id']}: "
                f"Starting navigation to {self.current_task['location']}"
            )
            self.publish_task_status(
                self.current_task,
                'NAVIGATING_TO_TARGET',
                10,
                f"Navigating to {self.current_task['location']}"
            )

        elif current_status == 'NAVIGATING_TO_TARGET':
            # Check if navigation completed
            if self.check_navigation_completed():
                self.current_task['status'] = 'EXECUTING'
                self.task_start_time = datetime.now()
                self.task_current_progress = 0
                self.get_logger().info(
                    f"Task {self.current_task['task_id']}: "
                    f"Navigation complete, starting execution"
                )
                self.publish_task_status(
                    self.current_task,
                    'EXECUTING',
                    30,
                    f"Executing cleaning task at {self.current_task['location']}"
                )
            else:
                # Check navigation timeout
                nav_elapsed = (datetime.now() - self.nav_start_time).total_seconds()
                if nav_elapsed > self.nav_timeout:
                    self.current_task['status'] = 'FAILED'
                    self.get_logger().error(
                        f"Task {self.current_task['task_id']}: "
                        f"Navigation timeout after {nav_elapsed:.1f}s"
                    )
                    self.publish_task_status(
                        self.current_task,
                        'FAILED',
                        0,
                        f"Navigation timeout"
                    )
                    self.complete_current_task(False)
                else:
                    # Update progress based on time
                    progress = int((nav_elapsed / self.nav_timeout) * 30) + 10
                    self.current_task['progress'] = min(progress, 50)
                    self.publish_task_status(
                        self.current_task,
                        'NAVIGATING_TO_TARGET',
                        self.current_task['progress'],
                        f"Navigating to {self.current_task['location']}..."
                    )

        elif current_status == 'EXECUTING':
            # Check if execution completed
            location = self.current_task['location']
            if location in self.cleaning_locations:
                duration = self.cleaning_locations[location]['duration']
            else:
                duration = 120  # Default 2 minutes

            elapsed = (datetime.now() - self.task_start_time).total_seconds()
            progress = int((elapsed / duration) * 70) + 30  # Progress from 30-100%

            if elapsed > duration:
                self.current_task['status'] = 'COMPLETED'
                self.current_task['progress'] = 100
                self.get_logger().info(
                    f"Task {self.current_task['task_id']}: "
                    f"Execution completed in {elapsed:.1f}s"
                )
                self.publish_task_status(
                    self.current_task,
                    'COMPLETED',
                    100,
                    f"Task completed successfully"
                )
                self.complete_current_task(True)
            else:
                # Update progress
                self.current_task['progress'] = min(progress, 99)
                self.publish_task_status(
                    self.current_task,
                    'EXECUTING',
                    self.current_task['progress'],
                    f"Executing... ({elapsed:.1f}/{duration}s)"
                )

    def start_next_task(self):
        """
        Start the next task in the queue
        """
        if len(self.task_queue) == 0:
            return

        self.current_task = self.task_queue.popleft()
        self.current_task['status'] = 'WAITING'
        self.current_task['progress'] = 5

        self.get_logger().info(
            f"Starting task: {self.current_task['task_name']} "
            f"(ID: {self.current_task['task_id']})"
        )
        self.publish_task_status(
            self.current_task,
            'WAITING',
            5,
            'Task processing started'
        )

    def check_navigation_completed(self) -> bool:
        """
        Check if navigation to target has completed
        This is a simplified check - in real implementation,
        subscribe to actual robot pose feedback
        
        Returns:
            True if navigation completed
        """
        # Simplified simulation: Check timeout
        if self.nav_start_time is None:
            return False

        nav_elapsed = (datetime.now() - self.nav_start_time).total_seconds()
        
        # Simulated navigation takes 5-15 seconds depending on location
        location = self.current_task['location']
        nav_time = 10.0 if location in self.cleaning_locations else 15.0
        
        return nav_elapsed > nav_time

    def complete_current_task(self, success: bool):
        """
        Mark current task as completed
        
        Args:
            success: True if task completed successfully
        """
        if success:
            self.task_stats['completed_tasks'] += 1
        else:
            self.task_stats['failed_tasks'] += 1

        self.get_logger().info(
            f"Task {self.current_task['task_id']} completed. "
            f"Stats: {self.task_stats['completed_tasks']} completed, "
            f"{self.task_stats['failed_tasks']} failed out of "
            f"{self.task_stats['total_tasks']} total"
        )
        
        self.current_task = None
        self.task_start_time = None
        self.nav_start_time = None

    def publish_task_status(self, task: dict, status: str, progress: int, message: str):
        """
        Publish task status feedback
        
        Args:
            task: Task dictionary
            status: Task status string
            progress: Progress percentage (0-100)
            message: Status message
        """
        msg = TaskStatus()
        msg.task_id = task['task_id']
        msg.status = status
        msg.progress = progress
        msg.message = message

        self.status_publisher.publish(msg)

    def get_task_statistics(self) -> dict:
        """
        Get task execution statistics
        
        Returns:
            Dictionary with task statistics
        """
        uptime = (datetime.now() - self.task_stats['start_time']).total_seconds()
        return {
            'total_tasks': self.task_stats['total_tasks'],
            'completed_tasks': self.task_stats['completed_tasks'],
            'failed_tasks': self.task_stats['failed_tasks'],
            'queue_size': len(self.task_queue),
            'uptime_seconds': uptime,
            'tasks_per_hour': self.task_stats['completed_tasks'] / (uptime / 3600.0) if uptime > 0 else 0
        }


def main(args=None):
    rclpy.init(args=args)
    node = TaskSchedulerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stats = node.get_task_statistics()
        node.get_logger().info(
            f"Final statistics:\n"
            f"  Total tasks: {stats['total_tasks']}\n"
            f"  Completed: {stats['completed_tasks']}\n"
            f"  Failed: {stats['failed_tasks']}\n"
            f"  Uptime: {stats['uptime_seconds']:.1f}s\n"
            f"  Throughput: {stats['tasks_per_hour']:.2f} tasks/hour"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
