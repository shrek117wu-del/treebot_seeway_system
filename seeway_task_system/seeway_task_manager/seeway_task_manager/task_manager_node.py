#!/usr/bin/env python3
"""
Task Manager Node for Seeway Robot
Publishes task commands and manages task history
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import uuid
from datetime import datetime
from seeway_task_msgs.msg import TaskCommand, TaskStatus


class TaskManagerNode(Node):
    """
    Manages task creation, validation, and publication
    Publishes to: /sys_task_cmd
    Subscribes to: /task_status_feedback
    """

    def __init__(self):
        super().__init__('task_manager_node')

        # QoS Profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publishers
        self.task_publisher = self.create_publisher(
            TaskCommand,
            '/sys_task_cmd',
            qos_profile
        )

        # Subscribers
        self.status_subscriber = self.create_subscription(
            TaskStatus,
            '/task_status_feedback',
            self.status_callback,
            qos_profile
        )

        # Task tracking
        self.task_history = {}  # {task_id: {task_info}}
        self.active_tasks = {}  # {task_id: status}

        # Configuration
        self.max_task_history = 100
        self.valid_owners = ['vla', 'manual', 'autonomous']
        self.valid_tasks = [
            'clean_toilet',
            'clean_sink',
            'clean_urinal',
            'clean_floor',
            'clean_wall',
            'clean_task1',
            'clean_task2',
            'clean_all'
        ]

        # Valid cleaning locations
        self.valid_locations = ['toilet', 'sink', 'urinal', 'floor', 'wall']

        self.get_logger().info('Task Manager Node initialized')

    def validate_task_command(self, cmd: TaskCommand) -> tuple:
        """
        Validate task command structure and content
        
        Returns:
            (is_valid, error_message)
        """
        errors = []

        # Check owner
        if not cmd.own:
            errors.append("Owner field is empty")
        elif cmd.own not in self.valid_owners:
            errors.append(f"Invalid owner: {cmd.own}. Valid: {self.valid_owners}")

        # Check task name
        if not cmd.task:
            errors.append("Task name is empty")
        elif cmd.task not in self.valid_tasks:
            errors.append(f"Invalid task: {cmd.task}. Valid: {self.valid_tasks}")

        # Validate param JSON
        if cmd.param and cmd.param != "none":
            try:
                param_dict = json.loads(cmd.param)
                
                # Validate location if present
                if 'location' in param_dict:
                    if param_dict['location'] not in self.valid_locations:
                        errors.append(
                            f"Invalid location: {param_dict['location']}. "
                            f"Valid: {self.valid_locations}"
                        )
                
                # Validate force if present (0-100)
                if 'force' in param_dict:
                    force = param_dict['force']
                    if not isinstance(force, (int, float)) or force < 0 or force > 100:
                        errors.append(f"Force must be 0-100, got {force}")
                        
            except json.JSONDecodeError as e:
                errors.append(f"Invalid JSON in param: {str(e)}")

        return (len(errors) == 0, errors)

    def create_task_command(self, own: str, task: str, param: str = "none") -> TaskCommand:
        """
        Create a TaskCommand message
        
        Args:
            own: Owner identifier (vla, manual, autonomous)
            task: Task name
            param: JSON formatted parameter string
            
        Returns:
            TaskCommand message
        """
        cmd = TaskCommand()
        cmd.own = own
        cmd.task = task
        cmd.param = param
        return cmd

    def create_task_from_json(self, json_str: str) -> TaskCommand:
        """
        Create TaskCommand from JSON string
        
        Args:
            json_str: JSON string containing {own, task, param}
            
        Returns:
            TaskCommand message or None if invalid
        """
        try:
            data = json.loads(json_str)
            
            cmd = TaskCommand()
            cmd.own = data.get('own', '')
            cmd.task = data.get('task', '')
            cmd.param = data.get('param', 'none')
            
            return cmd
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {str(e)}")
            return None

    def publish_task(self, cmd: TaskCommand) -> bool:
        """
        Validate and publish a task command
        
        Args:
            cmd: TaskCommand message
            
        Returns:
            True if published successfully, False otherwise
        """
        # Validate
        is_valid, errors = self.validate_task_command(cmd)
        
        if not is_valid:
            error_msg = "; ".join(errors)
            self.get_logger().error(f"Task validation failed: {error_msg}")
            return False

        # Generate task ID
        task_id = str(uuid.uuid4())[:8]
        
        # Publish
        self.task_publisher.publish(cmd)
        
        # Track task
        self.task_history[task_id] = {
            'command': {
                'own': cmd.own,
                'task': cmd.task,
                'param': cmd.param
            },
            'timestamp': datetime.now().isoformat(),
            'task_id': task_id,
            'status': 'PUBLISHED'
        }
        self.active_tasks[task_id] = 'PUBLISHED'
        
        self.get_logger().info(
            f"Task published - ID: {task_id}, "
            f"Task: {cmd.task}, Owner: {cmd.own}"
        )
        
        return True

    def publish_task_from_json(self, json_str: str) -> bool:
        """
        Create and publish task from JSON string
        
        Args:
            json_str: JSON string
            
        Returns:
            True if successful
        """
        cmd = self.create_task_from_json(json_str)
        if cmd is None:
            return False
        
        return self.publish_task(cmd)

    def status_callback(self, msg: TaskStatus):
        """
        Handle task status feedback
        
        Args:
            msg: TaskStatus message
        """
        task_id = msg.task_id
        
        if task_id in self.task_history:
            self.task_history[task_id]['status'] = msg.status
            self.task_history[task_id]['progress'] = msg.progress
            self.task_history[task_id]['message'] = msg.message
            
            if task_id in self.active_tasks:
                self.active_tasks[task_id] = msg.status
                
                if msg.status in ['SUCCEEDED', 'FAILED']:
                    del self.active_tasks[task_id]

            self.get_logger().info(
                f"Task {task_id} status: {msg.status} ({msg.progress}%) - {msg.message}"
            )

    def get_task_status(self, task_id: str) -> dict:
        """Get status of a specific task"""
        return self.task_history.get(task_id, None)

    def list_active_tasks(self) -> list:
        """Get list of active tasks"""
        return list(self.active_tasks.items())

    def list_task_history(self, limit: int = 10) -> list:
        """Get recent task history"""
        sorted_tasks = sorted(
            self.task_history.items(),
            key=lambda x: x[1]['timestamp'],
            reverse=True
        )
        return sorted_tasks[:limit]

    def cleanup_old_tasks(self):
        """Remove old task history to prevent memory overflow"""
        if len(self.task_history) > self.max_task_history:
            # Sort by timestamp and remove oldest
            sorted_tasks = sorted(
                self.task_history.items(),
                key=lambda x: x[1]['timestamp']
            )
            tasks_to_remove = sorted_tasks[:len(self.task_history) - self.max_task_history]
            for task_id, _ in tasks_to_remove:
                del self.task_history[task_id]
            
            self.get_logger().info(
                f"Cleaned up {len(tasks_to_remove)} old tasks"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
