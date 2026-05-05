#!/usr/bin/env python3
"""
Joy Visualizer Example - 手柄输入可视化示例
实时显示手柄按键和摇杆状态
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyVisualizerExample(Node):
    """手柄可视化节点"""

    def __init__(self):
        super().__init__('joy_visualizer_example')
        
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('Joy Visualizer started - listening on /joy')
        
        # 按钮名称映射
        self.button_names = {
            0: 'A',
            1: 'B',
            2: 'X',
            3: 'Y',
            4: 'LB',
            5: 'RB',
            6: 'BACK',
            7: 'START',
            8: 'LEFT_THUMB',
            9: 'RIGHT_THUMB',
        }
        
        # 轴名称映射
        self.axes_names = {
            0: 'LEFT_X',
            1: 'LEFT_Y',
            2: 'LT',
            3: 'RIGHT_X',
            4: 'RIGHT_Y',
            5: 'RT',
            6: 'DPAD_X',
            7: 'DPAD_Y',
        }
    
    def joy_callback(self, msg: Joy):
        """手柄输入回调"""
        # 显示按钮状态
        pressed_buttons = []
        for i, button in enumerate(msg.buttons):
            if button == 1:
                button_name = self.button_names.get(i, f'Button_{i}')
                pressed_buttons.append(button_name)
        
        # 显示摇杆和触发器状态
        axes_info = []
        for i, axis in enumerate(msg.axes):
            axis_name = self.axes_names.get(i, f'Axis_{i}')
            if abs(axis) > 0.1:  # 只显示超过死区的轴
                axes_info.append(f'{axis_name}={axis:.2f}')
        
        # 构建输出信息
        output = []
        if pressed_buttons:
            output.append(f'Buttons: {", ".join(pressed_buttons)}')
        if axes_info:
            output.append(f'Axes: {", ".join(axes_info)}')
        
        if output:
            self.get_logger().info(' | '.join(output))


def main(args=None):
    rclpy.init(args=args)
    node = JoyVisualizerExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
