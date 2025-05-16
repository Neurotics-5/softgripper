import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class GripperROS2Node(Node):
    def __init__(self, gripper_controller):
        super().__init__('gripper_node')
        self.gripper = gripper_controller

        # Publishers
        self.status_pub = self.create_publisher(String, 'gripper/status', 100)
        self.force_status_pub = self.create_publisher(String, 'gripper/force_status', 10)
        self.goal_reached_pub = self.create_publisher(String, 'gripper/goal_reached', 10)

        # Simple command subscriber for testing
        self.command_sub = self.create_subscription(
            String,
            'gripper/command',
            self.command_callback,
            10
        )

        # Timers
        self.status_timer = self.create_timer(0.01, self.status_callback)  # 100Hz status
        self.force_monitor_timer = self.create_timer(0.02, self.force_monitor_callback)  # 50Hz force

        self.force_control_active = False
        self.force_target = 0.0

        self.get_logger().info('Gripper ROS2 Node ready for simple command testing.')

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {command}')

        if command == 'open':
            self.gripper.open()
            self.force_control_active = False

        elif command == 'close':
            self.gripper.close_parallel()
            self.force_control_active = False

        elif command.startswith('move_to_position:'):
            try:
                positions_str = command.split(':')[1]
                pos_dict = {}
                speed_dict = {}

                if '=' in positions_str:
                    for pair in positions_str.split(','):
                        motor_part = pair.strip()
                        if '@' in motor_part:
                            motor_val, speed = motor_part.split('@')
                        else:
                            motor_val, speed = motor_part, None

                        motor_id, position = motor_val.split('=')
                        mid = int(motor_id.strip())
                        pos_dict[mid] = float(position.strip())

                        if speed is not None:
                            speed_dict[mid] = int(speed.strip())

                else:
                    value = float(positions_str.strip())
                    pos_dict = {mid: value for mid in self.gripper.motor_ids}
                    speed_dict = {mid: 50 for mid in self.gripper.motor_ids}  # default speed

                self.gripper.move_to_positions(pos_dict, speed_dict=speed_dict)
                self.force_control_active = False
                self.get_logger().info(f'Moved to positions: {pos_dict} with speeds: {speed_dict}')
            except Exception as e:
                self.get_logger().error(f'Invalid position format: {e}')

        elif command.startswith('move_to_position_force:'):
            try:
                parts = command.split(':')
                pos_value = float(parts[1])
                force_value = float(parts[2])
                target_positions = [pos_value] * len(self.gripper.motor_ids)
                self.gripper.move_to_positions(target_positions, slow=True)
                self.force_target = force_value
                self.force_control_active = True
                self.get_logger().info(f'Move to {pos_value} with force control target {force_value}N')
            except (IndexError, ValueError):
                self.get_logger().error('Invalid format. Use move_to_position_force:<position>:<force>.')

        elif command.startswith('set_operating_mode:'):
            mode = command.split(':')[1].strip()
            if mode in ['position', 'force']:
                self.gripper.set_control_mode(mode)
                self.get_logger().info(f'Set torque control mode to {mode}')
            else:
                self.get_logger().error('Invalid operating mode. Use "position" or "force".')

        elif command.startswith('set_torque_enable:'):
            value = command.split(':')[1].strip()
            if value == 'on':
                self.gripper.set_torque_enabled(True)
                self.get_logger().info('Torque enabled.')
            elif value == 'off':
                self.gripper.set_torque_enabled(False)
                self.get_logger().info('Torque disabled.')
            else:
                self.get_logger().error('Invalid value for set_torque_enable. Use "on" or "off".')

        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def status_callback(self):
        status = self.gripper.update()
        msg = String()
        msg.data = f"Pos: {status['positions']}, Currents: {status['currents']}"
        self.status_pub.publish(msg)

        # Publish goal reached status
        goal_msg = String()
        if self.gripper.is_goal_reached():
            goal_msg.data = "reached"
        else:
            goal_msg.data = "moving"
        self.goal_reached_pub.publish(goal_msg)

    def force_monitor_callback(self):
        current_force = self.gripper.read_current_force()
        msg = String()
        msg.data = f'Current grip force: {current_force:.2f} N'
        self.force_status_pub.publish(msg)

        # Only run force control if explicitly enabled via move_to_position_force
        if self.force_control_active:
            self.gripper.adjust_force_control(self.force_target, current_force, dt=0.02)
