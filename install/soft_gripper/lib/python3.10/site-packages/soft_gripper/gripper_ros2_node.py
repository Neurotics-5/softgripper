import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np

# Map motor IDs to URDF joint names
JOINT_NAME_MAP = {
    1: "motor_simple-v2-1_Revolute-23",
    2: "motor_simple-v2-3_Revolute-25",
    3: "motor_simple-v2_Revolute-22",
    4: "motor_simple-v2-2_Revolute-24",
}

class GripperROS2Node(Node):
    def __init__(self, gripper_controller):
        super().__init__('gripper_node')
        self.gripper = gripper_controller

        # Publishers
        self.status_pub = self.create_publisher(String, 'gripper/status', 100)
        self.goal_reached_pub = self.create_publisher(String, 'gripper/goal_reached', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create joint name list in correct motor ID order
        self._joint_names = [JOINT_NAME_MAP[mid] for mid in self.gripper.motor_ids]

        # Command subscriber
        self.command_sub = self.create_subscription(
            String,
            'gripper/command',
            self.command_callback,
            10
        )

        # Timer for status + joint states
        self.status_timer = self.create_timer(0.01, self.status_callback)  # 100Hz

        self.get_logger().info('Gripper ROS2 Node ready.')

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {command}')

        if command == 'open':
            self.gripper.open()

        elif command.startswith('parallel_grip:'):
            try:
                width_mm = float(command.split(':')[1].strip())
                self.get_logger().info(f'Executing parallel grip with width {width_mm} mm')
                self.gripper.parallel_grip(width_mm)
            except Exception as e:
                self.get_logger().error(f'Invalid parallel grip command: {e}')

        elif command.startswith('pinch_grip:'):
            try:
                state = int(command.split(':')[1].strip())
                if state not in (0, 1):
                    raise ValueError('state must be 0 or 1')
                self.get_logger().info(f'Executing pinch grip: {"open" if state else "close"}')
                self.gripper.pinch_grip(state)
            except Exception as e:
                self.get_logger().error(f'Invalid pinch grip command: {e}')

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
                    speed_dict = {mid: 50 for mid in self.gripper.motor_ids}

                self.gripper.move_to_positions(pos_dict, speed_dict=speed_dict)
                self.get_logger().info(f'Moved to positions: {pos_dict} with speeds: {speed_dict}')
            except Exception as e:
                self.get_logger().error(f'Invalid position format: {e}')

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
            JOINT_POSITION_OFFSETS = {
                "motor_simple-v2_Revolute-22": -np.pi,
                "motor_simple-v2-1_Revolute-23": -np.pi,
                "motor_simple-v2-2_Revolute-24": -np.pi,
                "motor_simple-v2-3_Revolute-25": -np.pi,
            }

            status = self.gripper.update()

            msg = String()
            msg.data = f"Pos: {status['positions']}, Currents: {status['currents']}"
            self.status_pub.publish(msg)

            goal_msg = String()
            goal_msg.data = "reached" if self.gripper.is_goal_reached() else "moving"
            self.goal_reached_pub.publish(goal_msg)

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self._joint_names
            js.position = [
                status['positions'][i] + JOINT_POSITION_OFFSETS.get(name, 0.0)
                for i, name in enumerate(self._joint_names)
            ]
            self.joint_pub.publish(js)
