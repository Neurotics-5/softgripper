import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
# from soft_gripper.action import ParallelGrip
import numpy as np
import asyncio

class GripperROS2Node(Node):
    def __init__(self, gripper_controller):
        super().__init__('gripper_node')
        self.gripper = gripper_controller

        # Publishers
        self.status_pub = self.create_publisher(String, 'gripper/status', 100)
        self.goal_reached_pub = self.create_publisher(String, 'gripper/goal_reached', 10)

        # Simple command subscriber (e.g. open, move_to_position)
        self.command_sub = self.create_subscription(
            String,
            'gripper/command',
            self.command_callback,
            10
        )

        # Timers
        self.status_timer = self.create_timer(0.01, self.status_callback)  # 100Hz status

        # # Action Server
        # self._parallel_grip_action_server = ActionServer(
        #     self,
        #     ParallelGrip,
        #     'parallel_grip',
        #     self.execute_parallel_grip_callback,
        #     callback_group=ReentrantCallbackGroup()
        # )

        self.get_logger().info('Gripper ROS2 Node ready. Action server available on /parallel_grip')

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
                state_str = command.split(':')[1].strip()
                state = int(state_str)          # 1 = open, 0 = close
                if state not in (0, 1):
                    raise ValueError('state must be 0 (close) or 1 (open)')

                self.get_logger().info(f'Executing pinch grip: {"open" if state == 1 else "close"}')
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
                    speed_dict = {mid: 50 for mid in self.gripper.motor_ids}  # default speed

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
        status = self.gripper.update()
        msg = String()
        msg.data = f"Pos: {status['positions']}, Currents: {status['currents']}"
        self.status_pub.publish(msg)

        # Publish goal reached status
        goal_msg = String()
        goal_msg.data = "reached" if self.gripper.is_goal_reached() else "moving"
        self.goal_reached_pub.publish(goal_msg)

    # async def execute_parallel_grip_callback(self, goal_handle):
    #     width_mm = goal_handle.request.width_mm
    #     self.get_logger().info(f'Received parallel grip action goal: {width_mm:.2f} mm')

    #     try:
    #         self.gripper.parallel_grip(width_mm)

    #         while not self.gripper.is_goal_reached():
    #             status = self.gripper.update()
    #             feedback_msg = ParallelGrip.Feedback()
    #             feedback_msg.current_width = status['width']
    #             goal_handle.publish_feedback(feedback_msg)
    #             await asyncio.sleep(0.05)  # 20 Hz loop

    #         result = ParallelGrip.Result()
    #         result.success = True
    #         result.message = f'Grip completed for width {width_mm:.2f} mm'
    #         self.get_logger().info(result.message)
    #         return result

    #     except Exception as e:
    #         result = ParallelGrip.Result()
    #         result.success = False
    #         result.message = f'Error: {str(e)}'
    #         self.get_logger().error(result.message)
    #         return result
