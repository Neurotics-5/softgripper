import numpy as np

class GripperController:
    def __init__(self, dynamixel_client, motor_ids, open_positions, closed_positions, config):
        self.client = dynamixel_client
        self.motor_ids = motor_ids
        self.open_positions = np.array(open_positions)
        self.closed_positions = np.array(closed_positions)

        self.min_position = {str(mid): config['min_position_rad'][mid] for mid in motor_ids}
        self.max_position = {str(mid): config['max_position_rad'][mid] for mid in motor_ids}

        self.current_control_mode = 'position'
        self.goal_positions = None  # Do not predefine a goal that triggers movement

        self.pulley_radius = {str(mid): config['pulley_radius'][mid] for mid in motor_ids}  # mm
        self.joint_radius = {str(mid): config['joint_radius'][mid] for mid in motor_ids}    # meters

        self.link_length = config.get('link_length', 0.1)  # assume same for all if single value

        self.pid_kp = 0.01  # Proportional gain (tune experimentally)
        self.pid_ki = 0.0
        self.pid_kd = 0.0

        self.pid_integral = 0.0
        self.pid_last_error = 0.0

    def connect(self):
        self.client.connect()
        self.client.set_torque_enabled(self.motor_ids, False)  # Always start with motors disabled

    def disconnect(self):
        self.client.disconnect()

    def open(self, speed=50):
        self.client.write_profile_velocity(self.motor_ids, np.full(len(self.motor_ids), speed))
        self.goal_positions = np.copy(self.open_positions)
        self.client.write_desired_pos(self.motor_ids, self.goal_positions)

    def close_parallel(self, speed=50):
        self.client.write_profile_velocity(self.motor_ids, np.full(len(self.motor_ids), speed))
        self.goal_positions = np.copy(self.closed_positions)
        self.client.write_desired_pos(self.motor_ids, self.goal_positions)


    def set_torque_enabled(self, enabled: bool):
        self.client.set_torque_enabled(self.motor_ids, enabled)

    def set_control_mode(self, mode):
        if mode == 'position':
            self.client.set_operating_mode(self.motor_ids, 3)
        elif mode == 'force':
            self.client.set_operating_mode(self.motor_ids, 5)
        else:
            raise ValueError(f"Unknown control mode: {mode}")
        self.current_control_mode = mode

    # def open(self):
    #     self.goal_positions = np.copy(self.open_positions)
    #     self.client.write_desired_pos(self.motor_ids, self.goal_positions)

    # def close_parallel(self):
    #     self.goal_positions = np.copy(self.closed_positions)
    #     self.client.write_desired_pos(self.motor_ids, self.goal_positions)

    def read_current_positions(self):
        positions, _, _ = self.client.read_pos_vel_cur()
        return positions

    def update(self):
        positions, velocities, currents = self.client.read_pos_vel_cur()
        temperatures = self.client.read_temperature()
        return {
            'positions': positions,
            'velocities': velocities,
            'currents': currents,
            'temperatures': temperatures
        }

    def get_joint_positions(self):
        motor_positions = self.read_current_positions()
        joint_positions = []

        for i, motor_id in enumerate(self.motor_ids):
            motor_angle = motor_positions[i]
            pulley_radius = self.pulley_radius[str(motor_id)] / 1000.0
            joint_radius = self.joint_radius[str(motor_id)]
            joint_angle = (pulley_radius / joint_radius) * motor_angle
            joint_positions.append(joint_angle)

        return joint_positions

    def estimate_fingertip_distance(self):
        joint_positions = self.get_joint_positions()
        angle_avg = np.mean(joint_positions[:2])
        distance = 2 * self.link_length * np.cos(angle_avg)
        return distance

    def is_goal_reached(self, tolerance=0.05):
        if self.goal_positions is None:
            return True

        current_pos = self.read_current_positions()
        return np.allclose(current_pos, self.goal_positions, atol=tolerance)


    def move_to_positions(self, target_dict, speed_dict=None):
        if self.goal_positions is None:
            self.goal_positions = self.read_current_positions()

        pos_array = np.copy(self.goal_positions)
        speed_array = np.full(len(self.motor_ids), 50)  # default speed

        for i, mid in enumerate(self.motor_ids):
            if mid in target_dict:
                pos_array[i] = target_dict[mid]
            if speed_dict and mid in speed_dict:
                speed_array[i] = speed_dict[mid]

        self.client.write_profile_velocity(self.motor_ids, speed_array)
        self.goal_positions = pos_array
        self.client.write_desired_pos(self.motor_ids, pos_array)

    def read_current_force(self):
        _, _, currents = self.client.read_pos_vel_cur()
        estimated_force = np.mean(currents) * 0.02
        return estimated_force

    def adjust_force_control(self, target_force, current_force, dt=0.02):
        if self.goal_positions is None:
            return  # nothing to adjust

        error = target_force - current_force
        p_term = self.pid_kp * error
        self.pid_integral += error * dt
        i_term = self.pid_ki * self.pid_integral
        derivative = (error - self.pid_last_error) / dt
        d_term = self.pid_kd * derivative
        pid_output = p_term + i_term + d_term
        self.pid_last_error = error

        self.goal_positions -= pid_output
        self.client.write_desired_pos(self.motor_ids, self.goal_positions)

    def adjust_grip_if_needed(self, current_force, hold_force=5.0, dt=0.02):
        if current_force < hold_force - 0.5:
            self.adjust_force_control(hold_force, current_force, dt=dt)


