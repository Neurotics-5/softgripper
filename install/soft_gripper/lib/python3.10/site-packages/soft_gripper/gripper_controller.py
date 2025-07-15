import numpy as np
import time

class GripperController:
    def __init__(self, dynamixel_client, motor_ids, config):
        self.client = dynamixel_client
        self.motor_ids = motor_ids
        self.open_positions = [config['open_positions'][mid] for mid in motor_ids]

        self.min_position = {str(mid): config['min_position_rad'][mid] for mid in motor_ids}
        self.max_position = {str(mid): config['max_position_rad'][mid] for mid in motor_ids}

        self.current_control_mode = 'position'
        self.goal_positions = None  # Do not predefine a goal that triggers movement

        self.l1 = config.get('linkBase', 0.1)
        self.l2 = config.get('linkTact', 0.1) 
        self.l3 = config.get('linkTip', 0.1)  

        self.grip = ''

        self.pid_kp = 0.01  # Proportional gain (tune experimentally)
        self.pid_ki = 0.0
        self.pid_kd = 0.0

        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        self.last_positions = np.zeros(len(self.motor_ids))
        self.last_load = np.zeros(len(self.motor_ids))

        self.contact_threshold = 20.0 # Example: threshold at which contact is considered made
        self.max_load_threshold = 20.0  # Example: stall-level load
        self.tip_tolerance_rad = np.deg2rad(20)  # Small follow-up motion for tip fingers



    def connect(self):
        self.client.connect()
        self.client.set_torque_enabled(self.motor_ids, False)  # Always start with motors disabled

    def disconnect(self):
        self.client.disconnect()

    def set_torque_enabled(self, enabled: bool):
        self.client.set_torque_enabled(self.motor_ids, enabled)

    def open(self,):
        self.goal_positions = np.copy(self.open_positions)
        self.client.write_desired_pos(self.motor_ids, self.goal_positions)

    def parallel_grip(self, width):
        """
        Perform a parallel grip to achieve the specified width (in mm) between fingertips.
        Assumes:
        - Motors 1,2 = Finger A (base, tip)
        - Motors 3,4 = Finger B (base, tip), mirrored
        """
        self.grip = 'Parallel'

        y_dis = width / 2.0  # One side of the total width

        if y_dis > self.l1:
            print(f"[parallel_grip] Width too large: half-width {y_dis} mm exceeds l1 = {self.l1}")
            return

        try:
            theta = np.arcsin(y_dis / self.l1)
        except ValueError:
            print("[parallel_grip] Invalid width: arcsin input out of range")
            return

        # Calculate joint targets (adjusted to actual joint ranges in config)
        target_positions = {
            1: np.clip(theta + np.pi, self.min_position["1"], self.max_position["1"]),
            2: np.clip(-theta + np.pi, self.min_position["2"], self.max_position["2"]),
            3: np.clip(-theta + np.pi, self.min_position["3"], self.max_position["3"]),
            4: np.clip(theta + np.pi, self.min_position["4"], self.max_position["4"]),
        }

        self.move_to_positions(target_positions)

    def parallel_force(self, width):
        self.grip = 'ParallelForce'

        y_dis = width / 2.0
        if y_dis > self.l1:
            print(f"[parallel_force] Width too large: half-width {y_dis} mm exceeds l1 = {self.l1}")
            return

        try:
            theta = np.arcsin(y_dis / self.l1)
        except ValueError:
            print("[parallel_force] Invalid width: arcsin input out of range")
            return

        target_positions = {
            1: np.clip(theta + np.pi, self.min_position["1"], self.max_position["1"]),
            2: np.clip(-theta + np.pi, self.min_position["2"], self.max_position["2"]),
            3: np.clip(-theta + np.pi, self.min_position["3"], self.max_position["3"]),
            4: np.clip(theta + np.pi, self.min_position["4"], self.max_position["4"]),
        }

        self.move_to_positions(target_positions)

        # Wait for contact on base motors (1 and 3)
        print("[parallel_force] Monitoring load for contact on base fingers...")
        start_time = time.time()
        timeout_sec = 3.0
        while True:
            _, _, load = self.client.read_pos_vel_load()
            if abs(load[0]) > self.contact_threshold or abs(load[2]) > self.contact_threshold:
                print(f"[parallel_force] Contact detected. Load: {load}")
                break
            if time.time() - start_time > timeout_sec:
                print("[parallel_force] Timeout: No contact detected within 3 seconds.")
                return
            time.sleep(0.01)

        # Freeze base motors, let tip motors follow slightly
        positions, _, _ = self.client.read_pos_vel_load()
        base_hold = {
            1: positions[0],  # base A
            3: positions[2],  # base B
        }
        tip_followup = {
            2: np.clip(positions[1] - self.tip_tolerance_rad, self.min_position["2"], self.max_position["2"]),
            4: np.clip(positions[3] + self.tip_tolerance_rad, self.min_position["4"], self.max_position["4"]),
        }

        final_target = {**base_hold, **tip_followup}
        print(f"[parallel_force] Tip follow-up targets: M2 -> {tip_followup[2]:.3f}, M4 -> {tip_followup[4]:.3f}")
        self.move_to_positions(final_target)

        # Wait for any motor to reach high force
        print("[parallel_force] Tip fingers continuing to apply force...")
        start_time = time.time()
        while True:
            _, _, load = self.client.read_pos_vel_load()
            if np.any(np.abs(load) > self.max_load_threshold):
                print(f"[parallel_force] Max load reached, stopping. Load: {load}")
                break
            if time.time() - start_time > timeout_sec:
                print("[parallel_force] Timeout: no force threshold reached within 3 seconds.")
                break
            time.sleep(0.01)

        print("[parallel_force] Grip complete.")


    def pinch_grip(self, state: int):
        """
        Instant pinch grip position:
        - state = 1 → open (tips: 210°, 150°)
        - state = 0 → closed (tips: 180°)
        Base joints (1 and 4) always at 180°.
        """
        self.grip = 'Pinch'

        if state == 1:
            # Open configuration
            target_positions = {
                1: np.deg2rad(200),        # base A
                2: np.deg2rad(200),        # tip A
                3: np.deg2rad(160),        # base B
                4: np.deg2rad(160),        # tip B
            }
        elif state == 0:
            # Closed configuration
            target_positions = {
                1: np.deg2rad(200),        # base A
                2: np.deg2rad(140),        # tip A
                3: np.deg2rad(160),        # base B
                4: np.deg2rad(220),        # tip B
            }
        else:
            print(f"[pinch_grip] Invalid state: {state}. Use 1=open or 0=close.")
            return

        # Apply joint limits
        for mid in target_positions:
            target_positions[mid] = np.clip(
                target_positions[mid],
                self.min_position[str(mid)],
                self.max_position[str(mid)]
            )

        self.move_to_positions(target_positions)
        self.current_positions = target_positions



    def form_grip(self):
        self.grip = 'Form'
        self.goal_positions = np.copy(self.closed_positions)
        self.client.write_desired_pos(self.motor_ids, self.goal_positions)

    def read_current_positions(self):
        positions, _, _ = self.client.read_pos_vel_load()
        return positions

    def update(self):
        positions, velocities, load = self.client.read_pos_vel_load()

        # Compute derived values (only if grip mode needs it)
        if self.grip in ('Pinch', 'Parallel'):
            x_pos = self.l1 * np.cos(positions[1]) + self.l2 * np.cos(positions[1] + positions[2])
            width = self.l1 * np.cos(positions[1])
        elif self.grip == 'Form':
            x_pos = 100.0
            width = 100.0
        else:
            x_pos = -1.0
            width = -1.0

        return {
            'positions': positions,
            'load': load,
            'grip': self.grip,
            'width': width,
            'x_pos': x_pos
        }


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
    
    def reboot_all_motors(self):
        self.client.reboot_motors(self.motor_ids)
