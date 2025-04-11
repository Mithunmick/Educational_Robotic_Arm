import numpy as np
import cv2
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from utils import *

class Robot:
    def __init__(self, port_name, baud_rate=1000000, protocol_version=1.0):
        """
        Initialize the robot with port and communication settings.
        """
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.protocol_version = protocol_version

        # Initialize communication objects
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        # Open the port
        if self.port_handler.openPort():
            print(f"Port {self.port_name} opened successfully.")
        else:
            raise IOError(f"Failed to open port {self.port_name}.")

        # Set the baud rate
        if self.port_handler.setBaudRate(self.baud_rate):
            print(f"Baud rate set to {self.baud_rate}.")
        else:
            raise IOError(f"Failed to set baud rate to {self.baud_rate}.")

        # Initialize instance variables
        self.current_torque_settings = {1: None, 2: None, 3: None, 4: None}
        self.goal_positions = []

        # Read motor settings
        self._initialize_motor_settings()

    def _initialize_motor_settings(self):
        """
        Read the current torque limits from all motors and update instance variable.
        """
        addr_torque_limit = 34  # Address for torque limit (Protocol 1.0)
        motor_ids = [1, 2, 3, 4]

        for motor_id in motor_ids:
            # Read current torque limit
            torque_limit, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, addr_torque_limit
            )
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                self.current_torque_settings[motor_id] = torque_limit
                print(f"Motor {motor_id} initial torque limit: {torque_limit}")
            else:
                print(f"Failed to read torque for motor {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")

    def _angles_to_dynamixel(self, thetas):
        """
        Maps angles in the real domain to Dynamixel units.
        """
        theta1_robot_angle = (thetas[0] + 150) % 360
        theta1_dynamixel = round((theta1_robot_angle / 300) * 1023)

        theta2_robot_angle = (thetas[1] + 60) % 360
        theta2_dynamixel = round((theta2_robot_angle / 300) * 1023)

        theta3_robot_angle = (thetas[2] + 150) % 360
        theta3_dynamixel = round((theta3_robot_angle / 300) * 1023)

        theta4_robot_angle = (thetas[3] + 90) * (240 - 150) / 90 + 150
        theta4_dynamixel = 512 + ((theta4_robot_angle - 150) / (240 - 150)) * (820 - 512)
        theta4_dynamixel = round(max(500, min(1023, theta4_dynamixel)))  # Clamp and round

        return np.array([theta1_dynamixel, theta2_dynamixel, theta3_dynamixel, theta4_dynamixel])

    def add_goal_position(self, angles):
        """
        Add a new goal position as a list of joint angles [theta1, theta2, theta3, theta4].
        """
        if len(angles) != 4:
            raise ValueError("Goal position must be a list of 4 joint angles.")
        self.goal_positions.append(angles)
        print(f"Added goal position: {angles}")

    def get_goal_positions(self):
        """
        Retrieve all stored goal positions.
        """
        return self.goal_positions

    def clear_goal_positions(self):
        """
        Clear all stored goal positions.
        """
        self.goal_positions.clear()
        print("All goal positions cleared.")

    def _set_torque_per_joint(self, motor_id, torque_limit):
        """
        Set the torque for a specific motor.
        """
        addr_torque_limit = 34  # Address for torque limit (Protocol 1.0)

        if self.current_torque_settings[motor_id] != torque_limit:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, addr_torque_limit, torque_limit
            )
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to set torque for motor {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Error on motor {motor_id} when setting torque: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                print(f"Motor {motor_id} torque updated to {torque_limit}.")
                self.current_torque_settings[motor_id] = torque_limit

    def drive_with_units(self, positions, torque_limits=None):
        """
        Drive the robot by setting joint positions in Dynamixel units, with configurable torque.
        Ensures all motors start and stop moving simultaneously.
        """
        if len(positions) != 4:
            raise ValueError("Exactly 4 joint positions are required.")

        if torque_limits is None:
            torque_limits = [180] * 4

        if len(torque_limits) != 4:
            raise ValueError("Torque limits must have exactly 4 values.")

        motor_ids = [1, 2, 3, 4]
        addr_goal_position = 30  # Address for goal position in Dynamixel protocol

        # Step 1: Set the torque limits for all motors
        for motor_id, torque_limit in zip(motor_ids, torque_limits):
            self._set_torque_per_joint(motor_id, torque_limit)

        # Step 2: Prepare synchronized write for goal positions
        sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, addr_goal_position, 2)

        for motor_id, position in zip(motor_ids, positions):
            param_goal_position = [DXL_LOBYTE(position), DXL_HIBYTE(position)]
            addparam_result = sync_write.addParam(motor_id, param_goal_position)
            if not addparam_result:
                print(f"Failed to add motor {motor_id} to sync write.")
                return

        # Step 3: Execute the synchronized write
        sync_write_result = sync_write.txPacket()
        if sync_write_result != COMM_SUCCESS:
            print(f"Sync write failed: {self.packet_handler.getTxRxResult(sync_write_result)}")
        else:
            print("All motors started moving simultaneously.")

        # Clear the parameters after execution
        sync_write.clearParam()


    def set_robot_angles(self, angles, torque_limits=None):
        """
        Set the robot's joint positions using angles in the real domain.
        """
        positions = self._angles_to_dynamixel(np.array(angles))
        self.drive_with_units(positions=positions, torque_limits=torque_limits)

    def run(self):
        while self.goal_positions:
            angles = self.goal_positions.pop(0)
            positions = self._angles_to_dynamixel(np.array(angles))
            
            # Drive to the goal position
            self.drive_with_units(positions=positions)
            
            # Wait until the robot reaches the goal position
            if not self._has_reached_goal(positions):
                print("Warning: Failed to reach goal position.")
    
    def _has_reached_goal(self, positions, tolerance=10, timeout=1, check_interval=0.03):
        """
        Check if all motors have reached their goal positions within a timeout period.
        """
        addr_present_position = 36  # Address for current position (Protocol 1.0)
        motor_ids = [1, 2, 3, 4]
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            all_reached = True

            for motor_id, goal_position in zip(motor_ids, positions):
                present_position, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, addr_present_position
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(f"Error reading position for motor {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                    continue
        
                if abs(goal_position - present_position) > tolerance:
                    print("haha")
                    all_reached = False
                    break

            if all_reached:
                return True

            time.sleep(check_interval)  # Short sleep to avoid excessive CPU usage

        print("Timeout waiting for motors to reach goal.")
        return False


    
    def create_path(self, waypoints, z_target=-10, z_above=0, N=10, interpolation='linear'):
        """
        Generate a trajectory path for the robot to follow, handling transitions and avoiding redundant points.

        Args:
            waypoints (list of [x, y, z, phi]): List of waypoints with coordinates and orientation.
            z_target (float): Z-coordinate for the target.
            z_above (float): Z-coordinate above the target.
            N (int): Number of intermediate points between each pair of waypoints.
            interpolation (str): Type of interpolation (default: 'linear').

        Returns:
            None
        """
        if interpolation != 'linear':
            raise ValueError(f"Unsupported interpolation type: {interpolation}")

        # Process each waypoint and add "above", "target", and "retreat above" positions
        full_waypoints = []
        for waypoint in waypoints:
            x, y, z, phi = waypoint
            full_waypoints.append([x, y, z_above, phi])  # Above
            full_waypoints.append([x, y, z_target, phi])  # Target
            full_waypoints.append([x, y, z_above, phi])  # Retreat above

        # Interpolate between the waypoints and add to goal positions
        raw_goal_positions = []
        for i in range(len(full_waypoints) - 1):
            # Calculate joint angles using IK for each waypoint
            start_angles = np.degrees(IK(*full_waypoints[i])).tolist()
            end_angles = np.degrees(IK(*full_waypoints[i + 1])).tolist()

            # Normalize 4th joint angle to [-180, 180]
            if start_angles[3] > 180:
                start_angles[3] -= 360
            if end_angles[3] > 180:
                end_angles[3] -= 360

            # Interpolate between angles
            interpolated = [
                np.linspace(start_angles[j], end_angles[j], N).tolist()
                for j in range(len(start_angles))
            ]

            # Append interpolated points to raw goal positions
            raw_goal_positions.extend(list(map(list, zip(*interpolated))))

        # Deduplicate consecutive positions
        self.goal_positions = [raw_goal_positions[0]]
        for i in range(1, len(raw_goal_positions)):
            if raw_goal_positions[i] != raw_goal_positions[i - 1]:
                self.goal_positions.append(raw_goal_positions[i])

        # Ensure the last position is included
        final_angles = np.degrees(IK(*full_waypoints[-1])).tolist()
        if final_angles[3] > 180:
            final_angles[3] -= 360
        if self.goal_positions[-1] != final_angles:
            self.goal_positions.append(final_angles)

        print("\nFinal Goal Positions (Joint Angles):")
        for pos in self.goal_positions:
            print(pos)

    def close(self):
        """
        Close the port when done.
        """
        self.port_handler.closePort()
        print("Port closed.")

class RobotAssignment(Robot):
    def __init__(self, port_name, baud_rate=1000000, protocol_version=1.0):
        """
        Initialize the RobotAssignment class, inheriting from the Robot class.
        """
        super().__init__(port_name, baud_rate, protocol_version)
        self.initial_position = [0, 90, -90, -90]  # Replace with the robot's actual initial position
        self.waypoints = []
        self.red_objects = []  # List to store red object positions
        self.image_frame = None
        self.state = "IDLE"

        self.cap = cv2.VideoCapture(0)  # Open the webcam

        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")
        print("Webcam initialized successfully.")

    def create_waypoints(self):
        """
        Process the detected red objects and create 3D waypoints with orientation.
        Each red object is represented as [x, y, z, phi].

        Returns:
            list of [x, y, z, phi]: A list of 3D waypoints with orientation.
        """
        print("Creating waypoints from detected red objects...")
        # Extract [x, y, z, phi] from red objects
        waypoints = [[obj[0], obj[1], 55, np.pi] for obj in self.red_objects]
        print(f"Generated waypoints: {waypoints}")
        return waypoints

    def return_to_initial(self):
        """
        Return the robot to its initial position.
        """
        print("Returning to initial position...")
        self.set_robot_angles(self.initial_position)
        print("Returned to initial position.")
    
    def detect_smarties(self):
        # Convert the image from BGR to HSV color space
        image_hsv = cv2.cvtColor(self.image_frame, cv2.COLOR_BGR2HSV)

        # Define HSV thresholds for red color
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create a mask for red regions by combining both ranges
        mask1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Find contours of the red regions
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate areas of all bounding boxes and find the largest area
        bounding_box_areas = [cv2.boundingRect(contour)[2] * cv2.boundingRect(contour)[3] for contour in contours]
        max_area = max(bounding_box_areas) if bounding_box_areas else 0

        # Define a threshold for filtering: 80% of the largest bounding box area
        threshold_area = 0.80 * max_area

        # Prepare a list to store the bounding boxes
        filtered_bounding_boxes = []

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            box_area = w * h  # Calculate the area of the bounding box

            if box_area >= threshold_area:  # Retain bounding boxes above the threshold
                filtered_bounding_boxes.append((x, y, w, h))

        self.red_objects = filtered_bounding_boxes
    
    def take_photo(self, test):
        """
        Capture a photo from the webcam.
        """
        print("Press 's' to capture the image or 'q' to quit.")
        if not test:
            while True:
                # Read a frame from the webcam
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame.")
                    break

                # Wait for key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):  # Save the image on 's' key press
                    self.image_frame = frame
                    print("Image captured.")
                    break
                elif key == ord('q'):  # Quit without saving on 'q' key press
                    print("Quit without capturing.")
                    break

    def execute(self,test=False):
        """
        Execute the robot's state machine.
        """
        while True:  # The state machine loops indefinitely
            if self.state == "IDLE":
                self.set_robot_angles(self.initial_position)
                print("State: IDLE")
                user_input = input("Press Enter to start or 'q' to quit: ").strip().lower()
                if user_input == 'q':
                    # Release the webcam and close the window
                    self.cap.release()
                    cv2.destroyAllWindows()
                    print("Exiting program...")
                    self.close()
                    break
                elif user_input == '':
                    self.state = "PHOTO"

            elif self.state == "PHOTO":
                print("State: PHOTO")
                self.take_photo(test)
                self.state = "DETECT"

            elif self.state == "DETECT":
                print("State: DETECT")
                self.detect_smarties()
                self.state = "WAYPOINTS"

            elif self.state == "WAYPOINTS":
                print("State: WAYPOINTS")
                self.waypoints = self.create_waypoints()
                self.state = "TRAJECTORY"

            elif self.state == "TRAJECTORY":
                print("State: TRAJECTORY")
                self.create_path(self.waypoints, z_target=55, z_above=105)
                self.state = "DRIVE"

            elif self.state == "DRIVE":
                print("State: DRIVE")
                self.run()
                self.state = "RETURN"

            elif self.state == "RETURN":
                print("State: RETURN")
                self.return_to_initial()
                self.state = "IDLE"  # Go back to IDLE instead of DONE

def test_robot():
    # Create a robot instance
    robot = Robot("COM11")

    try:
        wp1 = [150, 0, 50, np.pi]
        wp2 = [150, 50, 50, np.pi]
        robot.create_path([wp1,wp2],55,105)
        print(robot.get_goal_positions())
        robot.run()

    finally:
        # Ensure the port is closed after testing
        robot.close()

def test_cycle():
    robot = RobotAssignment('COM11')
    try:
        robot.execute(test=True)

    finally:
        # Ensure the port is closed after testing
        robot.close()

# Run the test
if __name__ == "__main__":
    test_cycle()
