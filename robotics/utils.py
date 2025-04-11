import numpy as np
def IK(x ,y, z, Phi):
    """
    Calculates the inverse kinematics for a 4-DOF robotic arm in the elbow-down configuration,
    where (x, y, z) specifies the wrist position and Phi specifies the end-effector orientation.
    """
    a2 = 93  # Length of link 2
    a3 = 93  # Length of link 3
    d1 = 50  # Offset along Z axis
    a4 = 50  # Length of link 4
    # Step 1: Calculate Theta1 (Base joint angle)
    theta1 = np.arctan2(y, x)
    # Step 2: Calculate the distance from the base to the wrist in the XY plane and along Z
    r = np.sqrt(x**2 + y**2)  # Distance in the XY plane
    s = z - d1                # Distance along Z-axis from the base to the wrist
    # Step 3: Solve for Theta3 using the law of cosines
    cos_theta3 = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    cos_theta3 = np.clip(cos_theta3, -1, 1)  # Clamp to valid range [-1, 1]
    theta3 = np.arctan2(-np.sqrt(1 - cos_theta3**2), cos_theta3)  # Elbow-down solution
    # Step 4: Solve for Theta2
    gamma = np.arctan2(s, r)  # Angle to the wrist center
    beta = np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))  # Offset due to link geometry
    theta2 = gamma - beta # Final Theta2 calculation
    # Step 5: Solve for Theta4 (End-effector orientation adjustment)
    theta4 = Phi - (theta2 + theta3) +np.pi/2
    return [theta1, theta2, theta3, theta4]

def expand_waypoints(waypoints, z_target=-50, z_above=0):
    """
    Generate a sequence of waypoints with in-between transitions for a given list of waypoints.

    Args:
        waypoints (list of [x, y, z, phi]): List of waypoints with coordinates and orientation.
        z_target (float): The z-coordinate for the red object positions.
        z_above (float): The z-coordinate above each target (transition height).

    Returns:
        list of [x, y, z, phi]: List of waypoints including transitions, avoiding duplicates.
    """
    full_waypoints = []

    for i, (x, y, z, phi) in enumerate(waypoints):
        # Add the waypoint above the target
        if not full_waypoints or full_waypoints[-1] != [x, y, z_above, phi]:
            full_waypoints.append([x, y, z_above, phi])

        # Add the target waypoint
        full_waypoints.append([x, y, z_target, phi])

        # Add the retreat waypoint above the target
        if i < len(waypoints) - 1:
            next_x, next_y, _, next_phi = waypoints[i + 1]
            if [x, y, z_above, phi] != [next_x, next_y, z_above, next_phi]:
                full_waypoints.append([x, y, z_above, phi])

        # Transition to the next waypoint's above position
        if i < len(waypoints) - 1:
            next_x, next_y, _, next_phi = waypoints[i + 1]
            full_waypoints.append([next_x, next_y, z_above, next_phi])

    # Add the retreat waypoint above the final target
    if waypoints:
        final_x, final_y, _, final_phi = waypoints[-1]
        full_waypoints.append([final_x, final_y, z_above, final_phi])

    return full_waypoints