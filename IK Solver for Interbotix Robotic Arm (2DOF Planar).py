'''
import math
import matplotlib.pyplot as plt

def interbotix_inverse_kinematics(x, y, L_shoulder, L_elbow):
    """
    Calculate the inverse kinematics for the simplified Interbotix arm (2-DOF planar).

    Parameters:
    - x, y: Desired end-effector position
    - L_shoulder, L_elbow: Lengths of the shoulder and elbow links

    Returns:
    - theta_shoulder, theta_elbow: Joint angles in radians
    """

    # Calculate theta_elbow
    D = (x**2 + y**2 - L_shoulder**2 - L_elbow**2) / (2 * L_shoulder * L_elbow)
    theta_elbow = math.atan2(-math.sqrt(1 - D**2), D)

    # Calculate theta_shoulder
    K1 = L_shoulder + L_elbow * math.cos(theta_elbow)
    K2 = L_elbow * math.sin(theta_elbow)
    theta_shoulder = math.atan2(y, x) - math.atan2(K2, K1)

    return theta_shoulder, theta_elbow
'''
import math
import matplotlib.pyplot as plt

def interbotix_inverse_kinematics(x, y, L_shoulder, L_elbow):
    """
    Calculate the inverse kinematics for the simplified Interbotix arm (2-DOF planar).

    Parameters:
    - x, y: Desired end-effector position
    - L_shoulder, L_elbow: Lengths of the shoulder and elbow links

    Returns:
    - theta_shoulder, theta_elbow: Joint angles in radians
    """

    # Calculate theta_elbow
    D = (x**2 + y**2 - L_shoulder**2 - L_elbow**2) / (2 * L_shoulder * L_elbow)
    theta_elbow = math.atan2(-math.sqrt(1 - D**2), D)

    # Calculate theta_shoulder
    K1 = L_shoulder + L_elbow * math.cos(theta_elbow)
    K2 = L_elbow * math.sin(theta_elbow)
    theta_shoulder = math.atan2(y, x) - math.atan2(K2, K1)

    return theta_shoulder, theta_elbow

def plot_interbotix_arm(theta_shoulder, theta_elbow, L_shoulder, L_elbow):
    """
    Visualize the simplified Interbotix arm (2-DOF planar).

    Parameters:
    - theta_shoulder, theta_elbow: Joint angles in radians
    - L_shoulder, L_elbow: Lengths of the shoulder and elbow links
    """

    # Calculate end-effector position
    x_shoulder = L_shoulder * math.cos(theta_shoulder)
    y_shoulder = L_shoulder * math.sin(theta_shoulder)

    x_elbow = x_shoulder + L_elbow * math.cos(theta_shoulder + theta_elbow)
    y_elbow = y_shoulder + L_elbow * math.sin(theta_shoulder + theta_elbow)

    # Plot the robot arm
    plt.figure(figsize=(8, 8))
    plt.plot([0, x_shoulder, x_elbow], [0, y_shoulder, y_elbow], marker='o', linestyle='-', color='b', linewidth=2, markersize=8)
    plt.plot(0, 0, marker='o', color='r', markersize=8, label='Shoulder')
    plt.plot(x_shoulder, y_shoulder, marker='o', color='g', markersize=8, label='Elbow')
    plt.plot(x_elbow, y_elbow, marker='o', color='m', markersize=8, label='End Effector')

    # Set axis limits
    plt.xlim(-L_shoulder - L_elbow, L_shoulder + L_elbow)
    plt.ylim(-L_shoulder - L_elbow, L_shoulder + L_elbow)

    # Add labels and legend
    plt.title('Interbotix Arm (2-DOF Planar)')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.show()

# Example usage with specific dimensions
L_shoulder = 206.16  # Length of the shoulder link
L_elbow = 200 + 174.15    # Length of the elbow link
end_effector_x = 474.16# Home Pose, x= 374.16, y = 206.16
end_effector_y = 206.16

# Calculate inverse kinematics
theta_shoulder, theta_elbow = interbotix_inverse_kinematics(end_effector_x, end_effector_y, L_shoulder, L_elbow)

# Print the calculated joint angles
print(f"Theta Shoulder: {math.degrees(theta_shoulder):.2f} degrees")
print(f"Theta Elbow: {math.degrees(theta_elbow):.2f} degrees")

# Visualize the simplified Interbotix arm
plot_interbotix_arm(theta_shoulder, theta_elbow, L_shoulder, L_elbow)
