#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading  # <-- Add this import statement

# Global variables for storing the data
time_list = []
desired_heave_list = []
current_heave_list = []

def send_sinusoidal_command():
    pub = rospy.Publisher('stewart/platform_pose', Twist, queue_size=10)
    rospy.init_node('stewart_command_sinusoidal', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    start_time = time.time()

    # PD controller previous errors
    roll_error_prev = 0
    pitch_error_prev = 0
    heave_error_prev = 0
    yaw_error_prev=0
    x_error_prev=0
    y_error_prev=0
    # Control gains
    Kp = 0.4 # Proportional gain
    Kd = 0.2# Derivative gain

    # Sinusoidal parameters
    amplitude = 0.3  # Reduced amplitude for roll and pitch
    heave_amplitude = 0.6  # Reduced amplitude for heave
    frequency = 1  # Reduced frequency

    # Initialize last command values for filtering
    last_roll_command = 0
    last_pitch_command = 0
    last_heave_command = 0
    last_yaw_command=0
    last_x_command=0
    last_y_command=0
    # Filtering factor for low-pass filter
    alpha = 0.2

    # # Lists to store heave data
    # time_list = []
    # desired_heave_list = []
    # current_heave_list = []

    # # Set up the plot
    # plt.ion()
    # fig, ax = plt.subplots()
    # desired_heave_line, = ax.plot([], [], label="Desired Heave")
    # current_heave_line, = ax.plot([], [], label="Current Heave", linestyle='--')
    # ax.set_xlabel("Time (s)")
    # ax.set_ylabel("Heave (m)")
    # ax.legend()
    # ax.set_title("Desired vs Current Heave")


    while not rospy.is_shutdown():
        current_time = time.time() - start_time

        # Desired sinusoidal values for roll, pitch, and heave
        # desired_roll = amplitude * math.sin(frequency * current_time)
        # desired_pitch = amplitude * math.sin(frequency * current_time + math.pi / 2)
        # desired_heave = heave_amplitude * math.sin(frequency * current_time) + 0.4
        desired_roll = 0
        desired_pitch = 0
        desired_heave = 0.3+heave_amplitude * math.sin(frequency * current_time) + 0.4
        desired_x=0
        desired_y=0
        desired_yaw=0
        # Mock feedback (replace with actual feedback from sensors)
        feedback_roll = last_roll_command
        feedback_pitch = last_pitch_command
        feedback_heave = last_heave_command
        feedback_x= last_x_command
        feedback_y=last_y_command
        feedback_yaw=last_yaw_command
        # Calculate errors
        roll_error = desired_roll - feedback_roll
        pitch_error = desired_pitch - feedback_pitch
        heave_error = desired_heave - feedback_heave
        yaw_error=desired_yaw-feedback_yaw
        x_error   = desired_x-feedback_x
        y_error    = desired_y-feedback_y

        # PD control for roll, pitch, and heave
        roll_command = desired_roll + Kp * roll_error + Kd * (roll_error - roll_error_prev)
        pitch_command = desired_pitch + Kp * pitch_error + Kd * (pitch_error - pitch_error_prev)
        heave_command = desired_heave + Kp * heave_error + Kd * (heave_error - heave_error_prev)
        # PD control for yaw, x, and y
        yaw_command = desired_yaw + Kp * yaw_error + Kd * (yaw_error - yaw_error_prev)
        x_command   = desired_x + Kp * x_error + Kd * (x_error - x_error_prev)
        y_command   = desired_y + Kp * y_error + Kd * (y_error - y_error_prev)

        # Update previous errors
        roll_error_prev = roll_error
        pitch_error_prev = pitch_error
        heave_error_prev = heave_error
        yaw_error_prev = yaw_error
        x_error_prev = x_error
        y_error_prev = y_error

        # Apply low-pass filter to smooth the commands
        filtered_roll_command = alpha * roll_command + (1 - alpha) * last_roll_command
        filtered_pitch_command = alpha * pitch_command + (1 - alpha) * last_pitch_command
        filtered_heave_command = alpha * heave_command + (1 - alpha) * last_heave_command
        filtered_yaw_command = alpha * yaw_command + (1 - alpha) * last_yaw_command
        filtered_x_command   = alpha * x_command + (1 - alpha) * last_x_command
        filtered_y_command   = alpha * y_command + (1 - alpha) * last_y_command

        # Update last commands
        last_roll_command = filtered_roll_command
        last_pitch_command = filtered_pitch_command
        last_heave_command = filtered_heave_command
        last_yaw_command = filtered_yaw_command
        last_x_command = filtered_x_command
        last_y_command = filtered_y_command

         # Store data for plotting
        time_list.append(current_time)
        desired_heave_list.append(desired_heave)
        current_heave_list.append(filtered_heave_command)

        
        # Prepare the Twist message
        command = Twist()
        command.angular.x = filtered_roll_command
        command.angular.y = filtered_pitch_command
        command.linear.z = filtered_heave_command
        command.angular.z = filtered_yaw_command
        command.linear.x = filtered_x_command
        command.linear.y = filtered_y_command
        rospy.loginfo("Sending roll: %s, pitch: %s, heave: %s", filtered_roll_command, filtered_pitch_command, filtered_heave_command)
        pub.publish(command)
        rate.sleep()
def animate(i):
    # Update the plot data
    plt.cla()  # Clear the previous data
    plt.plot(time_list, desired_heave_list, label="Desired Heave")
    plt.plot(time_list, current_heave_list, label="Current Heave", linestyle='--')
    plt.xlabel("Time (s)")
    plt.ylabel("Heave (m)")
    plt.legend()
    plt.title("Desired vs Current Heave")
    plt.tight_layout()


# if __name__ == '__main__':
#     try:
#         send_sinusoidal_command()
#     except rospy.ROSInterruptException:
#         pass


if __name__ == '__main__':
    try:
        # Initialize the ROS node in the main thread
        rospy.init_node('stewart_command_sinusoidal', anonymous=True)

        # Run the command loop in a separate thread
        command_thread = threading.Thread(target=send_sinusoidal_command)
        command_thread.start()

        # Start the real-time plotting
        ani = FuncAnimation(plt.gcf(), animate, interval=100)
        plt.show()

        # Wait for the command loop thread to finish
        command_thread.join()

    except rospy.ROSInterruptException:
        pass
