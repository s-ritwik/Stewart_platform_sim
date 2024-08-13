#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import time

def send_sinusoidal_command():
    pub = rospy.Publisher('stewart/platform_pose', Twist, queue_size=10)
    rospy.init_node('stewart_command_sinusoidal', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    start_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time() - start_time

        # Sinusoidal parameters
        amplitude = 0.5  # Amplitude of the sine wave (in radians for roll/pitch and meters for heave)
        frequency = 0.5  # Frequency of the sine wave

        # Calculating sinusoidal values for roll, pitch, and heave
        roll = amplitude * math.sin(frequency * current_time)
        pitch = amplitude * math.sin(frequency * current_time + math.pi / 2)  # 90 degrees out of phase
        heave = 0.2 + amplitude * math.sin(frequency * current_time)  # Constant offset + sinusoidal

        command = Twist()
        command.angular.x = roll
        command.angular.y = pitch
        command.linear.z = heave

        rospy.loginfo("Sending roll: %s, pitch: %s, heave: %s", roll, pitch, heave)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_sinusoidal_command()
    except rospy.ROSInterruptException:
        pass

