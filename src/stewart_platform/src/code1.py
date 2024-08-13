#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def send_command(roll, pitch, heave):
    pub = rospy.Publisher('stewart/platform_pose', Twist, queue_size=10)
    rospy.init_node('stewart_command_sender', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        command = Twist()
        command.angular.x = roll
        command.angular.y = pitch
        command.linear.z = heave

        rospy.loginfo("Sending roll: %s, pitch: %s, heave: %s", roll, pitch, heave)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example values for roll, pitch, and heave
        roll = 0   # Roll in radians
        pitch = 0  # Pitch in radians
        heave = 0  # Heave in meters
        send_command(roll, pitch, heave)
    except rospy.ROSInterruptException:
        pass

