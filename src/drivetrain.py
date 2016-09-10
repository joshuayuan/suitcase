import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import pi

class Drivetrain(object):

    wheel_center_distance = 106.9 #in mm
    wheel_diameter = 83 #in mm
    def __init__(self):
        self.pubL = rospy.Publisher("/motor_command_left/rpm",std_msgs.Float64, queue_size=2)
        self.pubR = rospy.Publisher("/motor_command_right/rpm", std_msgs.Float64, queue_size=2)
        self.sub = rospy.Subscriber("/follower_velocity_smoother/raw_cmd_vel", geometry_msgs.Twist, twistCallback)

    def twistCallback(self, data):
        fwd_speed = data.linear.x
        turn_speed = data.angular.z
        left_motor = fwd_speed + turn_speed * wheel_center_distance
        right_motor = fwd_speed - turn_speed * wheel_center_distance
        
        left_rpm = left_motor / (pi * wheel_diameter)
        right_rpm = right_motor / (pi * wheel_diameter)

        self.pubL.publish(left_rpm)
        self.pubR.publish(right_rpm)
    
if __name__ == '__main__':
    rospy.init_node('drivetrain')
    c = Drivetrain()
    rospy.spin()





