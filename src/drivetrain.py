import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import pi

class Drivetrain(object):

    wheel_center_distance = 106.9
    wheel_diameter = 83
    def __init__(self):
        self.pub1 = rospy.Publisher("/motor_command/speed1",std_msgs.Float64, queue_size=2)
        self.pub2 = rospy.Publisher("/motor_command/speed2", std_msgs.Float64, queue_size=2)
        self.sub = rospy.Subscriber("cmd_vel", Twist, twistCallback)

    def twistCallback(self, data):
        fwd_speed = data.linear.x
        turn_speed = data.angular.z
        left_motor = fwd_speed + turn_speed * wheel_center_distance
        right_motor = fwd_speed - turn_speed * wheel_center_distance
        # apply formula for left and right motors
        left_rpm = left_motor / (pi * wheel_diameter)
        right_rpm = right_motor / (pi * wheel_diameter)

        self.pub1.publish(left_rpm)
        self.pub2.publish(right_rpm)
    
if __name__ == '__main__':
    rospy.init_node('drivetrain')
    c = Drivetrain()
    rospy.spin()





