import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
class PID:
    def __init__(self, setpoint=0.0, kp=0.09, ki=0.0, kd=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
    
    def update(self, current_value, dt=1.0):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

# Set the target
Y_sub = 0.0
YAW_sub = math.pi
X_sub = -5.0
# Initialize PID controllers for Y and YAW
PID_Y = PID(setpoint=Y_sub)
PID_YAW = PID(setpoint=YAW_sub)
PID_X = PID(setpoint=X_sub)
# Global variables for positions
x_position = 0.0
y_position = 0.0
yaw_position = 0.0

Check = 1
Check_x = 1

def calu_pid():
    global Check
    global Check_x
    
    twist = Twist()
    twist.linear.y = PID_Y.update(y_position)
    twist.angular.z = PID_YAW.update(yaw_position)
    
    if twist.angular.z < 0.0009 or Check == 0:
        Check = 0
        twist.angular.z = 0
        twist.linear.x = PID_X.update(x_position) * -1

    if (twist.linear.x < 0.001 and twist.linear.x != 0) or Check_x == 0:
        Check_x = 0
        twist.linear.x = 0    
    
    rospy.loginfo(twist.angular.z)
    rospy.loginfo(twist.linear.x)
    pub_Twist.publish(twist)

def odom_callback(msg):
    global x_position
    global y_position
    global yaw_position
    
    x_position = msg.pose.pose.position.x
    y_position = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    _, _, yaw_position = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    calu_pid()

def main():
    global pub_Twist
    
    rospy.init_node('pid_controller_node')

    pub_Twist = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.loginfo("PID Controller Node Started")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
