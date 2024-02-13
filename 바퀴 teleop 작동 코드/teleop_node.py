import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

rospy.init_node('teleop_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

key_mapping = {
    's': (2.0, 0),    # Forward
    'w': (-2.0, 0),   # Backward
    'd': (0, 0.025),    # Turn left
    'a': (0, -0.025),   # Turn right
}

twist = Twist()
current_twist = Twist()  # 현재 값 저장
current_twist_tmp = Twist()  # 현재 값 저장

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

while not rospy.is_shutdown():
    key = getKey()
    print("teleop test \n")
    if key in key_mapping:
        # 현재 값에 추가
        current_twist.linear.x += key_mapping[key][0]
        current_twist.angular.z += key_mapping[key][1]
        print("current_twist.linear.x ", current_twist.linear.x)
        print("current_twist.angular.z ", current_twist.angular.z)
        pub.publish(current_twist)
        time.sleep(0.05)
        pub.publish(current_twist)
    elif key == 'x':
        current_twist.linear.x = 0
        current_twist.angular.z = 0
        pub.publish(current_twist)
        time.sleep(0.05)
        pub.publish(current_twist)
    elif key == '\x03':
        break

# Stop the robot before exiting
twist.linear.x = 0
twist.angular.z = 0
pub.publish(twist)

# Restore terminal settings
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
