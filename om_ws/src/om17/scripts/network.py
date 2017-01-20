import rospy
from std_msgs.msg import String

def network():
    pass

if __name__ == "__main__":
    try:
        network()
    except rospy.ROSInterruptException:
        pass