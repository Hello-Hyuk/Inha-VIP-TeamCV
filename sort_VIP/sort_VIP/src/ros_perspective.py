import rospy

from std_msgs.msg import String
from sort_VIP.msg import SortedBbox

def talker(distance):
    pub=rospy.Publisher('destance_msg',String,queue_size=100)
    rate=rospy.Rate(10)
    pub.publish(distance)

def callback():
    hi=1

def listener():
    rospy.Subscriber('sort_msg',SortedBbox,callback)
    rospy.spin()

rospy.init_node('perspective_node',anonymous=True)
distance=1
talker(distance)
listener()