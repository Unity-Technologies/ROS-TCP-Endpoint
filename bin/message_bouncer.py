import rospy
from std_msgs.msg import String


def bounce_string(msg: String):
    print(f"bouncing {msg}")
    string_pub.publish(msg)


rospy.init_node("message_bouncer")
string_pub = rospy.Publisher("test/string/in", String, queue_size=0)
rospy.Subscriber("test/string/out", String, bounce_string)
rospy.spin()
