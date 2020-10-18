import rospy
from sensor_msgs.msg import Image



class SnapshotSave:

    def __init__(self, topic_list):
        self.topic_list = topic_list

        #topic <string>, sub <rospy.Subscriber>
        self.topic_subscriber_map = {}
        #add all topics to a callback
        for topics in topic_list:
            rospy.loginfo("Subscribing to '{}'".format(topics))
            sub = rospy.Subscriber(topics, Image, self.image_callback)
            self.topic_subscriber_map[topics] = sub

    def image_callback(self, msg):
        pass

        
