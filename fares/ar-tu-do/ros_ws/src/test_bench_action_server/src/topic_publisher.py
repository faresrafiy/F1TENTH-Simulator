#!/usr/bin/env python

from email import message
from unittest import TestCase
import rospy
from geometry_msgs.msg import PoseStamped
from test_bench_action_server.msg import TestBenchActionGoal


def publisher(data):

    #pub = rospy.Publisher('test_bench_action_server/goal', TestBenchActionGoal, queue_size=1)
    message = TestBenchActionGoal()
    message.goal.target_pose = data

    #rospy.loginfo("Target X: %s",data.pose.position.x)
    #rospy.loginfo("Target Y: %s",data.pose.position.y)

    #rospy.loginfo("Target X: %s",message.goal.target_pose.pose.position.x)
    #rospy.loginfo("Target Y: %s",message.goal.target_pose.pose.position.y)
    pub.publish(message)
    pub.unregister
    print("De topic is published")

    #rospy.sleep(10)



if __name__ == '__main__':
    rospy.init_node("topic_publisher")

    topic_to_subscribe = rospy.get_param('~topic_to_subscribe')
    topic_to_publish = rospy.get_param('~topic_to_publish')

    global pub
    pub = rospy.Publisher(topic_to_publish, TestBenchActionGoal, queue_size=1)

    rospy.Subscriber(topic_to_subscribe, PoseStamped, publisher, queue_size=1)
    rospy.spin()