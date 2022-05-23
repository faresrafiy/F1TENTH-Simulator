#! /usr/bin/env python

import math
import rospy
import tf
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from test_bench_action_server.msg import TestBenchAction, TestBenchResult


class ActionServer():
    '''
    The ActionServer class contains the Action Server.
    The Action Server is able to measure the time, the distance travaled, the average velocity and the maxixmum velocity.
    The Action Server is able to get this information by subcribing on the topic /odom.
    The Action Server uses the current position of the car to determine when the car has reached the target position.
    If the car has reached the target position, the Action Server markes the goal as achieved.

    Parameters
    ----------
    accuracy_along_x : float
        This variable is used to determine if the x position of the car 
        is equal to the x position of the target up until a certain accuracy.
        It is impossible that these two positions will be completely equal.

    accuracy_along_y : float
        This variable is used to determine if the y position of the car 
        is equal to the y position of the target up until a certain accuracy.
        It is impossible that these two positions will be completely equal.
    
    Attributes
    ----------
    action_server : SimpleActionServer
        This variable holds the Action Server object.
    sub : Subcriber
        This variable holds the Subcriber object which subcribes to the topic /odom.
    listener : TransformListener
        This vairiable holds the TransformListener object and is used 
        to transform points from the odom frame to the map frame.
    accuracy_along_x : float
        This variable stores the argument acuaracy_along_x.
    accuracy_along_y : float
        This variable stores the argument acuaracy_along_.
    goal_received : boolean
        This variable indicates that a goal message is received.
    goal_achieved : boolaen
        This variable indicates whether the goal is achieved.
    start_position_determined : boolean
        This variable indicates whether the start position is determined.
    goal_pose : geometry_msgs.msg/Pose
        This variable is used to store the goal pose of the goal message.
    start_time : float
        This variable stores the start time.
    end_time : float
        This variable stores the end time.
    start_position_x : float
        This variabel stores the x coordinate of the start position.
    start_position_y : float
        This variabel stores the y coordinate of the start position.
    end_postion_x : float
        This variabel stores the x coordinate of the end position.
    end_postion_y : float
        This variabel stores the y coordinate of the end position.
    max_velocity : float
        This variable stores the maximum velocity obtained during the trajectory.
    points_list : list
        This list stores the points published on the topic /odom during the trajectory
        to later determine the travaled distance.
    '''
    
    def __init__(self, accuracy_along_x, accuracy_along_y):
        # Create Action Server
        self.action_server = actionlib.SimpleActionServer(
            "test_bench_action_server", TestBenchAction, execute_cb=self.action_server_cb, auto_start=False)
        # Start Action Server
        self.action_server.start()
        # Subscribe to topic /odom to get position information of the car
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_subscriber_cb, queue_size=10)
        self.listener = tf.TransformListener()
        # Variables
        self.accuracy_along_x = accuracy_along_x
        self.accuracy_along_y = accuracy_along_y
        self.goal_received = False
        self.goal_achieved = False
        self.start_position_determined = False
        self.goal_pose = Pose()
        # Time variables
        self.start_time = rospy.get_rostime()
        self.end_time = rospy.get_rostime()
        # Position variables
        self.start_position_x = 0
        self.start_position_y = 0
        self.end_position_x = 0
        self.end_position_y = 0
        # Max velocity variable 
        self.max_velocity = 0
        # Points list
        self.points_list = []

        
    # Odom Subsriber callback    
    def odom_subscriber_cb(self, data):
        # Determine the position of the car at the start
        if(self.goal_received and not self.start_position_determined):
            point = PoseStamped()

            point.header = data.header
            point.pose = data.pose.pose
            # Transform point from odom frame to map frame
            self.listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(5))
            new_point = self.listener.transformPose("/map", point)

            self.points_list.append(new_point)
            
            self.start_position_x = new_point.pose.position.x
            self.start_position_y = new_point.pose.position.y
            self.previous_position_x = self.start_position_x
            self.previous_position_y = self.start_position_y

            print("Start Time: %i,%i s" %(self.start_time.secs,self.start_time.nsecs))
            print("Start Position X: %s" %self.start_position_x)
            print("Start Position Y: %s" %self.start_position_y)
            print("-----------------")

            self.start_position_determined = True

        # Determine the position of the car during the trajectory
        if(self.goal_received and not self.goal_achieved):
            point = PoseStamped()

            point.header = data.header
            point.pose = data.pose.pose
            # Transform point from odom frame to map frame
            self.listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(5))
            new_point = self.listener.transformPose("/map", point)

            # Determine the maximum velocity
            self.max_velocity = max(self.max_velocity, data.twist.twist.linear.x, data.twist.twist.linear.y)

            self.points_list.append(new_point)

            if(abs(new_point.pose.position.x - self.goal_pose.position.x) < self.accuracy_along_x 
                and abs(new_point.pose.position.y - self.goal_pose.position.y) < self.accuracy_along_y):
                    self.end_time = rospy.get_rostime()
                    print("End Time: %i.%i s" %(self.end_time.secs,self.end_time.nsecs))
                    self.goal_achieved = True

                    self.end_position_x = new_point.pose.position.x
                    self.end_position_y = new_point.pose.position.y

                    print("End Position X: %s" %self.end_position_x)
                    print("End Position Y: %s" %self.end_position_y)


    # Action Server callback  
    def action_server_cb(self, goal):
        self.goal_received = True
        self.start_position_determined = False
        success = True
        print("Goal Received!")
        
        self.start_time = goal.target_pose.header.stamp
        self.goal_pose = goal.target_pose.pose
        print("Target X: %s" %goal.target_pose.pose.position.x)
        print("Target Y: %s" %goal.target_pose.pose.position.y)

        # Stay in while lus until goal is achieved
        while not self.goal_achieved :
            # When a new goal is received before the previous goal is achieved
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                print("Preemted")
                success = False
                self.goal_received = False
                break
            rospy.sleep(0.050)

        # When the goal is achieved, calculate the metrics
        if self.goal_achieved and success:
                # Result message
                result = TestBenchResult()
                # Calculate the time lapse
                time_passed_secs = self.end_time.secs - self.start_time.secs
                time_passed_nsecs = self.end_time.nsecs - self.start_time.nsecs
                if(time_passed_nsecs < 0):
                    time_passed_secs = time_passed_secs - 1
                    time_passed_nsecs = 1000000000 + time_passed_nsecs
                time_passed = time_passed_secs + (time_passed_nsecs*pow(10, -9))
                print("Goal achieved in time: %s s" %time_passed)
                result.timePassed = time_passed

                #Calculate the distance traveled
                length = len(self.points_list)
                if(length > 1):
                    previous_position_x = self.points_list[0].pose.position.x
                    previous_position_y = self.points_list[0].pose.position.y
                    i = 1
                    total_distance = 0

                    while i < length:
                        current_position_x = self.points_list[i].pose.position.x
                        current_position_y = self.points_list[i].pose.position.y

                        distance = math.sqrt((current_position_x - previous_position_x)**2 + (current_position_y - previous_position_y)**2)
                        total_distance = total_distance + distance

                        previous_position_x = current_position_x
                        previous_position_y = current_position_y
                        i += 1
                    print("Distance Travaled: %s" %total_distance)
                    result.distanceTravaled = total_distance

                    #Calculate the average velocity
                    average_velocity = total_distance / time_passed
                    print("Average Velocity: %s" %average_velocity)
                    result.averageVelocity = average_velocity

                # The maxixmum velocity
                print("Maximum Velocity: %s" %self.max_velocity)
                result.maxVelocity = self.max_velocity

                self.action_server.set_succeeded(result)
                self.goal_achieved = False
                self.goal_received = False
                self.total_distance = 0
                self.max_velocity = 0
                del self.points_list[:]
                print("-----------------------------------------------------------")


if __name__ == "__main__":
    rospy.init_node("action_server")

    # To get parameters from the launch file
    accuracy_along_x = rospy.get_param('~accuracy_along_x')
    accuracy_along_y = rospy.get_param('~accuracy_along_y')

    print("-----------------------------------------------------------")
    print("Welcome to the Test Bench")
    print("--------------------------")
    print("The metrics are the time, the distance traveled, ")
    print("the average velocity and the maximum velocity.")
    print("You can publish goals using an Action Client or RViz. ")
    print("-----------------------------------------------------------")

    action_server = ActionServer(accuracy_along_x, accuracy_along_y)

    rospy.spin()