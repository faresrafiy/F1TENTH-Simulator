#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from test_bench_action_server.msg import TestBenchAction, TestBenchActionGoal, TestBenchGoal
from move_base_msgs.msg import MoveBaseGoal

def action_client(goal_list) :
    # Create Action Client
    client = actionlib.SimpleActionClient('test_bench_action_server', TestBenchAction)
    # Create Publisher
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=5)
    rospy.sleep(0.500)


    # Waits until Action Server has started up and started listening for goals
    client.wait_for_server()
    print("Server is on")
    results = []
    for i  in goal_list :
        time = rospy.Time.now()
        i.header.stamp = time
        
        # Publish on topic move_base
        pub.publish(i)
        print("Goal published")
        rospy.sleep(0.050)
        
        # Creating goal message
        goal_action_server = TestBenchGoal()
        goal_action_server.target_pose = i

        # Send goal to the Action Server
        client.send_goal(goal_action_server)
        # Waits for the server to finish performing the action
        client.wait_for_result()

        results.append(client.get_result())
    return results


if __name__ == "__main__":
    rospy.init_node("action_client")
    rospy.sleep(0.200)
    


    # print("-----------------------------------------------------------")
    # print("Welcome to the Test Bench")
    # print("--------------------------")
    # print("The metrics are the time, the distance traveled, ")
    # print("the average velocity and the maximum velocity.")
    # print("You can publish goals using an Action Client or RViz. ")
    # print("-----------------------------------------------------------")

    goal_list = []

    goal1 = PoseStamped()

    # 1ste point to get until the Finish Line
    # goal1.header.frame_id = "map"
    # goal1.pose.position.x = 1.77863669395
    # goal1.pose.position.y = 0.152771949768
    # goal1.pose.position.z = 0.0
    # #z: -0.0600690511667
    # goal1.pose.orientation.w = 0.998194224133
    # goal_list.append(goal1)
    
    goal1.header.frame_id = "map"
    goal1.pose.position.x = 1.98562955856
    goal1.pose.position.y = 0.196025371552
    goal1.pose.position.z = 0.0
    #z: -0.0600690511667
    goal1.pose.orientation.w = 0.998194224133
    goal_list.append(goal1)


    # 2th point
    goal2 = PoseStamped()
    goal2.header.frame_id = "map"
    goal2.pose.position.x = 1.460702419
    goal2.pose.position.y = 16.142496109
    goal2.pose.position.z = 0.0
    goal2.pose.orientation.z = 0.99999723163
    goal2.pose.orientation.w = 0.0023530262837
    goal_list.append(goal2)
    
    # 3th point
    goal3 = PoseStamped()
    goal3.header.frame_id = "map"
    goal3.pose.position.x = -17.6036148071
    goal3.pose.position.y = 15.7120847702
    goal3.pose.position.z = 0.0
    goal3.pose.orientation.z = -0.755936898452
    goal3.pose.orientation.w = 0.654644487916
    goal_list.append(goal3)
    
    # 4th point
    # goal4 = PoseStamped()
    # goal4.header.frame_id = "map"
    # goal4.pose.position.x = 1.955078125
    # goal4.pose.position.y = 0.123607635498
    # goal4.pose.position.z = 0.0
    # goal4.pose.orientation.z = -0.0374480933084
    # goal4.pose.orientation.w = 0.999298574155
    # goal_list.append(goal4)

    goal4 = PoseStamped()
    goal4.header.frame_id = "map"
    goal4.pose.position.x = 1.98562955856
    goal4.pose.position.y = 0.196025371552
    goal4.pose.position.z = 0.0
    goal4.pose.orientation.w = 0.998194224133
    goal_list.append(goal4)


    results = action_client(goal_list)

    j = 1
    for i in results :
        print("%s" %j)
        print("Goal achieved in time: %s s" %i.timePassed)
        print("Distance Travaled: %s" %i.distanceTravaled)
        print("Average Velocity: %s" %i.averageVelocity)
        print("Maximum Velocity: %s" %i.maxVelocity)
        j += 1

    print("end")