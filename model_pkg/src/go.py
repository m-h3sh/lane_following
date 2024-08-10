#!/usr/bin/python3
# import sensor_msgs.point_cloud2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
# from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import tf_transformations
import numpy as np
# import time

# Global variables
global pos, posflag, odom, odomflag, start_time, end_time, flag
pos = Point()
odom = Odometry()
odomflag = False
posflag= False


# class Subscriber(Node):
#     def __init__(self):
#         super().__init__('lf')
#         global modoflag
#         modoflag == False
#         self.create_subscription(Float32MultiArray, '/far_ipm', self.goal, 10)
#         self.create_subscription(Odometry, '/zed/zed_node/transformed_odom', self.odom, 10)
        

#     def odom(self, msg):
       
#         global modo, modoflag
#         if modoflag == False:
            
#             modo = msg
#             modoflag = True
            

#     def goal(self, msg):
#         global pos, start_time
#         # if msg is None:
#             # Aligner().run()
#         pos = msg
        
#         # LaneFollower().run()


class Odom_Subscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.create_subscription(Odometry, '/odom', self.odom, 10)
            
    def odom(self, msg):
       
        global odom, odomflag
        if odomflag == False:
            #print ("odom subscriber")
            odom = msg
            odomflag = True

class Goal_Pose_Subscriber(Node):
        def __init__(self):
            super().__init__('goal_pose_subscriber')
            self.create_subscription(Float32MultiArray, '/far_ipm', self.goal, 10)
        def goal(self, msg):
            global new_pos, flag, posflag         
            new_pos = msg
            #print ("goal subscriber")
            posflag = True

class Goal_Publisher(Node):
        def __init__(self):
            super().__init__('goal_pose_publisher')
            self.goal_viz = self.create_publisher(Marker, '/goal_viz_final', 1)


            
            

        


class LaneFollower:
    def __init__(self):
        self.navigator = BasicNavigator()
        self.marker = Marker()
        

    def run(self):
        global pos, odom, odomflag, start_time, end_time, flag
        flag = False
        # self.navigator.waitUntilNav2Active()
        init_pose = PoseStamped()
        self.navigator.setInitialPose(init_pose)
        
            
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map' # changed from odom to map frame
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        
        self.goal_pose.pose.position.x = (pos.data[0] + odom.pose.pose.position.x)
        self.goal_pose.pose.position.y = (pos.data[1] + odom.pose.pose.position.y)
        self.goal_pose.pose.position.z = pos.data[2] + odom.pose.pose.position.z
        self.goal_pose.pose.orientation.x = odom.pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = odom.pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = odom.pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = odom.pose.pose.orientation.w

        self.marker.header.frame_id = "map"
        self.marker.id = 0 
        self.marker.type = 2
        self.marker.pose.position.x = self.goal_pose.pose.position.x 
        self.marker.pose.position.z = self.goal_pose.pose.position.y 
        self.marker.pose.orientation.x = 0.0 
        self.marker.pose.orientation.y = 0.0 
        self.marker.pose.orientation.z = 0.0 
        self.marker.pose.orientation.w = 1.0 
        self.marker.scale.x = 1.0 
        self.marker.scale.y = 0.1 
        self.marker.scale.z = 0.1 
        self.marker.color.a = 1.0 
        self.marker.color.r = 0.0 
        self.marker.color.g = 1.0 
        self.marker.color.b = 0.0 
        #self.goal_viz.publish(self.marker)

        print("navigating to goal")
        self.navigator.goToPose(self.goal_pose)
        if (self.navigator.isTaskComplete()) :
            flag = True

        


       




def main(args=None):
    global flag, odom, pos, odomflag, posflag, new_pos
    rclpy.init(args=args)
    flag = True
    odom_subscriber = Odom_Subscriber()
    goal_pos_subscriber = Goal_Pose_Subscriber()
    lane_follower = LaneFollower()
    goal_publisher = Goal_Publisher()
    goal_threshold = 0.5
    posflag = False
    rclpy.spin_once(goal_pos_subscriber)
    #lane_follower.navigator.waitUntilNav2Active()
    while True:
        rclpy.spin_once(odom_subscriber)
        rclpy.spin_once(goal_pos_subscriber)
        if odomflag:
            current_pos = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z])
            odomflag = False
        else: 
            print("Odom not Received")
            while (not odomflag):
                rclpy.spin_once(odom_subscriber)
            continue
        
        if flag :
            pos = new_pos
            if (posflag and (pos.data[0]> 0 ) ):
                posflag = False
                lane_follower.run()
                goal_publisher.goal_viz.publish(lane_follower.marker)
            else:
                print("goal pose not received")
                continue

        pos_list = np.array([pos.data[0],pos.data[1],pos.data[2]])
        if (np.linalg.norm(pos_list-current_pos) < goal_threshold) or (lane_follower.navigator.isTaskComplete()) :
            print ("threshold reached")
            flag = True


        

if __name__ == '__main__':
    main()
