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
import tf_transformations
import numpy as np
from geometry_msgs.msg import Point
import math
# import time

# Global variables
global pos, posflag, odom, odomflag, start_time, end_time, flag, direc_pose, goal_pos, goalposflag
pos = Float32MultiArray()
goal_pos = Float32MultiArray()
direc_pose = Float32MultiArray()
odom = Odometry()
odomflag = False
posflag= False
goalposflag = False


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
            self.create_subscription(Float32MultiArray, '/direction_point', self.dir, 10)
            self.create_subscription(Float32MultiArray, '/final_goal_point_cpp', self.final_goal, 10)
            self.dir_pos = Point()
            self.dirflag = False
        def goal(self, msg):
            global new_pos, flag, posflag        
            new_pos = msg
            #print ("goal subscriber")
            posflag = True

        def dir(self, msg):
            self.dir_pos = msg
            self.dirflag = True

        def final_goal(self, msg):
            global new_goal_pos, goalposflag
            new_goal_pos = msg
            goalposflag = True

class Goal_Publisher(Node):
        def __init__(self):
            super().__init__('goal_pose_publisher')
            self.goal_viz = self.create_publisher(Marker, '/goal_viz_final', 1)
            self.goal_viz2 = self.create_publisher(Marker, '/direction_marker', 1)


class LaneFollower:
    def __init__(self):
        self.navigator = BasicNavigator()
        self.marker = Marker()
        self.marker2 = Marker()
        self.heading = 0.0
    
    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def run(self):
        global pos, odom, odomflag, start_time, end_time, flag, direc_pose, goal_pos
        flag = False
        # self.navigator.waitUntilNav2Active()
        init_pose = PoseStamped()
        self.navigator.setInitialPose(init_pose)

        # converting position from base_link to map frame
        position = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        
        orientation = odom.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        
        # Convert goal point from base_link frame to map frame
        goal_in_base_link = np.array([pos.data[0], pos.data[1], pos.data[2]])
        dir_in_base_link = np.array([direc_pose.data[0], direc_pose.data[1], direc_pose.data[2]])    
        goal_in_map_frame = np.dot(rotation_matrix, goal_in_base_link) + position
        dir_in_map_frame = np.dot(rotation_matrix, dir_in_base_link) + position
        final_goal = np.array([goal_in_map_frame])
        final_dir_point = np.array([dir_in_map_frame])
        
        final_goal_base_link = np.array([goal_pos.data[0], goal_pos.data[1], goal_pos.data[2]])
        final_goal_in_map_frame = np.dot(rotation_matrix, final_goal_base_link) + position
        final_final_goal = np.array([final_goal_in_map_frame])

        self.heading = ((180/math.pi) * math.atan((final_goal[0][1] - final_dir_point[0][1])/(final_goal[0][0] - final_dir_point[0][0])))
        print("calculated heading = ", self.heading)
        quaternion_dir = self.euler_to_quaternion(self.heading, 0.0, 0.0)

        gpoint = Point()
        gpoint.x = final_goal[0][0]
        gpoint.y = final_goal[0][1]
        gpoint.z = final_goal[0][2]

        dpoint = Point()
        dpoint.x = final_dir_point[0][0]
        dpoint.y = final_dir_point[0][1]
        dpoint.z = final_dir_point[0][2]

        # Publishing marker to visualize goal position
        self.marker.header.frame_id = "map"
        self.marker.id = 0 
        self.marker.type = 0
        # self.marker.pose.position.x = self.goal_pose.pose.position.x 
        # self.marker.pose.position.y = self.goal_pose.pose.position.y
        # self.marker.pose.position.z = self.goal_pose.pose.position.z
        # self.marker.pose.orientation.x = quaternion_dir[0]
        # self.marker.pose.orientation.y = quaternion_dir[1]
        # self.marker.pose.orientation.z = quaternion_dir[2]
        # self.marker.pose.orientation.w = quaternion_dir[3]
        self.marker.scale.x = 1.0 
        self.marker.scale.y = 0.1 
        self.marker.scale.z = 0.1 
        self.marker.color.a = 1.0 
        self.marker.color.r = 0.0 
        self.marker.color.g = 1.0 
        self.marker.color.b = 0.0 

        self.marker.points = [dpoint, gpoint]

        

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map' # changed from odom to map frame
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        
        self.goal_pose.pose.position.x = final_goal[0][0]
        self.goal_pose.pose.position.y = final_goal[0][1]
        self.goal_pose.pose.position.z = final_goal[0][2]
        self.goal_pose.pose.orientation.x = odom.pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = odom.pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = odom.pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = odom.pose.pose.orientation.w

        self.marker2.header.frame_id = "map"
        self.marker2.id = 0 
        self.marker2.type = 0
        self.marker2.pose.position.x = self.goal_pose.pose.position.x 
        self.marker2.pose.position.y = self.goal_pose.pose.position.y
        self.marker2.pose.position.z = self.goal_pose.pose.position.z
        self.marker2.pose.orientation.x = self.goal_pose.pose.orientation.x
        self.marker2.pose.orientation.y = self.goal_pose.pose.orientation.y
        self.marker2.pose.orientation.z = self.goal_pose.pose.orientation.z
        self.marker2.pose.orientation.w = self.goal_pose.pose.orientation.w
        self.marker2.scale.x = 1.0 
        self.marker2.scale.y = 0.1 
        self.marker2.scale.z = 0.1 
        self.marker2.color.a = 1.0 
        self.marker2.color.r = 0.0 
        self.marker2.color.g = 1.0 
        self.marker2.color.b = 0.0 
        #self.goal_viz.publish(self.marker)

        self.goal_pose_2 = PoseStamped()
        self.goal_pose_2.header.frame_id = 'map' # changed from odom to map frame
        self.goal_pose_2.header.stamp = self.navigator.get_clock().now().to_msg()
        
        
        self.goal_pose_2.pose.position.x = final_dir_point[0][0]
        self.goal_pose_2.pose.position.y = final_dir_point[0][1]
        self.goal_pose_2.pose.position.z = final_dir_point[0][2]
        self.goal_pose_2.pose.orientation.x = odom.pose.pose.orientation.x
        self.goal_pose_2.pose.orientation.y = odom.pose.pose.orientation.y
        self.goal_pose_2.pose.orientation.z = odom.pose.pose.orientation.z
        self.goal_pose_2.pose.orientation.w = odom.pose.pose.orientation.w

        


        

        print("navigating to goal")
        # self.navigator.goToPose(self.goal_pose)
        #self.navigator.goThroughPoses([self.goal_pose_2, self.goal_pose])
        self.navigator.goToPose(self.goal_pose_2)
        if (self.navigator.isTaskComplete()) :
            self.goal_pose.pose.orientation.x = odom.pose.pose.orientation.x
            self.goal_pose.pose.orientation.y = odom.pose.pose.orientation.y
            self.goal_pose.pose.orientation.z = odom.pose.pose.orientation.z
            self.goal_pose.pose.orientation.w = odom.pose.pose.orientation.w
            self.navigator.goToPose(self.goal_pose)
            if (self.navigator.isTaskComplete()):
                flag = True

def main(args=None):
    global flag, odom, pos, odomflag, posflag, new_pos, direc_pose, goal_pos, goalposflag, new_goal_pos
    rclpy.init(args=args)
    flag = True
    odom_subscriber = Odom_Subscriber()
    goal_pos_subscriber = Goal_Pose_Subscriber()
    lane_follower = LaneFollower()
    goal_publisher = Goal_Publisher()
    goal_threshold = 1.2
    posflag = False
    goalposflag = False
    rclpy.spin_once(goal_pos_subscriber)
    #lane_follower.navigator.waitUntilNav2Active()
    while True:
        rclpy.spin_once(odom_subscriber)
        rclpy.spin_once(goal_pos_subscriber)
        if odomflag:
            current_pos = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z])
            current_heading = odom.pose.pose.orientation.z
            print(current_heading, lane_follower.heading)
            odomflag = False
        else: 
            print("Odom not Received")
            while (not odomflag):
                rclpy.spin_once(odom_subscriber)
            continue
        
        if flag and goal_pos_subscriber.dirflag:
            pos = new_pos
            goal_pos = new_goal_pos
            direc_pose = goal_pos_subscriber.dir_pos
            if (posflag and (pos.data[0]> 0 ) ):
                posflag = False
                lane_follower.run()
                goal_publisher.goal_viz.publish(lane_follower.marker)
                goal_publisher.goal_viz2.publish(lane_follower.marker2)
            else:
                print("goal pose not received")
                continue
        if (len(pos.data) > 2):
            pos_list = np.array([pos.data[0],pos.data[1],pos.data[2]])
        if ((np.linalg.norm(pos_list-current_pos) < goal_threshold) and ((lane_follower.heading - current_heading) < 0.6)) or (lane_follower.navigator.isTaskComplete()) :
            print ("threshold reached")
            flag = True


        

if __name__ == '__main__':
    main()
