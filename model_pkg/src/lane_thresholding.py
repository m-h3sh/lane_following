from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import math
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
 
class zedNODE(Node):
 
    def __init__(self):
        super().__init__('pubsub')
        self.bridge = CvBridge()

        self.nav = BasicNavigator()
        
        #creating publishers and subscribers
        self.final_img_publisher = self.create_publisher(Image, '/model_lanes', 1)
        self.final_goal_pub = self.create_publisher(PoseStamped, 'final_goal_point', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_point', 1)
        self.dir_point_pub = self.create_publisher(PoseStamped, '/dir_point', 1)
        self.camerasub = self.create_subscription(Image, '/camera_forward/image_raw', self.camera_callack, 10)
        self.cinfosub = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.infocallback, 10)

        self.odomsub = self.create_subscription(Odometry, '/odom', self.odomcallback, 10)
        self.camerasub
        self.cinfosub
        self.lane = 1 # 1=right, -1=left

    def quaternion_to_euler(self, x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    def odomcallback(self, msg):
        r, p, y = self.quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        #print(y)
    
    def getPerpCoord(self, lane, aX, aY, bX, bY, length):
        vX = bX-aX
        vY = bY-aY
        mag = math.sqrt(vX*vX + vY*vY)
        vX = vX / mag
        vY = vY / mag
        temp = vX
        vX = 0-vY
        vY = temp
        cX = bX + 2*lane*(vX * length)
        cY = bY + 2*lane*(vY * length)
        dX = aX + lane*(vX * length)
        dY = aY + lane*(vY * length)
        return int(cX), int(cY), int(dX), int(dY)
 
    def camera_callack(self, data):
        self.cvimage = self .bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image
        cv2.imshow("original image", self.cvimage)
        cv2.waitKey(1)
        imgheight = self.cvimage.shape[0]
        imgwidth = self.cvimage.shape[1]
 
        gray_img = cv2.cvtColor(self.cvimage, cv2.COLOR_BGR2GRAY)
        binary_img = cv2.inRange(gray_img, 120, 140)

        binary_img = cv2.medianBlur(binary_img, 11)

        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        black_img = np.zeros((imgheight, imgwidth, 3), dtype=np.uint8)
        if (len(contours) > 0):
            maxcontour = contours[0]
            secondcontour = contours[0]
            for contour in contours:
                if len(contour) > len(maxcontour):
                    maxcontour = contour
                if len(contour) < len(maxcontour) and len(contour) > len(secondcontour):
                    secondcontour = contour

            # finding midpoint of the contour
            M = cv2.moments(maxcontour)
            if (M["m00"] != 0.0):
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = 0
                cy = 0
            
            # getting the bottom most point of the lane contour
            # checking if the lane bottom point is to the left or right of the center to determine left or right lane
            bottom_point = list(maxcontour[maxcontour[:, :, 1].argmax()][0])
            if bottom_point[0] <= 0:
                bottom_point[0] = 0
            if bottom_point[0] >= imgwidth:
                bottom_point[0] = imgwidth
            print(bottom_point)

            if (bottom_point[0] < (imgwidth/2)):
                self.lane = -1
                print("left lane")
                second_point = list(maxcontour[maxcontour[:, :, 1].argmin()][0])
                if (second_point[0] < 20):
                    goal_point = (second_point[0] - 30, second_point[1] + 100)
                    dir_point = (cx - 30, cy + 100)
                else:   
                    goal_point = (second_point[0] + 30, second_point[1] + 50)
                    dir_point = (cx + 180, cy + 50)
            else:
                self.lane = 1
                print("right lane")
                second_point = list(maxcontour[maxcontour[:, :, 0].argmin()][0])
                if (second_point[0] < 20):
                    goal_point = (second_point[0] + 30, second_point[1] + 100)
                    dir_point = (cx + 30, cy + 100)
                else:
                    goal_point = (second_point[0] - 30, second_point[1] + 50)
                    dir_point = (cx - 180, cy + 50)
            cv2.circle(black_img, bottom_point, 7, (255, 255, 255), -1)

            cv2.circle(black_img, dir_point, 7, (0, 0, 255), -1)
            cv2.circle(black_img, second_point, 7, (255, 255, 255), -1)
            cv2.circle(black_img, goal_point, 7, (0, 0, 255), -1)
            cv2.drawContours(black_img, maxcontour, -1, (0,255,0), 3)

            

            #-------------------------------------------------------- new algo
            bottom_point[0] = int((bottom_point[0] + second_point[0])/2)
            bottom_point[1] = int((bottom_point[1] + second_point[1])/2)
            cv2.line(black_img, bottom_point, second_point, (255, 255, 255))
            

            #finding the point on the perpendicular
            d = 150
        
            x3, y3, x2, y2 = self.getPerpCoord(self.lane, second_point[0], second_point[1], bottom_point[0], bottom_point[1], d)

            toppathpoint = [int(x2), int(y2)]
            bottompathpoint = [int(x3), int(y3)]
            if (bottompathpoint[0] < 0):
                bottompathpoint[0] = 0
            if (bottompathpoint[0] > imgwidth):
                bottompathpoint[0] = imgwidth
            if (bottompathpoint[1] > imgheight):
                bottompathpoint[1] = imgheight
                if (self.lane == -1):
                    bottompathpoint[0] = bottom_point[0] + 120
                else:
                    bottompathpoint[0] = bottom_point[0] - 120

            
            cv2.circle(black_img, toppathpoint, 7, (0, 255, 255), -1)
            cv2.circle(black_img, bottompathpoint, 7, (0, 255, 255), -1)

            


            #-----------------------------------------------------------------

            cv2.imshow("contours", black_img)
            cv2.waitKey(1)

            # publishing the goal pose as a PoseStamped on the topic /goal_point
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(toppathpoint[0]) #float(second_point[0])
            goal_pose.pose.position.y = float(toppathpoint[1]) #float(second_point[1]) 
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            


            second_point_pose = PoseStamped()
            second_point_pose.header.frame_id = 'map'
            second_point_pose.header.stamp = self.nav.get_clock().now().to_msg()
            second_point_pose.pose.position.x = float(toppathpoint[0]) #float(second_point[0])
            second_point_pose.pose.position.y = float(toppathpoint[1]) #float(second_point[1]) 
            second_point_pose.pose.orientation.x = 0.0
            second_point_pose.pose.orientation.y = 0.0
            second_point_pose.pose.orientation.z = 0.0
            second_point_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(second_point_pose)
            self.final_goal_pub.publish(goal_pose)


            # publishing the midpoint of the contour which will be used to proved orientation to nav goal on topic /dir_point
            dir_pose = PoseStamped()
            dir_pose.header.frame_id = 'map'
            dir_pose.header.stamp = self.nav.get_clock().now().to_msg()
            dir_pose.pose.position.x = float(dir_point[0]) #float(bottom_point[0])
            dir_pose.pose.position.y = float(dir_point[1]) #float(bottom_point[1])
            dir_pose.pose.orientation.x = 0.0
            dir_pose.pose.orientation.y = 0.0
            dir_pose.pose.orientation.z = 0.0
            dir_pose.pose.orientation.w = 1.0

            bottom_point_pose = PoseStamped()
            bottom_point_pose.header.frame_id = 'map'
            bottom_point_pose.header.stamp = self.nav.get_clock().now().to_msg()
            bottom_point_pose.pose.position.x = float(bottompathpoint[0]) #float(bottom_point[0])
            bottom_point_pose.pose.position.y = float(bottompathpoint[1]) #float(bottom_point[1])
            bottom_point_pose.pose.orientation.x = 0.0
            bottom_point_pose.pose.orientation.y = 0.0
            bottom_point_pose.pose.orientation.z = 0.0
            bottom_point_pose.pose.orientation.w = 1.0
            self.dir_point_pub.publish(bottom_point_pose)
    
            binary_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
            imgmessage = self.bridge.cv2_to_imgmsg(binary_img, "rgb8")
            self.final_img_publisher.publish(imgmessage)
 
    # getting camera information
    def infocallback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
 
 
def main(args=None):
    rclpy.init(args=args)
 
    velpub = zedNODE()
    rclpy.spin(velpub)
    velpub.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()