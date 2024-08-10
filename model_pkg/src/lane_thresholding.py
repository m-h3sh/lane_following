from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
 
class zedNODE(Node):
 
    def __init__(self):
        super().__init__('pubsub')
        self.bridge = CvBridge()

        self.nav = BasicNavigator()
        
        #creating publishers and subscribers
        self.final_img_publisher = self.create_publisher(Image, '/model_lanes', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_point', 1)
        self.camerasub = self.create_subscription(Image, '/camera_forward/image_raw', self.camera_callack, 10)
        self.cinfosub = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.infocallback, 10)
        self.camerasub
        self.cinfosub
 
 
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
        maxcontour = contours[0]
        secondcontour = contours[0]
        for contour in contours:
            if len(contour) > len(maxcontour):
                maxcontour = contour
            if len(contour) < len(maxcontour) and len(contour) > len(secondcontour):
                secondcontour = contour
        
        # getting the bottom most point of the lane contour
        # checking if the lane bottom point is to the left or right of the center to determine left or right lane
        bottom_point = tuple(maxcontour[maxcontour[:, :, 1].argmax()][0])

        if (bottom_point[0] < (imgwidth/2)):
            print("left lane")
            second_point = tuple(maxcontour[maxcontour[:, :, 0].argmax()][0])
            goal_point = (second_point[0] + 40, second_point[1])
        else:
            print("right lane")
            second_point = tuple(maxcontour[maxcontour[:, :, 0].argmin()][0])
            goal_point = (second_point[0] - 40, second_point[1])
            cv2.circle(black_img, bottom_point, 7, (255, 255, 255), -1)
        cv2.circle(black_img, second_point, 7, (255, 255, 255), -1)
        cv2.circle(black_img, goal_point, 7, (0, 0, 255), -1)
        cv2.drawContours(black_img, maxcontour, -1, (0,255,0), 3)

        cv2.imshow("contours", black_img)
        cv2.waitKey(1)


        # publishing the goal pose as a PoseStamped on the topic /goal_point
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(goal_point[0])
        goal_pose.pose.position.y = float(goal_point[1])
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)
 
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