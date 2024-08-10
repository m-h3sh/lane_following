from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import torch
import cv2
import ssl
import torchvision.transforms as transforms
import numpy as np
from scipy.spatial import ConvexHull
from cv_bridge import CvBridge
ssl._create_default_https_context = ssl._create_unverified_context
 
 
class zedNODE(Node):
 
    def __init__(self):
        super().__init__('pubsub')
        self.bridge = CvBridge()
        self.model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True)
 
        #creating publishers and subscribers
        self.final_img_publisher = self.create_publisher(Image, '/model_lanes', 1)
        self.camerasub = self.create_subscription(Image, '/camera_forward/image_raw', self.camera_callack, 10)
        self.cinfosub = self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.infocallback, 10)
        self.camerasub
        self.cinfosub
 
 
    def camera_callack(self, data):
        self.cvimage = self .bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image
        cv2.imshow("original image", self.cvimage)
        cv2.waitKey(1)
 
        self.cvimage = cv2.resize(self.cvimage, (1280, 720), cv2.INTER_LINEAR)
        tensor_transform = transforms.ToTensor()
        self.cvimage = tensor_transform(self.cvimage)
        self.cvimage = torch.reshape(self.cvimage, (1, 3, 720, 1280))
        resize_transform = transforms.Resize((384, 640))
        self.cvimage = resize_transform(self.cvimage)
 
        # getting the output from the model
        det_out, da_out, ll_out = self.model(self.cvimage.float())
        pad_h, pad_w, height, width = 12, 0, 384, 640
        ll_predict = ll_out[:, :, pad_h:(height-pad_h), pad_w:(width-pad_w)]
        ll_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(3), mode='bilinear')
        _, ll_mask = torch.max(ll_mask, 1)
        ll_mask = ll_mask.int().squeeze().cpu().numpy()
 
        mask_img = np.zeros((1080, 1920, 3), dtype=np.uint8)
        mask_img[ll_mask == 1] = [255, 255, 255]
        gray_img = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
        #print(gray_img.shape)
 
        gray_img = cv2.blur(gray_img, (11, 11))
        #cv2.imshow("after filtering", gray_img)
 
        contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        black_img = np.zeros((1080, 1920, 3), dtype=np.uint8)
        maxcontour = contours[0]
        secondcontour = contours[0]
        for i,contour in enumerate(contours):
            if len(contour) > len(maxcontour):
                maxcontour = contour
            if len(contour) < len(maxcontour) and len(contour) > len(secondcontour):
                secondcontour = contour
        cv2.drawContours(black_img, maxcontour, -1, (0,255,0), 3)
        print(maxcontour.shape)
        #M1 = contour
        # print(maxcontour[maxcontour[:, :, 1][0]])
        cv2.imshow("after contour", black_img)
        cv2.waitKey(1)
 
 
        imgmessage = self.bridge.cv2_to_imgmsg(black_img, "rgb8")
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