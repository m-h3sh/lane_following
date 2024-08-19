#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <math.h>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <unsupported/Eigen/FFT>
#include "visualization_msgs/msg/marker.hpp"
// #include "pcl/point_types.h"
// #include "pcl_conversions/pcl_conversions.h"

using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;
using std::placeholders::_1;
using namespace cv;

class LaneFollower : public rclcpp::Node
{
    public:
        LaneFollower() : Node("lane_follower")
        {   
            this->declare_parameter("bt_low",180);
            this->declare_parameter("bt_high",255);
            bt_low = this->get_parameter("bt_low").as_int();
            bt_high = this->get_parameter("bt_high").as_int();
            subscription_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_forward/camera_info", 10,
                std::bind(&LaneFollower::call, this, _1)
            );
            goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_point", 10,
                std::bind(&LaneFollower::goal_callback, this, _1)
            );
            dir_point_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/dir_point", 10,
                std::bind(&LaneFollower::dir_callback, this, _1)
            );
            final_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/final_goal_point", 10,
                std::bind(&LaneFollower::final_goal_callback, this, _1)
            );
            publisher_far = this->create_publisher<std_msgs::msg::Float32MultiArray>("/far_ipm", 10);
            dir_point_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/direction_point", 10);
            final_goal_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/final_goal_point_cpp", 10);
            publisher_near = this->create_publisher<sensor_msgs::msg::PointCloud2>("/near_ipm", 10);
            goal_viz = this->create_publisher<visualization_msgs::msg::Marker>("goal_viz", 10);
        }

    private:

        void call(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            this->camera_info = *msg;
            cam_info_received = true;
        }
        // void call_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //     this->odom = *msg;
        //     odom_received = true;

        // }



        std_msgs::msg::Float32MultiArray process_point(int y, int x) {
            // sensor_msgs::msg::PointCloud2 pub_pointcloud;
            // auto cloud_msg = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

            // Camera extrinsic parameters
            float roll = 0;
            float pitch = 0;
            float yaw = 0;
            float h = 0.8;

            // Pre-compute sin and cos values
            double cy = cos(yaw);
            double sy = sin(yaw);
            double cp = cos(pitch);
            double sp = sin(pitch);
            double cr = cos(roll);
            double sr = sin(roll);

            // Rotation matrix K (combining yaw, pitch, and roll)
            Eigen::Matrix3d K;
            K << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                -sp,     cp * sr,                cp * cr;

            // Normal vector to the ground plane (assuming flat ground)
            Eigen::Vector3d nor(0.0, 1.0, 0.0);

            // Calculate nc, the rotated normal vector
            Eigen::Vector3d nc = K * nor;

            // Inverse camera intrinsic matrix
            auto caminfo = this->camera_info.k; // assuming row-major order
            Eigen::Map<Matrix<double,3,3,RowMajor>> kin(caminfo.data());
            kin = kin.inverse().eval();

            // Convert the pixel coordinates (x, y) to homogeneous coordinates
            Eigen::Vector3d uv_hom(x, y, 1);

            // Map pixel coordinates to 3D camera ray
            Eigen::Vector3d kin_uv = kin * uv_hom;

            // Calculate the denominator for scaling (distance along the ray to the plane)
            double denom = kin_uv.dot(nc);
            // Ensure denom is not zero to avoid division by zero
            // if (denom != 0) {
            //     // Scale the ray by the height of the plane h
            //     pcl::PointXYZ point;
            //     point.x = h * kin_uv[2] / denom;
            //     point.y = -h * kin_uv[0] / denom;
            //     point.z = 0; // z-coordinate is zero on the ground plane

            //     // Output the point coordinates for debugging
            //     std::cout << "2D pixel: (" << x << ", " << y << ") -> 3D point: ("
            //             << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

            //     // Add the point to the cloud
            //     cloud_msg->points.push_back(point);
            // } else {
            //     std::cerr << "Denominator is zero, invalid projection for point (" << x << ", " << y << ")" << std::endl;
            // }
            std::vector<float> vec(3);
            vec[0] = h * kin_uv[2] / denom;
            vec[1] = -h * kin_uv[0] / denom;
            vec[2] = 0;

            // Prepare the PointCloud2 message for publishing
            // cloud_msg->height = 1;
            // cloud_msg->width = cloud_msg->points.size();
            // cloud_msg->is_dense = false;
            // pcl::toROSMsg(*cloud_msg, pub_pointcloud);
            // pub_pointcloud.header.frame_id = "base_link";
            // pub_pointcloud.header.stamp = rclcpp::Clock().now();

            // return pub_pointcloud;
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.pose.position.x = vec[0];
            marker.pose.position.z = vec[1];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            goal_viz->publish(marker);
            pub_array.set__data(vec);
            return pub_array;
        }        
        
        void send_goal(int y, int x, int y1, int x1, int y2, int x2) {
            std_msgs::msg::Float32MultiArray cloud_far = process_point(y, x);
            std_msgs::msg::Float32MultiArray cloud_dir = process_point(y1, x1);
            std_msgs::msg::Float32MultiArray goal_dir = process_point(y2, x2);
            dir_point_pub->publish(cloud_dir);
            publisher_far->publish(cloud_far);
            final_goal_pub->publish(goal_dir);
        }

        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            
            send_goal(int(msg->pose.position.y), int(msg->pose.position.x), int(dir_point.pose.position.y), int(dir_point.pose.position.x), int(final_goal_point.pose.position.y), int(final_goal_point.pose.position.x));
        }

        void dir_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            dir_point = *msg;
        }

        void final_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            final_goal_point = *msg;
        }
    
        // Start of Defining Variables ------------------------------------------------------------
        int largestClusterID=0, SecondLargestClusterID=0;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
        int bt_low, bt_high;

        sensor_msgs::msg::Image::SharedPtr dbMsg;
        sensor_msgs::msg::Image::SharedPtr threshMsg;

        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr vecto;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dir_point_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr final_goal_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

        geometry_msgs::msg::PoseStamped dir_point;
        geometry_msgs::msg::PoseStamped final_goal_point;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_far;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr final_goal_pub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dir_point_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_near;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_viz;

        sensor_msgs::msg::CameraInfo camera_info;
        size_t smoothing_window_size = 10;

        std_msgs::msg::Float32MultiArray pub_array;
        // nav_msgs::msg::Odometry odom;
        bool odom_received = false;
        
        int count =0;
        bool cam_info_received = false;

    // End of Defining Variables

};




int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto lane_follower = std::make_shared<LaneFollower>();
    
    // Example: process a single point (x, y)
    // ipm_node->process_point(100, 150); // Replace (100, 150) with desired point coordinates
    
    rclcpp::spin(lane_follower);
    rclcpp::shutdown();
    return 0;
}