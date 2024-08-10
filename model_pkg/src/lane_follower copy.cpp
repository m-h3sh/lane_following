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
            publisher_far = this->create_publisher<std_msgs::msg::Float32MultiArray>("/far_ipm", 10);
            publisher_near = this->create_publisher<sensor_msgs::msg::PointCloud2>("/near_ipm", 10);
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
            // Process the single point

            float roll = 0;
            float pitch = 0;
            float yaw = 0;
            float h = 0.8;

            vector<double> k(9);

            double cy, cr, sy, sr, sp, cp;
            cy = cos(yaw);
            sy = sin(yaw);
            cp = cos(pitch);
            sp = sin(pitch);
            cr = cos(roll);
            sr = sin(roll);
            
            k[0] = cr * cy + sp * sr + sy;
            k[1] = cr * sp * sy - cy * sr;
            k[2] = -cp * sy;
            k[3] = cp * sr;
            k[4] = cp * cr;
            k[5] = sp;
            k[6] = cr * sy - cy * sp * sr;
            k[7] = -cr * cy * sp - sr * sy;
            k[8] = cp * cy;

            Eigen::Matrix3d K;
            K << k[0], k[1], k[2],
                k[3], k[4], k[5],
                k[6], k[7], k[8];
            
            Eigen::Vector3d nor(0.0, 1.0, 0.0);
            
                
            
            // Calculate UV
            Eigen::Vector3d nc = K*nor;


            // Inverse camera matrix
            
            auto caminfo = this->camera_info.k;
            
            Eigen::Map<Eigen::Matrix3d> kin(caminfo.data());
            kin = kin.inverse().eval();
            

        
            Eigen::Vector3d uv_hom = { x, y, 1 };


            
            Eigen::Vector3d kin_uv = kin*uv_hom;
            double denom = kin_uv.dot(nc);


            std::vector<float> vec(3);
            vec[0] = h * kin_uv[2] / denom;
            vec[1] = -h * kin_uv[0] / denom;
            vec[2] = 0;
            //----------------------------------------------------------
            // cloud_msg->points.push_back(vec);

            // Prepare the PointCloud message for publishing
            // cloud_msg->height = 1;
            // cloud_msg->width = cloud_msg->points.size();
            // cloud_msg->is_dense = false;
            // pcl::toROSMsg(*cloud_msg, pub_pointcloud);
            // pub_pointcloud.header.frame_id = "base_link";
            // pub_pointcloud.header.stamp = rclcpp::Clock().now();


            // // Return the cloud
            // for (int i =0 ; i< pub_pointcloud.data.size() ; i++) {
            //     cout<<pub_pointcloud.data[i]<<" ";
            // }
            // cout<<endl;
            // return pub_pointcloud;
            //--------------------------------------------------------
            pub_array.set__data(vec);
            return pub_array;
            
        }


        void send_goal(int y, int x) {
            std_msgs::msg::Float32MultiArray cloud_far = process_point(y, x);
            publisher_far->publish(cloud_far);
        }

        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            send_goal(int(msg->pose.position.y), int(msg->pose.position.x));
        }
    
        // Start of Defining Variables ------------------------------------------------------------
        int largestClusterID=0, SecondLargestClusterID=0;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
        int bt_low, bt_high;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2;
        // // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l_publisher;
        // // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr r_publisher;
        // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;

        // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ma_publisher;
        // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ma2_publisher;


        sensor_msgs::msg::Image::SharedPtr dbMsg;
        sensor_msgs::msg::Image::SharedPtr threshMsg;

        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr vecto;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_far;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_near;

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