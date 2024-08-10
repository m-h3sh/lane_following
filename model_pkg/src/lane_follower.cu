#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
// #include <mlpack/methods/dbscan/dbscan.hpp>
#include <mlpack/core.hpp>
#include "mlpack.hpp"
#include <armadillo>
#include <vector>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
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
// #include "nav_msgs/msg/odometry.hpp"
#include "dbscan_cuda.cuh"
#include "std_msgs/msg/float32_multi_array.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_util/lifecycle_service_client.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "nav2_util/simple_action_server.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



using namespace std;
using namespace std::chrono_literals;
using namespace std;
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
        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed/masked_image", 10, std::bind(&LaneFollower::binary_thresholding, this, std::placeholders::_1));
        
        bt_low = this->get_parameter("bt_low").as_int();
        bt_high = this->get_parameter("bt_high").as_int();
        subscription_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed/zed_node/rgb/camera_info", 10,
            std::bind(&LaneFollower::call, this, _1));
        publisher_far = this->create_publisher<std_msgs::msg::Float32MultiArray>("/far_ipm", 10);
        publisher_near = this->create_publisher<sensor_msgs::msg::PointCloud2>("/near_ipm", 10);

        db_publisher = this->create_publisher<sensor_msgs::msg::Image>("/dbImage", 10);
        thresh_publisher = this->create_publisher<sensor_msgs::msg::Image>("/threshImage", 10);
    }

private:

    void binary_thresholding(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
        ////cout<<" RECEIVED IMAGE"<<endl; 
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cv_image = cv_ptr->image, gray_image, thresholded_image;
        int rows = cv_image.rows;
        int cols = cv_image.cols;
        // cv_image = cv_image(cv::Range(rows/2,rows),cv::Range::all());
        cv::medianBlur(cv_image,cv_image,5);
        cv::cvtColor(cv_image, gray_image,CV_BGR2GRAY); 
        // cv::imshow("window", cv_image);
        // cv::waitKey(0);
        for (int y=0 ; y< rows/4 ; y++) {
            for (int x=0 ; x< cols ; x++) {
                gray_image.at<uchar>(y, x) = 0;
            }
        }
        ////cout<<"BINARY THRESHOLDING STARTED"<<endl;
        // cv::inRange(cv_image, Scalar(110, 0, 0), Scalar(145, 255, 255), thresholded_image);
        cv::inRange(gray_image, bt_low, bt_high, thresholded_image);
        threshMsg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",thresholded_image).toImageMsg();
        thresh_publisher->publish(*threshMsg);
        ////cout<<"BINARY THRESHOLDING DONE"<<endl;

        
        std::vector<cv::Point> white_pixel_indices;

        // cv::findNonZero(thresholded_image, white_pixel_indices);
        

        // ////cout<<"COLLECTING WHITE POINTS ENDED"<<endl;

        // arma::mat data(2, white_pixel_indices.size());
        
        // for (size_t i = 0; i < white_pixel_indices.size(); ++i) {
        //     data(0, i) = white_pixel_indices[i].x;
        //     data(1, i) = white_pixel_indices[i].y;
        // }

        // data.resize(2, (white_pixel_indices.size()+1)/2);

        // ////cout<<white_pixel_indices.size()<<endl;
        // ////cout<<"SETTING PARAMETERS FOR DB SCAN"<<endl;
        // // Parameters for DBSCAN
        // double epsilon = 5.0; // Adjust as needed
        // size_t minPoints = 10; // Adjust as needed
        int eps = 10;
        // //cout << "nigga2" << endl;
        
        // cv::Canny(thresholded_image,thresholded_image,50,75, 3,true);

        // Mat dilated;
        // float dilation_size = 0.5;  // Adjust the dilation size as needed
        // Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
        // dilate(thresholded_image, thresholded_image, element);
        
        
        
        
        // cv::imshow("window",thresholded_image);
        // cv::waitKey(10);

        Graph graph(thresholded_image, eps);
        DBSCAN scanner(&graph, 0.1f, 10);
        scanner.identify_cluster();

        // //cout<<"nigga"<<endl;
        arma::Row<size_t> assignments(graph.size());

        for (int i =0 ; i < graph.size() ; i++) {
            // //cout<<"nigga3"<<endl;
            assignments[i] = scanner.label(i);
            // //cout << assignments[i]<<" ";
        }
        // //cout<<endl;F

        for (int i =0 ; i<graph.size() ; i++) {
            cv::Point point;
            point.x = graph.node(i).y;
            point.y = graph.node(i).x;
            
            white_pixel_indices.push_back(point);
            
        }

        // ////cout<<"DBSCAN STARTING"<<endl;
        // mlpack::dbscan::DBSCAN<> dbscan(epsilon, minPoints);
        // dbscan.Cluster(data, assignments);
        // ////cout<<"DBSCAN ENDED"<<endl;
        std::unordered_map<size_t, size_t> clusterSizes;

  // Count points in each cluster
        ////cout <<"Count points in each cluster" <<endl;
        
        for (size_t i = 0; i < assignments.n_elem; ++i) {
            if (assignments[i] != SIZE_MAX) { // Ignore noise points
            clusterSizes[assignments[i]]++;
            }
        }

        ////cout<<"Create a vector of pairs (clusterID, size) and sort it"<<endl;
        std::vector<std::pair<size_t, size_t>> sortedClusters(clusterSizes.begin(), clusterSizes.end());
        
        std::sort(sortedClusters.begin(), sortedClusters.end(), [](const std::pair<size_t, size_t>& a, const std::pair<size_t, size_t>& b) {
            return b.second < a.second; // Sort in descending order of size
        });
        
        

        if (sortedClusters.size() < 2) {
            // RCLCPP_WARN(this->get_logger(), "Not enough clusters found");
            return;
        }


        if (!sortedClusters.empty()) {
            if (sortedClusters[0].second == 0) {
                ////cout<<"No Clusters Found" <<endl;
                return;
            }
            largestClusterID = sortedClusters[0].first;
            SecondLargestClusterID = sortedClusters[1].first;


        }
        else {
            ////cout<<"No sorted clusters"<<endl;
            return;
        }
        
        vector<cv::Point> largestCluster, secondLargestCluster;
        ////cout<<"test"<<endl;
        for (size_t i = 0; i<assignments.n_elem ; ++i) {
            if (assignments[i] == largestClusterID) largestCluster.push_back(white_pixel_indices[i]);
            else if (assignments[i] == SecondLargestClusterID) secondLargestCluster.push_back(white_pixel_indices[i]);
        }
        
        
        
        
        cv::Mat dbImage = cv::Mat::zeros(gray_image.rows, gray_image.cols,CV_8UC1);
        cv::Mat dbImage2 = cv::Mat::zeros(gray_image.rows, gray_image.cols,CV_8UC1);

        

        for (cv::Point point : largestCluster) {
            dbImage.at<uchar>(point.y,point.x) = 255;
            dbImage2.at<uchar>(point.y,point.x) = 255;
            
        }

        for (cv::Point point : secondLargestCluster) {
            dbImage2.at<uchar>(point.y,point.x) = 255;
        }

       
        dbMsg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",dbImage2).toImageMsg();
        db_publisher->publish(*dbMsg);
       
        midpoint_publisher(largestCluster,secondLargestCluster,dbImage2);
    

    }
    


    void midpoint_publisher(vector<cv::Point> largestCluster, vector<cv::Point> secondLargestCluster, cv::Mat lanes_binary) {

        geometry_msgs::msg::Point left,right,mid_farthest, mid_closest;
        int y__cord, x1__cord, x2__cord;

        bool midpoint_flag_farthest = false;
        bool midpoint_flag_closest = false;

        int current_y=0, prev_y=0;

        

        // Finding farthest point
        for (int j = 0 ; j<secondLargestCluster.size() ; j++) {
            current_y = secondLargestCluster[j].y;
            if (current_y != 0 && prev_y !=0 && current_y == prev_y) continue;  
            for (int i =0 ; i<largestCluster.size(); ++i) {
                if (largestCluster[i].y > secondLargestCluster[j].y) break;
                if ((largestCluster[i].y <= secondLargestCluster[j].y + 5)&&(largestCluster[i].y >= secondLargestCluster[j].y - 5)) {
                        if (!largestCluster[i].x or !secondLargestCluster[j].x)
                        mid_farthest.y = largestCluster[i].y;
                        mid_farthest.x = (largestCluster[i].x + secondLargestCluster[j].x)/2;
                        midpoint_flag_farthest = true;
                        break;
                } 
            }
            if (midpoint_flag_farthest) break;
            prev_y = current_y;
        }

        // Finding Closest point

        current_y = 0;
        prev_y = 0;

        // std::sort(secondLargestCluster.end()-50, secondLargestCluster.end(), 
        //       [](const cv::Point& a, const cv::Point& b) {
        //           return a.y < b.y;
        //       });
     
        
        for (int j =secondLargestCluster.size() ; j>0 ; j--) {

            // //cout <<secondLargestCluster[j] <<" "<< largestCluster[j]<<endl;

            current_y = secondLargestCluster[j].y;
            if (current_y != 0 && prev_y !=0 && current_y == prev_y) continue;  
            if ((current_y - mid_farthest.y)<10) continue;
            if (current_y ==0 ) continue;
            for (int i =largestCluster.size() ; i>0 ; i--) {
                if ((largestCluster[i].y <= secondLargestCluster[j].y + 5)&&(largestCluster[i].y >= secondLargestCluster[j].y - 5)) {

                        
                        y__cord = largestCluster[i].y;
                        x1__cord = largestCluster[i].x;
                        x2__cord = secondLargestCluster[j].x;
                      
                        // //cout<<" Y_coord "<<y__cord<<" "<<j<<endl;
                        
                        if (abs(x1__cord - x2__cord)>50 && y__cord != 0) {
                            mid_closest.y = y__cord;
                            mid_closest.x = (x1__cord + x2__cord)/2;
                            midpoint_flag_closest = true;
                            break;
                        }
                } 
            }
            if (midpoint_flag_closest) break;
            prev_y = current_y;
        }

        // vector<geometry_msgs::msg::Point> mid_far_vect(10);
        // count++;
        
        
        ////cout<<midpoint_flag_farthest<<endl;
        
        if (midpoint_flag_farthest) {
            // Add the new midpoint to the deque
            smoothed_midpoints_far.push_back(mid_farthest);
            if (smoothed_midpoints_far.size() > smoothing_window_size) {
                smoothed_midpoints_far.pop_front();
            }

            // Calculate the average midpoint
            geometry_msgs::msg::Point avg_farthest = calculate_average_point(smoothed_midpoints_far);
            lanes_binary.at<uchar>(avg_farthest.y, avg_farthest.x) = 255;
            if (cam_info_received) {
                std_msgs::msg::Float32MultiArray cloud_far = process_point(avg_farthest.y, avg_farthest.x);
                cam_info_received = false;
                publisher_far->publish(cloud_far);
            }
            
        }

        if (midpoint_flag_closest) {
            // Add the new midpoint to the deque
            smoothed_midpoints_near.push_back(mid_closest);
            if (smoothed_midpoints_near.size() > smoothing_window_size) {
                smoothed_midpoints_near.pop_front();
            }

            // Calculate the average midpoint
            geometry_msgs::msg::Point avg_closest = calculate_average_point(smoothed_midpoints_near);
            lanes_binary.at<uchar>(avg_closest.y, avg_closest.x) = 255;
            if (cam_info_received) {
                // cout<<avg_closest.y<<" "<<avg_closest.x<<endl;
                // sensor_msgs::msg::PointCloud2 cloud_near = process_point(avg_closest.y, avg_closest.x);
                cam_info_received = false;
                // publisher_near->publish(cloud_near);
            }
            
            
        }

        
        

        // cv::imshow("window",lanes_binary);
        // cv::waitKey(30);
        // process_point(y__cord,(x1__cord+x2__cord)/2);


        
        
        // publish_vect(mid_farthest,mid_closest);




        
        // cv:imshow("window",lanes_binary);
        // cv::waitKey(30);

        


    }
    

    void call(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->camera_info = *msg;
        cam_info_received = true;
    }
    // void call_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //     this->odom = *msg;
    //     odom_received = true;

    // }


 std_msgs::msg::Float32MultiArray process_point(int y, int x) {
    
    sensor_msgs::msg::PointCloud2 pub_pointcloud;
    auto cloud_msg = std::make_unique<PointCloud>();

    // Process the single point

    float roll = 0;
    float pitch = -24 * M_PI / 180;
    float yaw = 0;
    float h = 1.41;

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
    kin = kin.inverse();
    

  
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
    geometry_msgs::msg::Point calculate_average_point(const std::deque<geometry_msgs::msg::Point>& points) {
        geometry_msgs::msg::Point avg_point;
        avg_point.x = 0;
        avg_point.y = 0;
        avg_point.z = 0;
        for (const auto& point : points) {
            avg_point.x += point.x;
            avg_point.y += point.y;
            avg_point.z += point.z;
        }
        avg_point.x /= points.size();
        avg_point.y /= points.size();
        avg_point.z /= points.size();
        return avg_point;
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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr db_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thresh_publisher;

    sensor_msgs::msg::Image::SharedPtr dbMsg;
    sensor_msgs::msg::Image::SharedPtr threshMsg;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr vecto;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_far;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_near;

    sensor_msgs::msg::CameraInfo camera_info;
    size_t smoothing_window_size = 10;
    std::deque<geometry_msgs::msg::Point> smoothed_midpoints_far;
    std::deque<geometry_msgs::msg::Point> smoothed_midpoints_near;

    std_msgs::msg::Float32MultiArray pub_array;
    // nav_msgs::msg::Odometry odom;
    bool odom_received = false;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
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

