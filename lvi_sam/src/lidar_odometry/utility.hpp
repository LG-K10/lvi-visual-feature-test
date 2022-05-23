#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp> //#include <ros/ros.h>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer : public rclcpp::Node
{
    public:

        std::string PROJECT_NAME;

        std::string robot_id;

        string pointCloudTopic;
        string imuTopic;
        string odomTopic;
        string gpsTopic;

        // GPS Settings
        bool useImuHeadingInitialization;
        bool useGpsElevation;
        float gpsCovThreshold;
        float poseCovThreshold;

        // Save pcd
        bool savePCD;
        string savePCDDirectory;

        // Velodyne Sensor Configuration: Velodyne
        int N_SCAN;
        int Horizon_SCAN;
        string timeField;
        int downsampleRate;

        // IMU
        float imuAccNoise;
        float imuGyrNoise;
        float imuAccBiasN;
        float imuGyrBiasN;
        float imuGravity;
        vector<double> extRotV;
        vector<double> extRPYV;
        vector<double> extTransV;
        Eigen::Matrix3d extRot;
        Eigen::Matrix3d extRPY;
        Eigen::Vector3d extTrans;
        Eigen::Quaterniond extQRPY;

        // LOAM
        float edgeThreshold;
        float surfThreshold;
        int edgeFeatureMinValidNum;
        int surfFeatureMinValidNum;

        // voxel filter paprams
        float odometrySurfLeafSize;
        float mappingCornerLeafSize;
        float mappingSurfLeafSize ;

        float z_tollerance; 
        float rotation_tollerance;

        // CPU Params
        int numberOfCores;
        double mappingProcessInterval;

        // Surrounding map
        float surroundingkeyframeAddingDistThreshold; 
        float surroundingkeyframeAddingAngleThreshold; 
        float surroundingKeyframeDensity;
        float surroundingKeyframeSearchRadius;
        
        // Loop closure
        bool loopClosureEnableFlag;
        int   surroundingKeyframeSize;
        float historyKeyframeSearchRadius;
        float historyKeyframeSearchTimeDiff;
        int   historyKeyframeSearchNum;
        float historyKeyframeFitnessScore;

        // global map visualization radius
        float globalMapVisualizationSearchRadius;
        float globalMapVisualizationPoseDensity;
        float globalMapVisualizationLeafSize;

        ParamServer(std::string node_name , const rclcpp::NodeOptions & options) : Node(node_name , options)
        {
            declare_parameter("/PROJECT_NAME","lvi_sam");
            get_parameter("/PROJECT_NAME",PROJECT_NAME);

            declare_parameter("/robot_id","robot");
            get_parameter("/robot_id",robot_id);

            declare_parameter(PROJECT_NAME + "/pointCloudTopic","points_raw");
            get_parameter(PROJECT_NAME + "/pointCloudTopic",pointCloudTopic);
            declare_parameter(PROJECT_NAME + "/imuTopic","imu_correct");
            get_parameter(PROJECT_NAME + "/imuTopic", imuTopic);
            declare_parameter(PROJECT_NAME + "/odomTopic","odometry/imu");
            get_parameter(PROJECT_NAME + "/odomTopic", odomTopic);
            declare_parameter(PROJECT_NAME + "/gpsTopic","odometry/gps");
            get_parameter(PROJECT_NAME + "/gpsTopic",gpsTopic);

            declare_parameter(PROJECT_NAME + "/useImuHeadingInitialization", false);
            get_parameter(PROJECT_NAME + "/useImuHeadingInitialization", useImuHeadingInitialization);
            declare_parameter(PROJECT_NAME + "/useGpsElevation",false);
            get_parameter(PROJECT_NAME + "/useGpsElevation", useGpsElevation);
            declare_parameter(PROJECT_NAME + "/gpsCovThreshold", 2.0);
            get_parameter(PROJECT_NAME + "/gpsCovThreshold", gpsCovThreshold );
            declare_parameter(PROJECT_NAME + "/poseCovThreshold", 25.0);
            get_parameter(PROJECT_NAME + "/poseCovThreshold", poseCovThreshold );

            declare_parameter(PROJECT_NAME + "/savePCD", false);
            get_parameter(PROJECT_NAME + "/savePCD", savePCD );
            declare_parameter(PROJECT_NAME + "/savePCDDirectory", "/tmp/loam/");
            get_parameter(PROJECT_NAME + "/savePCDDirectory", savePCDDirectory);

            declare_parameter(PROJECT_NAME + "/N_SCAN", 16);
            get_parameter(PROJECT_NAME + "/N_SCAN", N_SCAN);
            declare_parameter(PROJECT_NAME + "/Horizon_SCAN", 1800);
            get_parameter(PROJECT_NAME + "/Horizon_SCAN", Horizon_SCAN );
            declare_parameter(PROJECT_NAME + "/timeField","time");
            get_parameter(PROJECT_NAME + "/timeField", timeField );
            declare_parameter(PROJECT_NAME + "/downsampleRate", 1);
            get_parameter(PROJECT_NAME + "/downsampleRate", downsampleRate );

            declare_parameter(PROJECT_NAME + "/imuAccNoise", 0.01);
            get_parameter(PROJECT_NAME + "/imuAccNoise", imuAccNoise);
            declare_parameter(PROJECT_NAME + "/imuGyrNoise", 0.001);
            get_parameter(PROJECT_NAME + "/imuGyrNoise", imuGyrNoise);
            declare_parameter(PROJECT_NAME + "/imuAccBiasN", 0.0002);
            get_parameter(PROJECT_NAME + "/imuAccBiasN", imuAccBiasN);
            declare_parameter(PROJECT_NAME + "/imuGyrBiasN", 0.00003);
            get_parameter(PROJECT_NAME + "/imuGyrBiasN", imuGyrBiasN);
            declare_parameter(PROJECT_NAME + "/imuGravity", 9.80511);
            get_parameter(PROJECT_NAME + "/imuGravity", imuGravity);
            declare_parameter(PROJECT_NAME+ "/extrinsicRot", vector<double>());
            get_parameter(PROJECT_NAME+ "/extrinsicRot", extRotV);
            declare_parameter(PROJECT_NAME+ "/extrinsicRPY", vector<double>());
            get_parameter(PROJECT_NAME+ "/extrinsicRPY", extRPYV);
            declare_parameter(PROJECT_NAME+ "/extrinsicTrans", vector<double>());
            get_parameter(PROJECT_NAME+ "/extrinsicTrans", extTransV);
            extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
            extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
            extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
            extQRPY = Eigen::Quaterniond(extRPY);

            declare_parameter(PROJECT_NAME + "/edgeThreshold", 0.1);
            get_parameter(PROJECT_NAME + "/edgeThreshold", edgeThreshold);
            declare_parameter(PROJECT_NAME + "/surfThreshold", 0.1);
            get_parameter(PROJECT_NAME + "/surfThreshold", surfThreshold);
            declare_parameter(PROJECT_NAME + "/edgeFeatureMinValidNum", 10);
            get_parameter(PROJECT_NAME + "/edgeFeatureMinValidNum", edgeFeatureMinValidNum);
            declare_parameter(PROJECT_NAME + "/surfFeatureMinValidNum", 100);
            get_parameter(PROJECT_NAME + "/surfFeatureMinValidNum", surfFeatureMinValidNum);

            declare_parameter(PROJECT_NAME + "/odometrySurfLeafSize", 0.2);
            get_parameter(PROJECT_NAME + "/odometrySurfLeafSize", odometrySurfLeafSize);
            declare_parameter(PROJECT_NAME + "/mappingCornerLeafSize", 0.2);
            get_parameter(PROJECT_NAME + "/mappingCornerLeafSize", mappingCornerLeafSize);
            declare_parameter(PROJECT_NAME + "/mappingSurfLeafSize", 0.2);
            get_parameter(PROJECT_NAME + "/mappingSurfLeafSize", mappingSurfLeafSize);

            declare_parameter(PROJECT_NAME + "/z_tollerance", FLT_MAX);
            get_parameter(PROJECT_NAME + "/z_tollerance", z_tollerance);
            declare_parameter(PROJECT_NAME + "/rotation_tollerance", FLT_MAX);
            get_parameter(PROJECT_NAME + "/rotation_tollerance", rotation_tollerance);

            declare_parameter(PROJECT_NAME + "/numberOfCores", 2);
            get_parameter(PROJECT_NAME + "/numberOfCores", numberOfCores);
            declare_parameter(PROJECT_NAME + "/mappingProcessInterval", 0.15);
            get_parameter(PROJECT_NAME + "/mappingProcessInterval", mappingProcessInterval);

            declare_parameter(PROJECT_NAME + "/surroundingkeyframeAddingDistThreshold", 1.0);
            get_parameter(PROJECT_NAME + "/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
            declare_parameter(PROJECT_NAME + "/surroundingkeyframeAddingAngleThreshold", 0.2);
            get_parameter(PROJECT_NAME + "/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
            declare_parameter(PROJECT_NAME + "/surroundingKeyframeDensity", 1.0);
            get_parameter(PROJECT_NAME + "/surroundingKeyframeDensity", surroundingKeyframeDensity);
            declare_parameter(PROJECT_NAME + "/surroundingKeyframeSearchRadius", 50.0);
            get_parameter(PROJECT_NAME + "/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

            declare_parameter(PROJECT_NAME + "/loopClosureEnableFlag", false);
            get_parameter(PROJECT_NAME + "/loopClosureEnableFlag", loopClosureEnableFlag);
            declare_parameter(PROJECT_NAME + "/surroundingKeyframeSize", 50);
            get_parameter(PROJECT_NAME + "/surroundingKeyframeSize", surroundingKeyframeSize);
            declare_parameter(PROJECT_NAME + "/historyKeyframeSearchRadius", 10.0);
            get_parameter(PROJECT_NAME + "/historyKeyframeSearchRadius", historyKeyframeSearchRadius);
            declare_parameter(PROJECT_NAME + "/historyKeyframeSearchTimeDiff",30.0);
            get_parameter(PROJECT_NAME + "/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
            declare_parameter(PROJECT_NAME + "/historyKeyframeSearchNum", 25);
            get_parameter(PROJECT_NAME + "/historyKeyframeSearchNum", historyKeyframeSearchNum);
            declare_parameter(PROJECT_NAME + "/historyKeyframeFitnessScore",0.3);
            get_parameter(PROJECT_NAME + "/historyKeyframeFitnessScore", historyKeyframeFitnessScore);

            declare_parameter(PROJECT_NAME + "/globalMapVisualizationSearchRadius", 1e3);
            get_parameter(PROJECT_NAME + "/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
            declare_parameter(PROJECT_NAME + "/globalMapVisualizationPoseDensity", 10.0);
            get_parameter(PROJECT_NAME + "/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
            declare_parameter(PROJECT_NAME + "/globalMapVisualizationLeafSize", 1.0);
            get_parameter(PROJECT_NAME + "/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

            usleep(100);
        }

        sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
        {
            sensor_msgs::msg::Imu imu_out = imu_in;
            // rotate acceleration
            Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
            acc = extRot * acc;
            imu_out.linear_acceleration.x = acc.x();
            imu_out.linear_acceleration.y = acc.y();
            imu_out.linear_acceleration.z = acc.z();
            // rotate gyroscope
            Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
            gyr = extRot * gyr;
            imu_out.angular_velocity.x = gyr.x();
            imu_out.angular_velocity.y = gyr.y();
            imu_out.angular_velocity.z = gyr.z();
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
            {
                RCLCPP_ERROR(get_logger(),"Invalid quaternion, please use a 9-axis IMU!");
                rclcpp::shutdown();
            }

            return imu_out;
        }
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, T thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double stamp2Sec(const T& stamp)
{
    return rclcpp::Time(stamp).seconds();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif