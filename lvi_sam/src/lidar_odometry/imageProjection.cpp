#include "utility.hpp"
#include "lvi_sam/msg/cloud_info.hpp"

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

const int queueLength = 500;

class ImageProjection : public ParamServer
{
    private:

        std::mutex imuLock;
        std::mutex odoLock;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
        //ros::Publisher  pubLaserCloud;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
        rclcpp::Publisher<lvi_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        std::deque<sensor_msgs::msg::Imu> imuQueue;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        std::deque<nav_msgs::msg::Odometry> odomQueue;

        std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
        sensor_msgs::msg::PointCloud2 currentCloudMsg;

        double *imuTime = new double[queueLength];
        double *imuRotX = new double[queueLength];
        double *imuRotY = new double[queueLength];
        double *imuRotZ = new double[queueLength];

        int imuPointerCur;
        bool firstPointFlag;
        Eigen::Affine3f transStartInverse;

        pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
        pcl::PointCloud<PointType>::Ptr   fullCloud;
        pcl::PointCloud<PointType>::Ptr   extractedCloud;

        int deskewFlag;
        cv::Mat rangeMat;

        bool odomDeskewFlag;
        float odomIncreX;
        float odomIncreY;
        float odomIncreZ;

        lvi_sam::msg::CloudInfo cloudInfo;
        double timeScanCur;
        double timeScanNext;
        std_msgs::msg::Header cloudHeader;

    public:

        ImageProjection(const rclcpp::NodeOptions &options) : ParamServer("lvi_sam_imageProjection",options) , deskewFlag(0)
        {
            subImu   = create_subscription<sensor_msgs::msg::Imu>(imuTopic,2000,std::bind(&ImageProjection::imuHandler, this,std::placeholders::_1));
            subOdom  = create_subscription<nav_msgs::msg::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000,std::bind(&ImageProjection::odometryHandler, this,std::placeholders::_1));
            subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(pointCloudTopic, 5,std::bind(&ImageProjection::cloudHandler, this,std::placeholders::_1));

            pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
            pubLaserCloudInfo = create_publisher<lvi_sam::msg::CloudInfo>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5);

            allocateMemory();
            resetParameters();

            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
        }

        void allocateMemory()
        {
            laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
            fullCloud.reset(new pcl::PointCloud<PointType>());
            extractedCloud.reset(new pcl::PointCloud<PointType>());

            fullCloud->points.resize(N_SCAN*Horizon_SCAN);

            cloudInfo.start_ring_index.assign(N_SCAN, 0);
            cloudInfo.end_ring_index.assign(N_SCAN, 0);

            cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
            cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);

            resetParameters();
        }

        void resetParameters()
        {
            laserCloudIn->clear();
            extractedCloud->clear();
            // reset range matrix for range image projection
            rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

            imuPointerCur = 0;
            firstPointFlag = true;
            odomDeskewFlag = false;

            for (int i = 0; i < queueLength; ++i)
            {
                imuTime[i] = 0;
                imuRotX[i] = 0;
                imuRotY[i] = 0;
                imuRotZ[i] = 0;
            }
        }

         ~ImageProjection(){}

        void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
        {
            sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);
            std::lock_guard<std::mutex> lock1(imuLock);
            imuQueue.push_back(thisImu);
        }

        void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
        {
            std::lock_guard<std::mutex> lock2(odoLock);
            odomQueue.push_back(*odometryMsg);
        }

        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
        {
            if (!cachePointCloud(laserCloudMsg))
                return;

            if (!deskewInfo())
                return;

            projectPointCloud();

            cloudExtraction();

            publishClouds();

            resetParameters();
        }

        bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& laserCloudMsg)
        {
            // cache point cloud
            cloudQueue.push_back(*laserCloudMsg);

            if (cloudQueue.size() <= 2)
                return false;
            else
            {
                currentCloudMsg = cloudQueue.front();
                cloudQueue.pop_front();

                cloudHeader = currentCloudMsg.header;
                timeScanCur = stamp2Sec(cloudHeader.stamp);
                timeScanNext = stamp2Sec(cloudQueue.front().header.stamp);
            }

            // convert cloud
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

            // check dense flag
            if (laserCloudIn->is_dense == false)
            {
                RCLCPP_ERROR(rclcpp::get_logger("eclcpp"),"Point cloud is not in dense format, please remove NaN points first!");
                rclcpp::shutdown();
            }

            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("eclcpp"),"Point cloud ring channel not available, please configure your point cloud data!");
                    rclcpp::shutdown();
                }
            }     

            // check point time
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == timeField)
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    RCLCPP_WARN(rclcpp::get_logger("eclcpp"),"Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }

            return true;
        }

        bool deskewInfo()
        {
            std::lock_guard<std::mutex> lock1(imuLock);
            std::lock_guard<std::mutex> lock2(odoLock);

            // make sure IMU data available for the scan
            if (imuQueue.empty() || stamp2Sec(imuQueue.front().header.stamp) > timeScanCur || stamp2Sec(imuQueue.back().header.stamp) < timeScanNext)
            {
                RCLCPP_INFO(rclcpp::get_logger("eclcpp"),"Waiting for IMU data ...");
                return false;
            }

            imuDeskewInfo();

            odomDeskewInfo();

            return true;
        }

        void imuDeskewInfo()
        {
            cloudInfo.imu_available = false;

            while (!imuQueue.empty())
            {
                if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                    imuQueue.pop_front();
                else
                    break;
            }

            if (imuQueue.empty())
                return;

            imuPointerCur = 0;

            for (int i = 0; i < (int)imuQueue.size(); ++i)
            {
                sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
                double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);

                if (currentImuTime > timeScanNext + 0.01)
                    break;

                if (imuPointerCur == 0){
                    imuRotX[0] = 0;
                    imuRotY[0] = 0;
                    imuRotZ[0] = 0;
                    imuTime[0] = currentImuTime;
                    ++imuPointerCur;
                    continue;
                }

                // get angular velocity
                double angular_x, angular_y, angular_z;
                imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

                // integrate rotation
                double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
                imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
                imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
                imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
                imuTime[imuPointerCur] = currentImuTime;
                ++imuPointerCur;
            }

            --imuPointerCur;

            if (imuPointerCur <= 0)
                return;

            cloudInfo.imu_available = true;
        }

        void odomDeskewInfo()
        {
            cloudInfo.odom_available = false;

            while (!odomQueue.empty())
            {
                if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
                    odomQueue.pop_front();
                else
                    break;
            }

            if (odomQueue.empty())
                return;

            if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
                return;

            // get start odometry at the beinning of the scan
            nav_msgs::msg::Odometry startOdomMsg;

            for (int i = 0; i < (int)odomQueue.size(); ++i)
            {
                startOdomMsg = odomQueue[i];

                if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
                    continue;
                else
                    break;
            }

            tf2::Quaternion orientation;
            tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

            double roll, pitch, yaw;
            tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

            // Initial guess used in mapOptimization
            cloudInfo.odom_x = startOdomMsg.pose.pose.position.x;
            cloudInfo.odom_y = startOdomMsg.pose.pose.position.y;
            cloudInfo.odom_z = startOdomMsg.pose.pose.position.z;
            cloudInfo.odom_roll  = roll;
            cloudInfo.odom_pitch = pitch;
            cloudInfo.odom_yaw   = yaw;
            cloudInfo.odom_reset_id = (int)round(startOdomMsg.pose.covariance[0]);

            cloudInfo.odom_available = true;

            // get end odometry at the end of the scan
            odomDeskewFlag = false;

            if (stamp2Sec(odomQueue.back().header.stamp) < timeScanNext)
                return;

            nav_msgs::msg::Odometry endOdomMsg;

            for (int i = 0; i < (int)odomQueue.size(); ++i)
            {
                endOdomMsg = odomQueue[i];

                if (stamp2Sec(endOdomMsg.header.stamp) < timeScanNext)
                    continue;
                else
                    break;
            }

            if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
                return;

            Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

            tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
            tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

            Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

            float rollIncre, pitchIncre, yawIncre;
            pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

            odomDeskewFlag = true;
        }

        void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
        {
            *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

            int imuPointerFront = 0;
            while (imuPointerFront < imuPointerCur)
            {
                if (pointTime < imuTime[imuPointerFront])
                    break;
                ++imuPointerFront;
            }

            if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
            {
                *rotXCur = imuRotX[imuPointerFront];
                *rotYCur = imuRotY[imuPointerFront];
                *rotZCur = imuRotZ[imuPointerFront];
            } else {
                int imuPointerBack = imuPointerFront - 1;
                double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
                *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
                *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
            }
        }

        void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
        {
            *posXCur = 0; *posYCur = 0; *posZCur = 0;

            // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            //     return;

            // float ratio = relTime / (timeScanNext - timeScanCur);

            // *posXCur = ratio * odomIncreX;
            // *posYCur = ratio * odomIncreY;
            // *posZCur = ratio * odomIncreZ;
        }

        PointType deskewPoint(PointType *point, double relTime)
        {
            if (deskewFlag == -1 || cloudInfo.imu_available == false)
                return *point;

            double pointTime = timeScanCur + relTime;

            float rotXCur, rotYCur, rotZCur;
            findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

            float posXCur, posYCur, posZCur;
            findPosition(relTime, &posXCur, &posYCur, &posZCur);

            if (firstPointFlag == true)
            {
                transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
                firstPointFlag = false;
            }

            // transform points to start
            Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
            Eigen::Affine3f transBt = transStartInverse * transFinal;

            PointType newPoint;
            newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
            newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
            newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
            newPoint.intensity = point->intensity;

            return newPoint;
        }

        void projectPointCloud()
        {
            int cloudSize = (int)laserCloudIn->points.size();
            // range image projection
            for (int i = 0; i < cloudSize; ++i)
            {
                PointType thisPoint;
                thisPoint.x = laserCloudIn->points[i].x;
                thisPoint.y = laserCloudIn->points[i].y;
                thisPoint.z = laserCloudIn->points[i].z;
                thisPoint.intensity = laserCloudIn->points[i].intensity;

                int rowIdn = laserCloudIn->points[i].ring;
                if (rowIdn < 0 || rowIdn >= N_SCAN)
                    continue;

                if (rowIdn % downsampleRate != 0)
                    continue;

                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

                static float ang_res_x = 360.0/float(Horizon_SCAN);
                int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;

                if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                    continue;

                float range = pointDistance(thisPoint);
                
                if (range < 1.0)
                    continue;

                if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                    continue;

                // for the amsterdam dataset
                // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
                //     continue;
                // if (thisPoint.z < -2.0)
                //     continue;

                rangeMat.at<float>(rowIdn, columnIdn) = range;

                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
                // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

                int index = columnIdn  + rowIdn * Horizon_SCAN;
                fullCloud->points[index] = thisPoint;
            }
        }

        void cloudExtraction()
        {
            int count = 0;
            // extract segmented cloud for lidar odometry
            for (int i = 0; i < N_SCAN; ++i)
            {
                cloudInfo.start_ring_index[i] = count - 1 + 5;

                for (int j = 0; j < Horizon_SCAN; ++j)
                {
                    if (rangeMat.at<float>(i,j) != FLT_MAX)
                    {
                        // mark the points' column index for marking occlusion later
                        cloudInfo.point_col_ind[count] = j;
                        // save range info
                        cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                        // save extracted cloud
                        extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        // size of extracted cloud
                        ++count;
                    }
                }
                cloudInfo.end_ring_index[i] = count -1 - 5;
            }
        }
        
        void publishClouds()
        {
            cloudInfo.header = cloudHeader;
            cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
            pubLaserCloudInfo->publish(cloudInfo);
        }

};

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"\033[1;32m----> Lidar Cloud Deskew Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();

    return 0;
}