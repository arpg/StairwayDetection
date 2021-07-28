// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <stairs/preanalysis.h>
#include <stairs/regions.h>
#include <stairs/regiongrowing.h>
#include <stairs/voxSAC.h>
#include <stairs/splitmerge.h>
#include <stairs/planeshape.h>
#include <stairs/recognition.h>
#include <stairs/StairVector.h>
// #include <stairs/prediction.h>

#include <stairs/ros_functions.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>
// #include <colormap/palettes.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

class StairDetection 
{
private:
    boost::mutex mutex_;

    ros::NodeHandle m_nh, m_private_nh;
    ros::Time stamp_;
    std::string frame_;
    tf::TransformListener listener;
    ros::Subscriber input_sub;

    std::string config_filepath_;
    YAML::Node config_;
    std::string fixed_frame_id_;
    ros::Publisher main_cloud_pub_;
    ros::Publisher normal_cloud_pub_;
    ros::Publisher seg_regions_pub_;
    ros::Publisher rise_regions_pub_;
    ros::Publisher tread_regions_pub_;
    ros::Publisher tread_region_nums_pub_;
    ros::Publisher rise_cloud_pub_;
    ros::Publisher tread_cloud_pub_;
    ros::Publisher rail_cloud_pub_;
    ros::Publisher whole_cloud_pub_;
    ros::Publisher is_stair_cloud_pub_;
    ros::Publisher seg_stairs_pub_;
    ros::Publisher stair_parts_pub_;
    ros::Publisher nearest_step_pose_pub_;

public:

    StairDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_nh(nh),
        m_private_nh(pnh)//,
        // busy(false)
    {
        // config_ = YAML::LoadFile(config_filepath_);
        // params_.fixed_frame_id = config_["ros"]["fixed_frame_id"].as<std::string>();
        
        input_sub = m_private_nh.subscribe("input_cloud",1,&StairDetection::inputCB,this);

        m_private_nh.param("config_filepath", config_filepath_, std::string(""));
        config_ = YAML::LoadFile(config_filepath_);
        fixed_frame_id_ = config_["ros"]["fixed_frame_id"].as<std::string>();

        main_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("main_cloud",1);
        normal_cloud_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("normal_cloud",1);
        seg_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("segmented_regions", 1);
        rise_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("rise_regions", 1);
        tread_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("tread_regions", 1);
        tread_region_nums_pub_ = m_private_nh.advertise<visualization_msgs::MarkerArray>("tread_region_nums",1);
        rise_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("rise_cloud", 1);
        tread_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("tread_cloud", 1);
        rail_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("rail_cloud", 1);
        whole_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("whole_cloud", 1);
        is_stair_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("is_stair_cloud", 1);
        seg_stairs_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("segmented_stairs", 1);
        stair_parts_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("stair_parts",1);
        nearest_step_pose_pub_ = m_private_nh.advertise<geometry_msgs::PoseStamped>("nearest_step_pose",1);

        ROS_INFO("Initialized.");
    }

    inline void inputCB(const sensor_msgs::PointCloud2& input_msg)
    {
        if(!ros::ok()){ return; }

        ROS_INFO("Got cloud!");

        boost::mutex::scoped_lock lock(mutex_);

        stamp_ = input_msg.header.stamp;
        frame_ = input_msg.header.frame_id;

    // Loading input point cloud //

        double loadS = pcl::getTime();

        PointCloudT::Ptr mainCloud;
        mainCloud.reset (new PointCloudT);

        pcl::fromROSMsg(input_msg, *mainCloud);

    // Starting preanalysis //

        ROS_INFO("---");
        ROS_INFO("Received point cloud with %d points",mainCloud->width);

        ROS_INFO("Starting preanalysis");
        double preAS = pcl::getTime();

        Preanalysis pre;
        pre.loadConfig(config_["preanalysis"]);
        NormalCloud::Ptr prepNomalCloud;
        prepNomalCloud.reset(new NormalCloud);
        PointCloudT floorPC;
        PointCloudC prepNorMap;

        Eigen::Matrix4d T_fixed_input_mat = Eigen::Matrix4d::Identity();
        if (input_msg.header.frame_id != fixed_frame_id_)
        {
            if (!getTransformFromTree(listener, fixed_frame_id_, input_msg.header.frame_id, &T_fixed_input_mat, stamp_))
            {
                // busy = false;
                return;
            }
        }
        // Eigen::Matrix4d invTransformCloud = transformCloud.inverse();

        // transform mainCloud (default identity),
        // downsample (default true, 0.01m res),
        // normal estimation and init prepNomalCloud with output,
        // filter ghost/shadow points (nonexistent points at discontinuities due to sensor noise),
        // extract floor and init floorPC with output,
        // init prepNorMap with xyz's of mainCloud points and rgb's of mainCloud normals
        pre.run(mainCloud, prepNomalCloud, prepNorMap, floorPC, T_fixed_input_mat);
        double preAE = pcl::getTime();
        ROS_INFO("Preanalysis took: %f",preAE-preAS);

        // pubTCloud(&main_cloud_pub_, *mainCloud);
        // pubCCloud(&normal_cloud_pub_, *mainCloud, *prepNomalCloud);

    // Starting segmentation //

        ROS_INFO("Starting segmentation");
        int mode = 0;
        double segS = pcl::getTime();
        regions segRegions;
        if(mode == 0)
        {
            ROS_INFO("Using Region Growing algorithm");
            RegionGrowing reGrow;
            reGrow.loadConfig(config_["regiongrowing"]);
            reGrow.setInputCloud(mainCloud);
            reGrow.setNormalCloud(prepNomalCloud);
            // extract and init segRegions with smooth regions
            reGrow.run(segRegions);
        }
        if(mode == 1)
        {
            ROS_INFO("Using Voxel SAC algorithm");
            voxSAC voxelSAC;
            voxelSAC.setInputCloud(mainCloud);
            voxelSAC.setNormalCloud(prepNomalCloud);
            voxelSAC.run(segRegions);
        }
        if(mode == 2)
        {
            ROS_INFO("Using Split & Merge algorithm");
            splitMerge sam;
            sam.setInputCloud(mainCloud);
            sam.setNormalCloud(prepNomalCloud);
            sam.splitProcess();
            sam.mergeProcess(segRegions);
        }
        double segE = pcl::getTime();
        ROS_INFO("Segmentation found %d regions",segRegions.size());
        ROS_INFO("Segmentation took: %f",segE-segS);

        pubCCloud(&seg_regions_pub_,segRegions,fixed_frame_id_,stamp_);

    // Starting plane finder - plane extraction //

        ROS_INFO("Starting plane finder");

        double pfS = pcl::getTime();
        planeshape psProc;
        psProc.loadConfig(config_["planeshape"]);
        regions stairTreads;
        regions stairRisers;
        psProc.setInputRegions(segRegions);
        // extract treads and risers regions from segRegions
        psProc.filterSc(stairTreads, stairRisers);

        double pfE = pcl::getTime();
        ROS_INFO("Plane filter took: %f",pfE-pfS);

        // pubCCloud(&rise_regions_pub_,stairRisers);
        // pubCCloud(&tread_regions_pub_,stairTreads);
        // pubLabels(&tread_region_nums_pub_,stairTreads);

    // eigen based stair detection/prediction

        // ROS_INFO("Starting prediction");
        // prediction pred;
        // pred.setTreadRegions(stairTreads);
        // // // pred.setRiseRegions(stairRisers);
        // pred.setFixedTform(T_fixed_input_mat.cast<float>());
        // pred.setFixedFrame(fixed_frame_id_);
        // pred.run();
        // stairTreads.clear();
        // pred.getFilteredRegions(stairTreads);

        // pubCCloud(&rise_regions_pub_,stairRisers);
        // pubCCloud(&tread_regions_pub_,stairTreads);
        // pubLabels(&tread_region_nums_pub_,stairTreads);

        // busy = false;
        // return;

    // Starting graph-based stair detection //

        ROS_INFO("Starting graph-based detection");
        StairVector detectedStairs;

        double refS = pcl::getTime();
        recognition stairDetect;
        stairDetect.loadConfig(config_["recognition"]);
        stairDetect.setInputRegions(segRegions);
        stairDetect.setStairTreadRegions(stairTreads);
        stairDetect.setStairRiseRegions(stairRisers);
        // filter
        // // filter rise regions
        // // // filter: horizontal distance
        // // // filter: normal angle difference
        // // // filter: height initializations
        // // // filter: vertical distance
        // // // filter: inlination
        // // // find (extend)
        // // // check
        // // filter tread regions
        // // // filter: vertical distance
        // // // filter: direction initilizations
        // // // filter: depth initilizations
        // // // filter: horizontal distance
        // // // filter: inlination
        // // // find (extend)
        // // // check
        // sort
        // either simple search
        // // finalize
        // or extended search
        // // finalize
        stairDetect.run(detectedStairs);
        double refE = pcl::getTime();

        ROS_INFO("There are treads: %d",stairTreads.size());
        ROS_INFO("There are risers: %d",stairRisers.size());

        ROS_INFO("Refinement took: %f",refE-refS);
        ROS_INFO("Total time  took: %f",refE-loadS);

        if (detectedStairs.size() > 0)
        {
            pubTCloud(&rise_cloud_pub_,detectedStairs.at(0).stairRiseCloud,fixed_frame_id_);
            pubTCloud(&tread_cloud_pub_,detectedStairs.at(0).stairTreadCloud,fixed_frame_id_);
            pubTCloud(&rail_cloud_pub_,detectedStairs.at(0).stairRailCloud,fixed_frame_id_);
            PointCloudT wholeStairCloud;
            wholeStairCloud += detectedStairs.at(0).stairRiseCloud;
            wholeStairCloud += detectedStairs.at(0).stairTreadCloud;
            wholeStairCloud += detectedStairs.at(0).stairRailCloud;
            pubTCloud(&whole_cloud_pub_, wholeStairCloud,fixed_frame_id_,stamp_);
            
            pcl::PointCloud<pcl::PointXYZI> isStairCloud;
            for (uint i=0; i<mainCloud->points.size(); i++) {
                pcl::PointXYZI bpt;
                bpt.x = mainCloud->points[i].x;
                bpt.y = mainCloud->points[i].y;
                bpt.z = mainCloud->points[i].z;
                bpt.intensity = 0.f;
                float ptStairDistThresh = 0.01;
                for (uint j=0; j<wholeStairCloud.points.size(); j++) {
                    if (pcl::euclideanDistance(mainCloud->points[i],wholeStairCloud.points[j])<ptStairDistThresh) {
                        bpt.intensity = 1.f;    
                        continue;
                    }
                }
                isStairCloud.points.push_back(bpt);
            }
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(isStairCloud, cloud_msg);
            cloud_msg.header.frame_id = fixed_frame_id_;
            cloud_msg.header.stamp = stamp_;
            is_stair_cloud_pub_.publish(cloud_msg);
            // ros::spinOnce();
        }

    // Printing out the results //

        bool colorByPart = true;

        PointCloudC resultCloud;

        bool addBackGround = true;
        if(addBackGround)
        {
            for(size_t pointID = 0; pointID < mainCloud->size(); pointID ++)
            {
                PointTC backPoint;
                backPoint.x = mainCloud->points[pointID].x;
                backPoint.y = mainCloud->points[pointID].y;
                backPoint.z = mainCloud->points[pointID].z;
                backPoint.r=255;
                backPoint.g=255;
                backPoint.b=255;
                resultCloud.push_back(backPoint);
            }
        }

        ROS_INFO("Detected stairways: %d",detectedStairs.size());
        if(detectedStairs.size()>0)
        {
            for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
            {
                Stairs stairCoefficients;
                stairCoefficients = detectedStairs.at(stairCoeffIdx);

                float slope = atan(stairCoefficients.dir[2] / sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)));

                ROS_INFO("-");
                ROS_INFO("Step depth:   %f",round(1000*sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2))));
                ROS_INFO("Step height:  %f",round(1000*stairCoefficients.dir[2]));
                ROS_INFO("Step width:   %f",round(1000*stairCoefficients.width));
                ROS_INFO("Slope is:     %f",round(100*slope/M_PI*180));
                ROS_INFO("Amount of stair parts: %d",stairCoefficients.size());

                float stairAngle = atan2(stairCoefficients.dir[1],stairCoefficients.dir[0]);
                float xStairDist = stairCoefficients.pos[0];
                float yStairDist = stairCoefficients.pos[1];

                Eigen::Vector2f sepDist;
                sepDist[0] = cos(stairAngle) * xStairDist + sin(stairAngle) * yStairDist;
                sepDist[1] = - sin(stairAngle) * xStairDist + cos(stairAngle) * yStairDist;

                ROS_INFO("-");
                ROS_INFO("Dist in X is: %f",round(1000*(stairCoefficients.pos[0])));
                ROS_INFO("Dist in Y is: %f",round(1000*stairCoefficients.pos[1]));

                ROS_INFO("Dist par is:  %f",round(1000*sepDist[0]));
                ROS_INFO("Dist ort is:  %f",round(1000*sepDist[1]));
                ROS_INFO("Anchor point is: %d",stairCoefficients.anchPoint);

                ROS_INFO("Angle is:     %f",round(100*atan2(stairCoefficients.dir[1],stairCoefficients.dir[0])/M_PI*180));

                if(colorByPart)
                    resultCloud += detectedStairs.getColoredParts(stairCoeffIdx);
                else
                    resultCloud += detectedStairs.getColoredCloud(stairCoeffIdx);
            }

        } // if(detectedStairs.size()>0)

        if(detectedStairs.size()>0)
        {
            uint stairCoeffIdx = 0;

            Stairs stairCoefficients = detectedStairs.at(stairCoeffIdx);
            regions stairParts = stairCoefficients.stairParts;
            std::vector<int> planeLabels = stairCoefficients.planeLabels;
            ROS_INFO(stairCoefficients.str().c_str());

            pubCCloud(&stair_parts_pub_,stairParts,fixed_frame_id_,stamp_);
            pubCCloud(&seg_stairs_pub_,resultCloud,fixed_frame_id_,stamp_);
        } // if(detectedStairs.size()>0)

        if(detectedStairs.size()>0)
        {
            geometry_msgs::PoseStamped step_pose;

            // Eigen::Vector3f weighted_dir(0,0,0);
            // std::vector<Eigen::Vector3f> diff_vecs;
            // for (uint i=0; i<filtered_idxs_.size()-1; i++)
            // {
            //     for (uint j=i+1; j<filtered_idxs_.size(); j++)
            //     {
            //         // if (i==j){ continue; }
            //         Eigen::Vector4f point0 = treads_.at(i).segmentCentroid;
            //         Eigen::Vector4f point1 = treads_.at(j).segmentCentroid;
            //         Eigen::Vector3f diff_vec = point0.head(3)-point1.head(3);
            //         diff_vec = (diff_vec.sum()>0 ? 1 : -1)*diff_vec;
            //         diff_vec.normalize();
            //         diff_vecs.push_back(diff_vec);

            //         Eigen::Vector3f point0_input = Eigen::Vector4f(T_fixed_input_mat_*point0).head(3);
            //         Eigen::Vector3f point1_input = Eigen::Vector4f(T_fixed_input_mat_*point1).head(3);
            //         float strong_nearness_factor = 1.0; // loosely corresponds to norm at which weight saturates
            //         float weight = std::min(float(10.0),float(pow(10,strong_nearness_factor/point0_input.norm())*pow(10,strong_nearness_factor/point1_input.norm())));
            //         weighted_dir += diff_vec*weight;

            //         ROS_INFO("diff %d->%d %f %f %f",i,j,diff_vec[0],diff_vec[1],diff_vec[2]);
            //     }
            // }
            // weighted_dir.normalize();
            // stair_dir_ = weighted_dir;
            // ROS_INFO("stair dir %f %f %f",weighted_dir[0],weighted_dir[1],weighted_dir[2]);

            // std::vector<Eigen::Vector3f> dist_vecs;
            // std::vector<float> dist_norms;
            // Eigen::Vector3f nearest_step;
            // float min_dist = INFINITY;
            // for (uint j=0; j<filtered_idxs_.size(); j++)
            // {
            //     Eigen::Vector4f stairctr = treads_.at(j).segmentCentroid;
            //     Eigen::Vector4f stairctr_from_input = T_fixed_input_mat_*stairctr;
            //     Eigen::Vector3f stairctr_head = stairctr_from_input.head(3);
            //     float stairctr_head_dist = stairctr_head.norm();
            //     dist_vecs.push_back(stairctr_head);
            //     dist_norms.push_back(stairctr_head_dist);
            //     // ROS_INFO("dist %d norm %f vec %f %f %f",j,stairctr_head_norm,stairctr_head[0],stairctr_head[1],stairctr_head[2]);

            //     if (stairctr_head_dist < min_dist)
            //     {
            //         min_dist = stairctr_head_dist;
            //         nearest_step = stairctr.head(3);
            //     }
            // }
            // nearest_step_ = nearest_step;
            // ROS_INFO("nearest step %f %f %f",nearest_step[0],nearest_step[1],nearest_step[2]);

            // float roll = 0;
            // // float pitch = atan(stair_dir_[2]/sqrt(pow(stair_dir_[0],2)+pow(stair_dir_[1],2)));
            // float pitch = -asin(stair_dir_[2]);
            // float yaw = atan2(stair_dir_[1],stair_dir_[0]);
            // Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
            //     * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            //     * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            // q.normalize();

            // ROS_INFO("pitch %f yaw %f",pitch,yaw);

            // step_pose.header.frame_id = fixed_frame_id_;
            // step_pose.header.stamp = ros::Time::now();
            // step_pose.pose.position.x = nearest_step_[0];
            // step_pose.pose.position.y = nearest_step_[1];
            // step_pose.pose.position.z = nearest_step_[2];
            // step_pose.pose.orientation.x = q.x();
            // step_pose.pose.orientation.y = q.y();
            // step_pose.pose.orientation.z = q.z();
            // step_pose.pose.orientation.w = q.w();

            ///////////////

            uint stairIdx=0;
            float minDist = INFINITY;
            // uint minIdx;
            Eigen::Vector3f minPos;
            float maxDist = -INFINITY;
            // uint maxIdx;
            Eigen::Vector3f maxPos;
            // for (uint i=0; i<detectedStairs.size(); i++) // for each set of stairs
            // {
            //     stairIdx = i;
            //     ROS_INFO("On stairs %d/%d",i,detectedStairs.size());
                for (uint j=0; j<detectedStairs.at(stairIdx).stairTreads.size(); j++) // for each tread
                {
                    // ROS_INFO("On treads %d/%d",j,detectedStairs.at(stairIdx).stairTreads.size());
                    float dist = detectedStairs.at(stairIdx).stairTreads.regs[j].segmentCentroid.head(3).norm();
                    if (dist<minDist)
                    {
                        minDist = dist;
                        // minIdx = j;
                        minPos = detectedStairs.at(stairIdx).stairTreads.regs[j].segmentCentroid.head(3);
                    }
                    if (dist>maxDist)
                    {
                        maxDist = dist;
                        // maxIdx = j;
                        maxPos = detectedStairs.at(stairIdx).stairTreads.regs[j].segmentCentroid.head(3);
                    }
                }

                for (uint j=0; j<detectedStairs.at(stairIdx).stairRises.size(); j++) // for each tread
                {
                    // ROS_INFO("On rises %d/%d",j,detectedStairs.at(stairIdx).stairRises.size());
                    float dist = detectedStairs.at(stairIdx).stairRises.regs[j].segmentCentroid.head(3).norm();
                    if (dist<minDist)
                    {
                        minDist = dist;
                        // minIdx = j;
                        minPos = detectedStairs.at(stairIdx).stairRises.regs[j].segmentCentroid.head(3);
                    }
                    if (dist>maxDist)
                    {
                        maxDist = dist;
                        // maxIdx = j;
                        maxPos = detectedStairs.at(stairIdx).stairRises.regs[j].segmentCentroid.head(3);
                    }
                }
            // }

            Eigen::Vector3f nearest_step_pos = minPos;
            Eigen::Vector3f nearest_step_dir = maxPos - minPos;

            ROS_INFO("stepPos %f %f %f stepDir %f %f %f",nearest_step_pos[0],nearest_step_pos[1],nearest_step_pos[2],nearest_step_dir[0],nearest_step_dir[1],nearest_step_dir[2]);

            // float roll = 0;
            // float pitch = atan(nearest_step_dir[2]/sqrt(pow(nearest_step_dir[0],2)+pow(nearest_step_dir[1],2)));
            // // float pitch = -asin(nearest_step_dir[2]);
            // float yaw = atan2(nearest_step_dir[1],nearest_step_dir[0]);
            // Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
            //     * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            //     * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            // q.normalize();

            // float angle = atan2( nearest_step_dir[1], nearest_step_dir[2] ); // Note: I expected atan2(z,x) but OP reported success with atan2(x,z) instead! Switch around if you see 90° off.
            // Eigen::Quaternionf q(cos(angle/2), 0, sin(angle/2), 0);

            float qw = sqrt(pow(Eigen::Vector3f(1,0,0).norm(),2) * pow(nearest_step_dir.norm(),2)) + Eigen::Vector3f(1,0,0).dot(nearest_step_dir);
            Eigen::Vector3f xyz = Eigen::Vector3f(1,0,0).cross(nearest_step_dir);
            Eigen::Quaternionf q(qw,xyz[0],xyz[1],xyz[2]);

            // ROS_INFO("stepRPY %f %f %f",roll,pitch,yaw);
            ROS_INFO("stepQuat %f %f %f %f",q.x(),q.y(),q.z(),q.w());

            step_pose.header.frame_id = fixed_frame_id_;
            // step_pose.header.stamp = ros::Time::now();
            step_pose.header.stamp = stamp_;
            step_pose.pose.position.x = nearest_step_pos[0];
            step_pose.pose.position.y = nearest_step_pos[1];
            step_pose.pose.position.z = nearest_step_pos[2];
            step_pose.pose.orientation.x = q.x();
            step_pose.pose.orientation.y = q.y();
            step_pose.pose.orientation.z = q.z();
            step_pose.pose.orientation.w = q.w();

            pubPose(&nearest_step_pose_pub_,step_pose);
        }
    }
};

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "stair_detection_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    StairDetection sd(n,np);

    ros::spin();
}
