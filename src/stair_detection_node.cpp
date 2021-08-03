// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <stair_detection/preanalysis.h>
#include <stair_detection/regions.h>
#include <stair_detection/regiongrowing.h>
#include <stair_detection/voxSAC.h>
#include <stair_detection/splitmerge.h>
#include <stair_detection/planeshape.h>
#include <stair_detection/recognition.h>
#include <stair_detection/StairVector.h>
// #include <stair_detection/prediction.h>

#include <stair_detection/ros_functions.hpp>
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

#include <dynamic_reconfigure/server.h>
#include <stair_detection/StairDetectionConfig.h>


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

    StairDetectionParams params_;

    ros::NodeHandle m_nh, m_private_nh;
    ros::Time stamp_;
    std::string frame_;
    tf::TransformListener listener;
    ros::Subscriber input_sub;
    dynamic_reconfigure::Server<stair_detection::StairDetectionConfig> cfg_server_;

    std::string fixed_frame_id_;
    ros::Publisher main_cloud_pub_;
    ros::Publisher normal_cloud_pub_;
    ros::Publisher seg_regions_pub_;
    ros::Publisher seg_region_nums_pub_;
    ros::Publisher rise_regions_pub_;
    ros::Publisher tread_regions_pub_;
    ros::Publisher risers_region_nums_pub_;
    ros::Publisher treads_region_nums_pub_;
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

        cfg_server_.setCallback(boost::bind(&StairDetection::cfgCb, this, _1, _2));

        m_private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("map"));
        // m_private_nh.param("pub_viz", params_.pub_viz_, true);
        m_private_nh.param("preanalysis/dsFlag", params_.preanalysis.dsFlag, true);
        m_private_nh.param("preanalysis/dsResolution", params_.preanalysis.dsResolution, 0.01);
        m_private_nh.param("preanalysis/neNeighMethod", params_.preanalysis.neNeighMethod, 0);
        m_private_nh.param("preanalysis/neSearchNeighbours", params_.preanalysis.neSearchNeighbours, 24);
        m_private_nh.param("preanalysis/neSearchRadius", params_.preanalysis.neSearchRadius, 0.2);
        m_private_nh.param("preanalysis/gpFlag", params_.preanalysis.gpFlag, false);
        m_private_nh.param("preanalysis/gpAngle", params_.preanalysis.gpAngle, 25.0);
        m_private_nh.param("preanalysis/pfActive", params_.preanalysis.pfActive, false);
        m_private_nh.param("preanalysis/pfAngle", params_.preanalysis.pfAngle, 20.0);
        m_private_nh.param("preanalysis/fsActive", params_.preanalysis.fsActive, false);
        m_private_nh.param("preanalysis/fsAngle", params_.preanalysis.fsAngle, 30.0);
        m_private_nh.param("preanalysis/fsRange", params_.preanalysis.fsRange, 0.05);
        m_private_nh.param("preanalysis/rob_x", params_.preanalysis.rob_x, 0.0);
        m_private_nh.param("preanalysis/rob_y", params_.preanalysis.rob_y, 0.0);
        m_private_nh.param("preanalysis/rob_z", params_.preanalysis.rob_z, 0.0);
        m_private_nh.param("preanalysis/robAngle", params_.preanalysis.robAngle, 0.0);
        m_private_nh.param("preanalysis/dsMethod", params_.preanalysis.dsMethod, false);
        m_private_nh.param("preanalysis/neMethod", params_.preanalysis.neMethod, 0);
        m_private_nh.param("segmentationmode", params_.segmentationmode, 0);
        m_private_nh.param("regiongrowing/minClustSize", params_.regiongrowing.minClustSize, 30);
        m_private_nh.param("regiongrowing/noNeigh", params_.regiongrowing.noNeigh, 24);
        m_private_nh.param("regiongrowing/smoothFlag", params_.regiongrowing.smoothFlag, false);
        m_private_nh.param("regiongrowing/smoothThresh", params_.regiongrowing.smoothThresh, 50.0);
        m_private_nh.param("regiongrowing/resFlag", params_.regiongrowing.resFlag, true);
        m_private_nh.param("regiongrowing/resThresh", params_.regiongrowing.resThresh, 0.08);
        m_private_nh.param("regiongrowing/curvFlag", params_.regiongrowing.curvFlag, false);
        m_private_nh.param("regiongrowing/curvThresh", params_.regiongrowing.curvThresh, 0.1);
        m_private_nh.param("regiongrowing/updateFlag", params_.regiongrowing.updateFlag, true);
        m_private_nh.param("regiongrowing/pointUpdateFlag", params_.regiongrowing.pointUpdateFlag, true);
        m_private_nh.param("regiongrowing/updateInterval", params_.regiongrowing.updateInterval, 100);
        m_private_nh.param("planeshape/angleMargin", params_.planeshape.angleMargin, 0.0);
        m_private_nh.param("planeshape/widthReqMin", params_.planeshape.widthReqMin, 0.0);
        m_private_nh.param("planeshape/widthReqMax", params_.planeshape.widthReqMax, 10.0);
        m_private_nh.param("planeshape/treadDepthMin", params_.planeshape.treadDepthMin, 0.0);
        m_private_nh.param("planeshape/treadDepthMax", params_.planeshape.treadDepthMax, 0.50);
        m_private_nh.param("planeshape/riserHeightMin", params_.planeshape.riserHeightMin, 0.0);
        m_private_nh.param("planeshape/riserHeightMax", params_.planeshape.riserHeightMax, 0.24);
        m_private_nh.param("recognition/graphMeth", params_.recognition.graphMeth, false);
        m_private_nh.param("recognition/optimizeFlag", params_.recognition.optimizeFlag, true);
        m_private_nh.param("recognition/widthReqVecMin", params_.recognition.widthReqVecMin, 0.0);
        m_private_nh.param("recognition/widthReqVecMax", params_.recognition.widthReqVecMax, 10.0);
        m_private_nh.param("recognition/widthFlag", params_.recognition.widthFlag, true);
        m_private_nh.param("recognition/parFlag", params_.recognition.parFlag, true);
        m_private_nh.param("recognition/parAngle", params_.recognition.parAngle, 15.0);
        m_private_nh.param("recognition/ndFlag", params_.recognition.ndFlag, true);
        m_private_nh.param("recognition/nDistanceMin", params_.recognition.nDistanceMin, 0.11);
        m_private_nh.param("recognition/nDistanceMax", params_.recognition.nDistanceMax, 0.24);
        m_private_nh.param("recognition/pdFlag", params_.recognition.pdFlag, true);
        m_private_nh.param("recognition/pDistanceMin", params_.recognition.pDistanceMin, 0.18);
        m_private_nh.param("recognition/pDistanceMax", params_.recognition.pDistanceMax, 0.50);
        m_private_nh.param("recognition/floorInformation", params_.recognition.floorInformation, false);
        m_private_nh.param("recognition/updateFlag", params_.recognition.updateFlag, false);
        m_private_nh.param("recognition/stairRailFlag", params_.recognition.stairRailFlag, false);
        m_private_nh.param("recognition/predifinedValues", params_.recognition.predifinedValues, false);
        m_private_nh.param("recognition/preDefDepth", params_.recognition.preDefDepth, 0.0);
        m_private_nh.param("recognition/preDefHeight", params_.recognition.preDefHeight, 0.0);
        m_private_nh.param("recognition/preDefWidth", params_.recognition.preDefWidth, 0.0);
        m_private_nh.param("recognition/maxStairRiseDist", params_.recognition.maxStairRiseDist, 0.05);
        m_private_nh.param("recognition/maxStairRiseHDist", params_.recognition.maxStairRiseHDist, 0.05);
        m_private_nh.param("recognition/maxStairTreadDist", params_.recognition.maxStairTreadDist, 0.05);
        m_private_nh.param("recognition/maxStairRiseAngle", params_.recognition.maxStairRiseAngle, 30.0);
        m_private_nh.param("recognition/minStairIncAngle", params_.recognition.minStairIncAngle, 10.0);
        m_private_nh.param("recognition/maxStairIncAngle", params_.recognition.maxStairIncAngle, 50.0);

        main_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("main_cloud",1);
        normal_cloud_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("normal_cloud",1);
        seg_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("segmented_regions", 1);
        seg_region_nums_pub_ = m_private_nh.advertise<visualization_msgs::MarkerArray>("seg_region_nums",1);
        rise_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("rise_regions", 1);
        tread_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("tread_regions", 1);
        risers_region_nums_pub_ = m_private_nh.advertise<visualization_msgs::MarkerArray>("risers_region_nums",1);
        treads_region_nums_pub_ = m_private_nh.advertise<visualization_msgs::MarkerArray>("treads_region_nums",1);
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

    inline void cfgCb(stair_detection::StairDetectionConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure requested.");

        params_.preanalysis.dsFlag = config.groups.preanalysis.dsFlag;
        params_.preanalysis.dsResolution = config.groups.preanalysis.dsResolution;
        params_.preanalysis.neNeighMethod = config.groups.preanalysis.neNeighMethod;
        params_.preanalysis.neSearchNeighbours = config.groups.preanalysis.neSearchNeighbours;
        params_.preanalysis.neSearchRadius = config.groups.preanalysis.neSearchRadius;
        params_.preanalysis.gpFlag = config.groups.preanalysis.gpFlag;
        params_.preanalysis.gpAngle = config.groups.preanalysis.gpAngle;
        params_.preanalysis.pfActive = config.groups.preanalysis.pfActive;
        params_.preanalysis.pfAngle = config.groups.preanalysis.pfAngle;
        params_.preanalysis.fsActive = config.groups.preanalysis.fsActive;
        params_.preanalysis.fsAngle = config.groups.preanalysis.fsAngle;
        params_.preanalysis.fsRange = config.groups.preanalysis.fsRange;
        params_.preanalysis.rob_x = config.groups.preanalysis.rob_x;
        params_.preanalysis.rob_y = config.groups.preanalysis.rob_y;
        params_.preanalysis.rob_z = config.groups.preanalysis.rob_z;
        params_.preanalysis.robAngle = config.groups.preanalysis.robAngle;
        params_.preanalysis.dsMethod = config.groups.preanalysis.dsMethod;
        params_.preanalysis.neMethod = config.groups.preanalysis.neMethod;

        params_.segmentationmode = config.segmentationmode;

        params_.regiongrowing.minClustSize = config.groups.region_growing.minClustSize;
        params_.regiongrowing.noNeigh = config.groups.region_growing.noNeigh;
        params_.regiongrowing.smoothFlag = config.groups.region_growing.smoothFlag;
        params_.regiongrowing.smoothThresh = config.groups.region_growing.smoothThresh;
        params_.regiongrowing.resFlag = config.groups.region_growing.resFlag;
        params_.regiongrowing.resThresh = config.groups.region_growing.resThresh;
        params_.regiongrowing.curvFlag = config.groups.region_growing.curvFlag;
        params_.regiongrowing.curvThresh = config.groups.region_growing.curvThresh;
        params_.regiongrowing.updateFlag = config.groups.region_growing.rgUpdateFlag;
        params_.regiongrowing.pointUpdateFlag = config.groups.region_growing.rgPointUpdateFlag;
        params_.regiongrowing.updateInterval = config.groups.region_growing.updateInterval;

        params_.planeshape.angleMargin = config.groups.ps.angleMargin;
        params_.planeshape.widthReqMin = config.groups.ps.widthReqMin;
        params_.planeshape.widthReqMax = config.groups.ps.widthReqMax;
        params_.planeshape.treadDepthMin = config.groups.ps.treadDepthMin;
        params_.planeshape.treadDepthMax = config.groups.ps.treadDepthMax;
        params_.planeshape.riserHeightMin = config.groups.ps.riserHeightMin;
        params_.planeshape.riserHeightMax = config.groups.ps.riserHeightMax;

        params_.recognition.graphMeth = config.groups.rec.graphMeth;
        params_.recognition.optimizeFlag = config.groups.rec.optimizeFlag;
        params_.recognition.widthReqVecMin = config.groups.rec.widthReqVecMin;
        params_.recognition.widthReqVecMax = config.groups.rec.widthReqVecMax;
        params_.recognition.widthFlag = config.groups.rec.widthFlag;
        params_.recognition.parFlag = config.groups.rec.parFlag;
        params_.recognition.parAngle = config.groups.rec.parAngle;
        params_.recognition.ndFlag = config.groups.rec.ndFlag;
        params_.recognition.nDistanceMin = config.groups.rec.nDistanceMin;
        params_.recognition.nDistanceMax = config.groups.rec.nDistanceMax;
        params_.recognition.pdFlag = config.groups.rec.pdFlag;
        params_.recognition.pDistanceMin = config.groups.rec.pDistanceMin;
        params_.recognition.pDistanceMax = config.groups.rec.pDistanceMax;
        params_.recognition.floorInformation = config.groups.rec.floorInformation;
        params_.recognition.updateFlag = config.groups.rec.recUpdateFlag;
        params_.recognition.stairRailFlag = config.groups.rec.stairRailFlag;
        params_.recognition.predifinedValues = config.groups.rec.predifinedValues;
        params_.recognition.preDefDepth = config.groups.rec.preDefDepth;
        params_.recognition.preDefHeight = config.groups.rec.preDefHeight;
        params_.recognition.preDefWidth = config.groups.rec.preDefWidth;
        params_.recognition.maxStairRiseDist = config.groups.rec.maxStairRiseDist;
        params_.recognition.maxStairRiseHDist = config.groups.rec.maxStairRiseHDist;
        params_.recognition.maxStairTreadDist = config.groups.rec.maxStairTreadDist;
        params_.recognition.maxStairRiseAngle = config.groups.rec.maxStairRiseAngle;
        params_.recognition.minStairIncAngle = config.groups.rec.minStairIncAngle;
        params_.recognition.maxStairIncAngle = config.groups.rec.maxStairIncAngle;
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
        pre.loadConfig(params_.preanalysis);
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
        double segS = pcl::getTime();
        regions segRegions;
        if(params_.segmentationmode == 0)
        {
            ROS_INFO("Using Region Growing algorithm");
            RegionGrowing reGrow;
            reGrow.loadConfig(params_.regiongrowing);
            reGrow.setInputCloud(mainCloud);
            reGrow.setNormalCloud(prepNomalCloud);
            // extract and init segRegions with smooth regions
            reGrow.run(segRegions);
        }
        if(params_.segmentationmode == 1)
        {
            ROS_INFO("Using Voxel SAC algorithm");
            voxSAC voxelSAC;
            voxelSAC.setInputCloud(mainCloud);
            voxelSAC.setNormalCloud(prepNomalCloud);
            voxelSAC.run(segRegions);
        }
        if(params_.segmentationmode == 2)
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
        pubLabels(&seg_region_nums_pub_,segRegions,fixed_frame_id_,stamp_);

    // Starting plane finder - plane extraction //

        ROS_INFO("Starting plane finder");

        double pfS = pcl::getTime();
        planeshape psProc;
        psProc.loadConfig(params_.planeshape);
        regions stairTreads;
        regions stairRisers;
        psProc.setInputRegions(segRegions);
        // extract treads and risers regions from segRegions
        psProc.filterSc(stairTreads, stairRisers);

        double pfE = pcl::getTime();
        ROS_INFO("Plane filter took %fs to find risers size %d, treads size %d",pfE-pfS,stairRisers.regs.size(), stairTreads.regs.size());

        if (!stairRisers.regs.empty()) pubCCloud(&rise_regions_pub_,stairRisers,fixed_frame_id_,stamp_);
        if (!stairTreads.regs.empty()) pubCCloud(&tread_regions_pub_,stairTreads,fixed_frame_id_,stamp_);
        pubLabels(&risers_region_nums_pub_,stairRisers,fixed_frame_id_,stamp_);
        pubLabels(&treads_region_nums_pub_,stairTreads,fixed_frame_id_,stamp_);

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

        // busy = false;
        // return;

    // Starting graph-based stair detection //

        ROS_INFO("Starting graph-based detection");
        StairVector detectedStairs;

        double refS = pcl::getTime();
        recognition stairDetect;
        stairDetect.loadConfig(params_.recognition);
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

        {
          PointCloudT wholeStairCloud;
          for (uint i=0; i<detectedStairs.size(); i++)
          {
              // if (detectedStairs.at(i).stairScore[0]>0.9 && detectedStairs.at(i).stairScore[1]>0.9 && detectedStairs.at(i).stairScore[2]>0.9) {
              // if (detectedStairs.at(i).accuracy > 0.1) {
              if (detectedStairs.at(i).stairScore[1]>0.9) {
                wholeStairCloud += detectedStairs.at(i).stairRiseCloud;
                wholeStairCloud += detectedStairs.at(i).stairTreadCloud;
                // wholeStairCloud += detectedStairs.at(0).stairRailCloud;
              }
          }

          pcl::PointCloud<pcl::PointXYZI> isStairCloud;
          for (uint i=0; i<mainCloud->points.size(); i++) {
              pcl::PointXYZI bpt;
              bpt.x = mainCloud->points[i].x;
              bpt.y = mainCloud->points[i].y;
              bpt.z = mainCloud->points[i].z;
              bpt.intensity = 0.f;
              float ptStairDistThresh = 0.005;
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

                ROS_INFO("- %d -", stairCoeffIdx);
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

                // ROS_INFO("-");
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

        for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
        {
            Stairs stairCoefficients = detectedStairs.at(stairCoeffIdx);
            regions stairParts = stairCoefficients.stairParts;
            std::vector<int> planeLabels = stairCoefficients.planeLabels;
            ROS_INFO("- %d -", stairCoeffIdx);
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
