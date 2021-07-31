#ifndef PLANESHAPE
#define PLANESHAPE

#include <stair_detection/regions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <stair_detection/ros_functions.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

class planeshape {
public:
    regions segments;
    float angleMargin;
    Eigen::Vector2f widthReq;
    Eigen::Vector2f treadDepth;
    Eigen::Vector2f riserHeight;

    planeshape();

    void loadConfig(StairDetectionParams::PlaneshapeParams);

    inline void clear()
    {
        segments.clear();
    }

    inline void setInputRegions(regions input)
    {
        segments = input;
    }

    inline void setAngleMargin(float input)
    {
        angleMargin = input;
    }

    inline void setWidthReq(Eigen::Vector2f input)
    {
    	widthReq = input;
    }

    inline void setTreadDepth(Eigen::Vector2f input)
    {
    	treadDepth = input;
    }

    inline void setRiserHeight(Eigen::Vector2f input)
    {
    	riserHeight = input;
    }

    void filterSc(regions& stairCases, regions& stairWalls);
};

#endif // PLANESHAPE

