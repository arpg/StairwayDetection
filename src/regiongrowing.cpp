#include <stairs/regiongrowing.h>

RegionGrowing::RegionGrowing()
{
// Minimum cluster size //
	minClustSize = 30;
// Number of neighbors //
	noNeigh = 24;
// Smoothness flag (true = compare to seed point; false = compare to neighboring point) //
	smoothFlag = true;
// Smoothness threshold //
	smoothThresh = 30.0; // for RGB-D data
	// smoothThresh = 20.0; // for LIDAR data
// Residual flag (true = compare to seed point; false = compare to neighboring point) //
	resFlag = true;
// Residual distance //
	resThresh = 0.08; // for RGB-D data
	// resThresh = 0.02; // for LIDAR data
// Curvature flag //
	curvFlag = false;
// Curvature threshold //
	curvThresh = 0.1; // for RGB-D data
	// curvThresh = 0.048; // for LIDAR data

// Update seed point during growing //
	updateFlag = true;
// Update pointwise //
	pointUpdateFlag = true;
// If not pointwise, update every: //
	updateInterval = 100;
}

//RegionGrowing::~RegionGrowing()
//{
//
//}

void RegionGrowing::run(regions& output)
{
    // Region growing

    pcl::RegionGrowingOwn<PointT, Normal> rg;
    rg.setMinClusterSize (minClustSize);
    rg.setNumberOfNeighbours (noNeigh);
    rg.setSmoothModeFlag (smoothFlag); // Depends on the cloud being processed
    rg.setSmoothnessThreshold (smoothThresh*M_PI/180);
    rg.setResidualTestFlag (resFlag);
    rg.setResidualThreshold (resThresh);
    rg.setCurvatureTestFlag (curvFlag);
    rg.setCurvatureThreshold (curvThresh);
    if(updateFlag)
    {
        if(pointUpdateFlag)
        {
            rg.setPointUpdateFlag(true);
            rg.setUpdateFlag(false);
        }
        else
        {
            rg.setUpdateFlag(true);
            rg.setPointUpdateFlag(false);
        }
    }
    else
    {
        rg.setPointUpdateFlag(false);
        rg.setUpdateFlag(false);
    }
    rg.setUpdateInterval(updateInterval);

    rg.setInputCloud (inputCloud);
    rg.setInputNormals (normalCloud);
    std::vector <pcl::PointIndices> clusters;
    rg.extract (clusters);
    cloud_segmented = rg.getColoredCloud ();
    output.regs.resize(clusters.size());

    for(size_t clustCounter=0;clustCounter < clusters.size(); clustCounter++)
    {
    	segmentPatch tempPatch;
        for(size_t indexCount=0; indexCount < clusters.at(clustCounter).indices.size(); indexCount++)
        {
        	tempPatch.segmentCloud.push_back(inputCloud->at(clusters.at(clustCounter).indices.at(indexCount)));
        	tempPatch.segmentIndices->indices.push_back(indexCount);
        	tempPatch.globalIndices.push_back(clusters.at(clustCounter).indices.at(indexCount));
        }
        output.at(clustCounter)=tempPatch;
    }
    for(size_t segmentCounter=0; segmentCounter < output.regs.size(); segmentCounter++)
    {
    	output.at(segmentCounter).segmentLabel = segmentCounter;
    	output.at(segmentCounter).analyse();
    }
    output.getExtensions();
//    output = segmentList;
}

void RegionGrowing::loadConfig(YAML::Node config)
{
    // Minimum cluster size //
	minClustSize = config["minClustSize"].as<int>();
    // Number of neighbors //
	noNeigh = config["noNeigh"].as<int>();
    // Smoothness flag (true = compare to seed point; false = compare to neighboring point) //
	smoothFlag = config["smoothFlag"].as<bool>();
    // Smoothness threshold //
	smoothThresh = config["smoothThresh"].as<double>(); 
    // Residual flag (true = compare to seed point; false = compare to neighboring point) //
	resFlag = config["resFlag"].as<bool>();
    // Residual distance //
	resThresh = config["resThresh"].as<double>();
    // Curvature flag //
	curvFlag = config["curvFlag"].as<bool>();
    // Curvature threshold //
	curvThresh = config["curvThresh"].as<double>(); 
    // Update seed point during growing //
	updateFlag = config["updateFlag"].as<bool>();
    // Update pointwise //
	pointUpdateFlag = config["pointUpdateFlag"].as<bool>();
    // If not pointwise, update every: //
	updateInterval = config["updateInterval"].as<int>();
}

