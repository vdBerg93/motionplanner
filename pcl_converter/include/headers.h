#include <ros/ros.h>

// Message headers
#include <sensor_msgs/PointCloud2.h>
//#include <vision_msgs/Detection3DArray.h>
//#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/BoundingBox3D.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

const double pi = M_PI;
const double inf = std::numeric_limits<double>::infinity();

struct vector2D{
    double dx, dy;
};

struct OBB{
    double theta, size_x, size_y, area;
};

double myDot(vector2D a, vector2D b){
    return a.dx*b.dx + a.dy*b.dy;
};

void findBestOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, vision_msgs::Detection2D &det);