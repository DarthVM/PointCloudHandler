#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class pclWrapper
{
private:
    pcl::PointCloud<pcl::PointXYZ> InitialCloud;
    pcl::PointCloud<pcl::PointXYZ> MeshedCloud;

public:
    pclWrapper(std::string&, std::string&);
    

    
};

