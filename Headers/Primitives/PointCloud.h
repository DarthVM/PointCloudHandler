#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#pragma once

#include "../typedefs.h"

class PointCloud {
private:
    Point_vector points;
public:
    PointCloud(const std::string &cloud_filepath);
    Point_vector getPoints();
    /** \brief Save given point cloud as file
         \param[in] output_filepath path to the output file
         \param[in] cloud point cloud to save
    */
    static void save(const std::string &output_filepath, const Point_vector &cloud);

    void estimateNormals(int k = 18);
    void estimateNormals();

    /** \brief Extract planes using RANSAC
    \param normal_angle
    \param probability Set probability to miss the largest primitive at each iteration
    \param shape_min_points Detect shapes with at least ... points
    \param epsilon Set maximum Euclidean distance between a point and a shape
    \param cluster_epsilon Set maximum Euclidean distance between points to be clustered
    */
    Point_vector Efficient_Ransac(const float &normal_angle, const float &probability = 0.05,
                                  const size_t &shape_min_points = 200, const float &epsilon = 0.002,
                                  const float &cluster_epsilon = 0.01);


    void Region_growing_Shape_Detector(const float &max_accepted_angle = 25,
                                       const float &search_sphere_radius = 2 / 100,
                                       const float &max_distance_to_plane = 2 / 100,
                                       const std::size_t &min_region_size = 200);

    Mesh triangulate();

    /** \brief Triangulate point cloud (create mesh of triangle polygons)
    \param search_sphere_radius (Default value: 2 / 100)
    \param max_distance_to_plane (Default value: 2 / 100)
    \param max_accepted_angle (Default value: 25)
    \param min_region_size Min region size (Default value: 200)
    */
    Mesh reconstructFront();
    static Mesh reconstructFront(std::vector<Point>& points);
};


#endif // POINT_CLOUD_H
