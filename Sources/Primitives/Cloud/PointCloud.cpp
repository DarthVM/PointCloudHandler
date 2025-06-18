#include "../../../Headers/Primitives/PointCloud.h"

#include <CGAL/IO/read_points.h>
#include <CGAL/Timer.h>
#include <CGAL/property_map.h>

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>


PointCloud::PointCloud(const std::string& cloud_filepath) {
    CGAL::Timer t;
    t.start();

    std::cout << "Processing Point Cloud file...";
    if (!CGAL::IO::read_points(cloud_filepath.c_str(), std::back_inserter(points),
        CGAL::parameters::point_map(Point_map()))) {
        std::cerr << "Error: cannot read file " << cloud_filepath << std::endl;
      }
    else {

        for (auto& p : points) {
            auto uuid = uuid_generator();
            auto uuidString = boost::lexical_cast<std::string>(uuid);
            p.second = uuidString;
        }

        std::cout << " Done. " << points.size() << " points. Time: " << t.time() << " sec." << std::endl;
    }

}

PointCloud::PointCloud(const Points &points) {
    for (auto& p : points) {
        auto uuid = uuid_generator();
        auto uuidString = boost::lexical_cast<std::string>(uuid);

        auto pair = std::pair(p, uuidString);
        this->points.push_back(pair);
    }
}

Point_vector PointCloud::getPoints() {
    return points;
};


/*void PointCloud::estimateNormals(const int k) {
    CGAL::Timer t;
    t.start();
    std::cout << "Estimating points normals...";

    double spacing
         = CGAL::compute_average_spacing<Concurrency_tag>
           (points, k,
            CGAL::parameters::point_map(Point_map()));



    /*CGAL::jet_estimate_normals<Concurrency_tag>
       (points, k,
        CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()));#1#
    CGAL::pca_estimate_normals<Concurrency_tag>
       (points, 0,
        CGAL::parameters::point_map(Point_map()).normal_map(Normal_map())
        .neighbor_radius(2. * spacing)); // use 2*spacing as neighborhood radius

    //std::list<PointVectorPair> points;
    // Orients normals.
    // Note: mst_orient_normals() requires a range of points
    //  as well as property maps to access each point's position and mean_normal.
    CGAL::mst_orient_normals(points, 0,
    CGAL::parameters::
    point_map(Point_map())
    .normal_map(Normal_map())
    .plane_index_map(Plane_index_map()));

    std::cout << " Done. " << "Time: " << t.time() << " sec." << std::endl;

}*/