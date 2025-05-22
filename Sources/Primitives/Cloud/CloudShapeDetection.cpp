#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/SCIP_mixed_integer_program_traits.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Region_growing/Point_set.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Timer.h>
#include <CGAL/property_map.h>

#include <boost/range/irange.hpp>

#include <cstdlib>
#include <filesystem> // C++17
#include <iostream>

#include "../../../Headers/Primitives/PointCloud.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with mean_normal, and plane index.

using Point_map_region_growing = CGAL::Compose_property_map<CGAL::Random_access_property_map<Point_vector>, Point_map>;
using Normal_map_region_growing = CGAL::Compose_property_map<CGAL::Random_access_property_map<Point_vector>, Normal_map>;

using Region_type =
        CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, std::size_t, Point_map_region_growing,
                                                                         Normal_map_region_growing>;
using Neighbor_query =
        CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel, std::size_t, Point_map_region_growing>;
using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Point_vector, Point_map, Normal_map>     Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits>             Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>                                                Plane;
typedef CGAL::Shape_detection::Point_to_shape_index_map<Traits>     Point_to_shape_index_map;


Point_vector PointCloud::Efficient_Ransac(const float &normal_angle, const float &probability,
                                          const size_t &shape_min_points, const float &epsilon,
                                          const float &cluster_epsilon) {
    // Set parameters for shape detection.
    Efficient_ransac::Parameters parameters;

    // Set probability to miss the largest primitive at each iteration.
    parameters.probability = FT(probability);

    // Detect shapes with at least 200 points.
    parameters.min_points = FT(shape_min_points);

    // Set maximum Euclidean distance between a point and a shape.
    parameters.epsilon = FT(epsilon);

    // Set maximum Euclidean distance between points to be clustered.
    parameters.cluster_epsilon = FT(cluster_epsilon);

    // Set maximum mean_normal deviation.
    // 0.9 < dot(surface_normal, point_normal);
    parameters.normal_threshold = FT(std::cos(normal_angle));

    CGAL::Timer t;
    t.start();
    // Shape detection
    Efficient_ransac ransac;
    ransac.set_input(points);
    ransac.add_shape_factory<Plane>();

    std::cout << "Extracting planes...";

    t.reset();

    ransac.detect(parameters);

    Efficient_ransac::Plane_range planes = ransac.planes();
    std::size_t num_planes = planes.size();

    std::cout << " Done. " << num_planes << " planes extracted. Time: " << t.time() << " sec." << std::endl;

    // Stores the plane index of each point as the third element of the tuple.
    Point_to_shape_index_map shape_index_map(points, planes);
    for (std::size_t i = 0; i < points.size(); ++i) {
        // Uses the get function from the property map that accesses the 3rd element of the tuple.
        int plane_index = get(shape_index_map, i);
        std::get<2>(points[i]) = plane_index;
    }

    return points;
}


void PointCloud::Region_growing_Shape_Detector(const float &max_accepted_angle, const float &search_sphere_radius,
                                       const float &max_distance_to_plane, const std::size_t &min_region_size)
{
    // Shape detection.
    CGAL::Timer t;
    // Default parameter values for the data file cube.pwn.

    const FT ssr = FT(search_sphere_radius);
    const FT mdp = FT(max_distance_to_plane);
    const FT maa = FT(max_accepted_angle);
    const std::size_t mrs = min_region_size;


    Point_map_region_growing point_map_rg(CGAL::make_random_access_property_map(points));
    Normal_map_region_growing normal_map_rg(CGAL::make_random_access_property_map(points));

    // Create instances of the classes Neighbor_query and Region_type.
    Neighbor_query neighbor_query(boost::irange<std::size_t>(0, points.size()),
                                  CGAL::parameters::sphere_radius(ssr).point_map(point_map_rg));

    Region_type region_type(CGAL::parameters::maximum_distance(mdp)
                                    .maximum_angle(maa)
                                    .minimum_region_size(mrs)
                                    .point_map(point_map_rg)
                                    .normal_map(normal_map_rg));

    // Create an instance of the region growing class.
    Region_growing region_growing(boost::irange<std::size_t>(0, points.size()), neighbor_query, region_type);

    std::cout << "Extracting planes...";
    std::vector<typename Region_growing::Primitive_and_region> regions;
    t.start();
    region_growing.detect(std::back_inserter(regions));
    std::cout << " Done. " << regions.size() << " planes extracted. Time: " << t.time() << " sec." << std::endl;
    t.stop();
    // Stores the plane index of each point as the third element of the tuple.
    for (std::size_t i = 0; i < points.size(); ++i)
        // Uses the get function from the property map that accesses the 3rd element of the tuple.
        std::get<2>(points[i]) = static_cast<int>(get(region_growing.region_map(), i));
}