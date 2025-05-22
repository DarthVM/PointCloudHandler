#ifndef TYPEDEFS_H
#define TYPEDEFS_H
#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Polygon_mesh.h>
#include <CGAL/Plane_3.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <filesystem> // C++17
#include <unordered_set>

inline boost::uuids::random_generator uuid_generator = boost::uuids::random_generator();

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

using FT = Kernel::FT;
using Point = Kernel::Point_3;
using Vector = Kernel::Vector_3;

using Mesh = CGAL::Surface_mesh<Point>;

using vertex_descriptor = boost::graph_traits<Mesh>::vertex_descriptor;
using face_descriptor = boost::graph_traits<Mesh>::face_descriptor;

using PointVectorPair = std::pair<Point, Vector>;
using PV = std::tuple<Point, Vector>;
using PI = std::pair<Point, std::string>;
using PNI = std::tuple<Point, Vector, int>;

using Points = std::vector<Point>;
using Point_vector = std::vector<PI>;
using Points_normals = std::vector<PV>;

using K = CGAL::Simple_cartesian<double>;


/*
typedef CGAL::Nth_of_tuple_property_map<0, PNI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI> Plane_index_map;
*/

using Point_map = CGAL::First_of_pair_property_map<PI>;
using Id_map = CGAL::Second_of_pair_property_map<PI>;


using Plane = Kernel::Plane_3;

using Vertices = std::vector<vertex_descriptor>;
using Vertex_set = std::unordered_set<vertex_descriptor>;
using Face_set = std::unordered_set<face_descriptor>;


using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Mesh>;
using Region_type = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, Mesh>;
using Sorting = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, Mesh, Neighbor_query>;
using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

using assignedType = std::vector<typename Region_growing::Primitive_and_region>;
using unassignedType = std::vector<typename Region_growing::Item>;



namespace fs = std::filesystem;


class MeshProperty {
public:
    inline static std::string vertex_plane_id = "v:plane_id";
    inline static std::string face_centroid = "f:centroid";
    inline static std::string face_normal = "f:normal";
    inline static std::string face_area = "f:area";
    inline static std::string face_plane_id = "f:plane_id";
    inline static std::string face_color = "f:color";
};


#endif //TYPEDEFS_H
