#include "../../../Headers/Primitives/PointCloud.h"
#include "../../../Headers/Primitives/Model.h"


#include <CGAL/Triangulation_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <cassert>
#include <vector>

typedef CGAL::Delaunay_triangulation_3<Kernel> Delaunay;
typedef CGAL::Triangulation_3<Kernel> Triangulation;

Mesh PointCloud::triangulate() {
    std::vector<Point> points_local;

    for (std::size_t i = 0; i < points.size(); ++i) {
        Point p = std::get<0>(points[i]);
        points_local.push_back(p);
    }
    Delaunay triangulation(points_local.begin(), points_local.end());

    Mesh triangulated_cloud;
    /// ????

    return triangulated_cloud;
}