#include "../../../Headers/Primitives/Model.h"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/remesh_planar_patches.h>

#include <boost/graph/visitors.hpp>
#include <CGAL/boost/graph/iterator.h>


void Model::estimatePolygonsNormals() {
    CGAL::Timer t;
    t.start();

    std::cout << "Estimating facets normals..." << std::flush;

    auto fnormals = mesh_.add_property_map<face_descriptor, Vector>("f:normal", CGAL::NULL_VECTOR).first;

    CGAL::Polygon_mesh_processing::compute_face_normals(mesh_, fnormals);

    t.stop();

    std::cout << " Done. Time: " << t.time() << " sec" << std::endl;
}



