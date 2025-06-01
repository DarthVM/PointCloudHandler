#include "../../../Headers/typedefs.h"
#include "../../../Headers/Primitives/Math.hpp"
#include "../../../Headers/Primitives/MeshPlaneDetector.h"


#include <CGAL/IO/Color.h>
#include <CGAL/Graphics_scene.h>


Metaplane MeshPlaneDetector::min_dist_to_plane_id(const std::vector<Metaplane>& metaplanes, Point& point) {
    double min_dist = CGAL_IA_MAX_DOUBLE;

    Metaplane metaplane_ptr;

    for (auto metaplane : metaplanes) {
        metaplane.compute_points_std();

        auto plane = metaplane.get_plane();
        auto dist = Math::distance_to_plane(plane, point);

        if (dist < metaplane.points_std && dist < min_dist) {
            min_dist = dist;
            metaplane_ptr = metaplane;
        }
    }


    return metaplane_ptr;
}


void MeshPlaneDetector::erase_from_free_faces(Face_set& result) {
    for (auto face_id : result) {
        free_faces.erase(face_id);
    }
}

face_descriptor MeshPlaneDetector::get_middle_element(const Face_set& faces) {

    // Calculate the middle index
    size_t middleIndex = faces.size() / 2;

    // Create an iterator to traverse the unordered_set
    auto it = faces.begin();

    // Advance the iterator to the middle index
    for (size_t i = 0; i < middleIndex; ++i) {
        ++it; // Move to the next element
    }


    return *it; // Dereference the iterator to get the middle element
}

void MeshPlaneDetector::set_plane_color(const Face_set& face_set) {
    CGAL::Random random(face_set.begin()->id());
    auto color = CGAL::get_random_color(random);

    for (auto faceId : face_set) {
        face_color_map[faceId] = color;
    }
}
