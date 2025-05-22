#include "../../../Headers/typedefs.h"
#include "../../../Headers/Primitives/MeshPlaneDetector.h"


#include <CGAL/IO/Color.h>
#include <CGAL/Graphics_scene.h>


void MeshPlaneDetector::set_plane_ids(std::vector<Metaplane>& metaplanes, Face_set &faces) {
    for (auto faceId : faces) {
        face_plane_id_map[faceId]

    }
}


void MeshPlaneDetector::erase_from_free_faces(Face_set& result) {
    auto id_begin = face_descriptor(result.begin()->id());
    auto id_end = face_descriptor(result.end()->id());

    auto un_begin = free_faces.find(id_begin);
    auto un_end = free_faces.find(id_end);

    free_faces.erase(un_begin, un_end);
}

template <typename T>
T MeshPlaneDetector::get_middle_element(const std::unordered_set<T>& s) {

    // Calculate the middle index
    size_t middleIndex = s.size() / 2;

    // Create an iterator to traverse the unordered_set
    auto it = s.begin();

    // Advance the iterator to the middle index
    for (size_t i = 0; i < middleIndex; ++i) {
        ++it; // Move to the next element
    }

    return *it; // Dereference the iterator to get the middle element
}

void MeshPlaneDetector::set_plane_color(Face_set& face_set) {
    CGAL::Random random(face_set.begin()->id());
    auto color = CGAL::get_random_color(random);

    for (auto faceId : face_set) {
        face_color_map[faceId] = color;
    }
}