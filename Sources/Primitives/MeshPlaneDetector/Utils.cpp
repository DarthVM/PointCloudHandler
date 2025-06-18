#include "../../../Headers/typedefs.h"
#include "../../../Headers/Primitives/Math.hpp"
#include "../../../Headers/Primitives/MeshPlaneDetector.h"


#include <CGAL/IO/Color.h>
#include <CGAL/Graphics_scene.h>


void MeshPlaneDetector::process_free_points(std::vector<Metaplane>& metaplanes) {

    /// 1. Compute STD \
    /// 2. Find vertices neighbors \
    /// 3. Compare neighbors. True - If vertex less than STD \
    /// 4.

    // auto vertex_color_map = mesh_->add_property_map<vertex_descriptor, CGAL::IO::Color>
    // ("v:color").first;
    size_t free_point_size = 0;
    for (auto face_id : free_faces) {
        auto hf = mesh_->halfedge(face_id);
        auto free_face_vertices = mesh_->vertices_around_face(hf);

        for (auto vertex_id : free_face_vertices) {
            if (vertex_plane_id_map[vertex_id].empty()) {
                auto point = mesh_->point(vertex_id);
                auto min_metaplane = min_dist_to_plane_id(metaplanes, point);
                free_point_size++;
                vertex_plane_id_map[vertex_id] = min_metaplane->id;
                min_metaplane->vertices.insert(vertex_id);

                // vertex_color_map[vertex_id] = face_color_map[fd];
            }
        }
    }
    std::cout << "Free points: " << free_point_size << std::endl;
}

void MeshPlaneDetector::process_free_faces(std::vector<Metaplane>& metaplanes) {
    volatile int a = 0;
    for (auto face_id : free_faces) {
        auto hf = mesh_->halfedge(face_id);
        auto face_vertices = mesh_->vertices_around_face(hf);
        a++;
        auto centroid = face_centroid_map[face_id];
        auto min_metaplane = min_dist_to_plane_id(metaplanes, centroid);

        face_plane_id_map[face_id] = min_metaplane->id;
        auto f = min_metaplane->faces.begin();
        face_descriptor fd(f->id());
        face_color_map[face_id] = face_color_map[fd];
    }
}


Metaplane* MeshPlaneDetector::min_dist_to_plane_id(const std::vector<Metaplane>& metaplanes, Point& point) {
    double min_dist = CGAL_IA_MAX_DOUBLE;

    static Metaplane* metaplane_ptr;

    for (auto metaplane : metaplanes) {
        metaplane.compute_points_std();

        auto plane = Plane(metaplane.centroid, metaplane.normal);
        auto dist = Math::distance_to_plane(plane, point);

        if (dist < metaplane.points_std && dist < min_dist) {
            min_dist = dist;
            metaplane_ptr = &metaplane;
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

void MeshPlaneDetector::set_faces_color(const Face_set& face_set) {
    CGAL::Random random(face_set.begin()->id());
    auto color = CGAL::get_random_color(random);

    for (auto faceId : face_set) {
        face_color_map[faceId] = color;
    }
}