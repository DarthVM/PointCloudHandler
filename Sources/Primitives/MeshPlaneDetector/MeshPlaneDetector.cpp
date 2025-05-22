#include "../../../Headers/typedefs.h"
#include "../../../Headers/Primitives/MeshPlaneDetector.h"
#include "../../../Headers/Primitives/Math.hpp"


#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Timer.h>
#include <CGAL/squared_distance_3.h>


Face_set MeshPlaneDetector::conditional_bfs(face_descriptor& start_face, Face_set& face_set, double max_angle) {
    Face_set result;

    // BFS
    std::queue<face_descriptor> q;

    // Mark source node as visited and enqueue it
    q.push(start_face);

    // Erase from non-processed faces
    face_set.erase(start_face);
    // Iterate over the queue
    while (!q.empty()) {

        // Dequeue a vertex from queue and store it
        auto main_face = q.front();
        result.insert(main_face);
        q.pop();

        auto halfedge = mesh_->halfedge(main_face);
        auto neighbors = mesh_->faces_around_face(halfedge);

        for (auto neighbor : neighbors) {
            if (neighbor == mesh_->null_face()) continue;
            bool visited = face_set.find(neighbor) == face_set.end(); // face is not found -> it's been visited before
            bool conditions_are_satisfied = Math::angle_less_than(face_normal_map[start_face], face_normal_map[neighbor], max_angle);

            if (!visited && conditions_are_satisfied) {
                face_set.erase(neighbor);
                q.push(neighbor);
            }
        }
    }

    return result;
}

std::vector<Metaplane> MeshPlaneDetector::detect(double min_region_size, double max_angle) {
    std::cout << "Detecting metaplanes..." << std::flush;
    CGAL::Timer t;
    t.start();


    auto faces = mesh_->faces();
    Face_set face_set;

    for (auto face : faces) {
        free_faces.insert(face);
        face_set.insert(face);
    }

    std::vector<Metaplane> metaplanes;

    while(!face_set.empty()) {
        face_descriptor start_face = get_middle_element(face_set);

        // Conditional BFS
        auto result = conditional_bfs(start_face, face_set, max_angle);

        if (result.size() >= min_region_size) {
            erase_from_free_faces(result);
            set_plane_color(result);

            Metaplane metaplane(*mesh_, result);
            metaplane.print_info();


            metaplanes.push_back(metaplane);
        }

    }
    t.stop();
    std::cout << " Done. Time: " << t.time() << std::endl;

    return metaplanes;
}

void MeshPlaneDetector::post_process(std::vector<Metaplane>& metaplanes) {
    std::cout << "Post-processing metaplanes..." << std::flush;

    CGAL::Timer t;
    t.start();



    /// 1. Compute STD \
    /// 2. Find vertices neighbors \
    /// 3. Compare neighbors. True - If vertex less than STD \
    /// 4.
    for (auto face : free_faces) {
        auto he = mesh_->halfedge(face);
        auto vertices = CGAL::vertices_around_face(he, *mesh_);

        for (auto vertex : vertices)
            free_vertices.push_back(vertex);
    }
    free_vertices

    for (auto metaplane : metaplanes) {
        (void)metaplane.compute_points_std();
    }


    for (auto metaplane : metaplanes) {
        metaplane.compute_points_std();
    }





    t.stop();
    std::cout << " Done. Time: " << t.time() << std::endl;
}

void MeshPlaneDetector::post_process_free_points(std::vector<Metaplane>& metaplanes) {

}

MeshPlaneDetector::MeshPlaneDetector(Mesh &mesh) : mesh_(&mesh) {
    vertex_plane_id_map = mesh.add_property_map<vertex_descriptor, std::string>
    (MeshProperty::vertex_plane_id).first;

    face_centroid_map = mesh_->add_property_map<face_descriptor, Point>
            (MeshProperty::face_centroid).first;

    face_normal_map = mesh_->add_property_map<face_descriptor, Vector>
    (MeshProperty::face_normal, CGAL::NULL_VECTOR).first;

    face_area_map = mesh_->add_property_map<face_descriptor, double>
    (MeshProperty::face_area).first;

    face_plane_id_map = mesh_->add_property_map<face_descriptor, std::string>
    (MeshProperty::face_plane_id).first;

    face_color_map = mesh_->add_property_map<face_descriptor, CGAL::IO::Color>
    (MeshProperty::face_color, CGAL::IO::Color(34, 34, 34)).first;

    auto faces = mesh_->faces();

    CGAL::Polygon_mesh_processing::compute_face_normals(*mesh_, face_normal_map);

    for (auto faceId : faces) {
        face_centroid_map[faceId] = Math::face_centroid(*mesh_, faceId);
        face_area_map[faceId] = CGAL::Polygon_mesh_processing::face_area(faceId, *mesh_);
    }
}