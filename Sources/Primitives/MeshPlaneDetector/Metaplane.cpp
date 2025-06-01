#include "../../../Headers/Primitives/Metaplane.h"
#include "../../../Headers/Primitives/Math.hpp"

#include <CGAL/Polygon_mesh_processing/measure.h>

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

Metaplane::Metaplane(Mesh& mesh, Face_set& faces) : mesh_(&mesh), faces(faces) {

    auto uuid = uuid_generator();
    auto uuidString = boost::lexical_cast<std::string>(uuid);
    this->id = uuidString;

    init_plane_ids();
    init_plane_area();
    init_plane_normal();

};

void Metaplane::print_info() const {
    std::cout
    << "\nPlane id: " << id
    << "\n * faces: " << faces.size()
    << "\n * normal: " << mean_normal
    << "\n * area: " << area
    << "\n * color: " << "not supported"
    << std::endl;
}

Plane Metaplane::get_plane() const { return Plane(centroid, mean_normal); }

double Metaplane::compute_points_std() {
    for (auto face : faces) {
        auto he = mesh_->halfedge(face);
        auto face_vertices = CGAL::vertices_around_face(he, *mesh_);

        for (auto vertex : face_vertices)
            vertices.push_back(vertex);
    }

    auto plane = get_plane();
    points_std = Math::points_to_plane_dist_std(*mesh_, plane, vertices);

    return points_std;
}

void Metaplane::add_free_vertex(vertex_descriptor& vertex) {
    free_vertices_.push_back(vertex);
}


void Metaplane::init_plane_ids() {
    auto vertex_plane_id_map = mesh_->property_map<vertex_descriptor, std::string>
            (MeshProperty::vertex_plane_id).value();
    auto face_plane_id_map = mesh_->property_map<face_descriptor, std::string>
            (MeshProperty::face_plane_id).value();

    for (auto faceId : faces) {
        face_plane_id_map[faceId] = this->id;
        auto halfedge = mesh_->halfedge(faceId);
        auto face_vertices = mesh_->vertices_around_face(halfedge);

        for (auto face_vertex_id : face_vertices) {
            vertex_plane_id_map[face_vertex_id] = this->id;
        }
    }
}

void Metaplane::init_plane_area() {
    auto face_area_map = mesh_->property_map<face_descriptor, double>
           (MeshProperty::face_area).value();
    double sum_area = .0;

    for (auto faceId : faces) {
        face_area_map[faceId] = CGAL::Polygon_mesh_processing::face_area(faceId, *mesh_);
        sum_area += face_area_map[faceId];
    }

    area = sum_area;
}

void Metaplane::init_plane_normal() {
    auto face_normal_map = mesh_->property_map<face_descriptor, Vector>
        (MeshProperty::face_normal).value();


    Vector sum_normals;

    for (auto faceId : faces) {
        sum_normals += face_normal_map[faceId];
    }

    mean_normal = sum_normals / sum_normals.squared_length();
}