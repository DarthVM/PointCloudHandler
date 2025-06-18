#ifndef MESHPLANEDETECTOR_H
#define MESHPLANEDETECTOR_H

#include "../typedefs.h"
#include "Metaplane.h"

class MeshPlaneDetector {
private:
    Mesh *mesh_;

    Mesh::Property_map<vertex_descriptor, std::string> vertex_plane_id_map;

    Mesh::Property_map<face_descriptor, Point> face_centroid_map;
    Mesh::Property_map<face_descriptor, Vector> face_normal_map;
    Mesh::Property_map<face_descriptor, double> face_area_map;
    Mesh::Property_map<face_descriptor, std::string> face_plane_id_map;
    Mesh::Property_map<face_descriptor, CGAL::IO::Color> face_color_map;

    Face_set free_faces;
    Vertex_set free_vertices;

    void erase_from_free_faces(Face_set& result);

    static Metaplane* min_dist_to_plane_id(const std::vector<Metaplane>& metaplanes, Point& point);

    Face_set conditional_bfs(const face_descriptor& start_face, Face_set& face_set, double max_angle);

    void process_free_points(std::vector<Metaplane>& metaplanes);
    void process_free_faces(std::vector<Metaplane>& metaplanes);
public:
    explicit MeshPlaneDetector(Mesh &mesh);
    std::vector<Metaplane> detect(size_t min_region_size, double max_angle);
    void process_free_objects(std::vector<Metaplane>& metaplanes);
    Mesh retriangulate(std::vector<Metaplane>& metaplanes);
    void set_faces_color(const Face_set& face_set);
    static face_descriptor get_middle_element(const Face_set& faces);
};
#endif