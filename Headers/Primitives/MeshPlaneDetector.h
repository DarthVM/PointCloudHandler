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
    void post_process_free_points(std::vector<Metaplane>& metaplanes);
    Face_set conditional_bfs(face_descriptor& start_face, Face_set& face_set, double max_angle);
public:
    MeshPlaneDetector(Mesh &mesh);
    std::vector<Metaplane> detect(double min_region_size, double max_angle);
    void post_process(std::vector<Metaplane>& metaplanes);
    void set_plane_color(Face_set& face_set);
    template <typename T>
    T get_middle_element(const std::unordered_set<T>& s);
};


#endif