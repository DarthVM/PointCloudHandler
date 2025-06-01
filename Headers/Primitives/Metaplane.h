#ifndef METAPLANE_HPP
#define METAPLANE_HPP

#include "../typedefs.h"

class Metaplane {
public:
    std::string id;
    Vertices vertices;
    Face_set faces;
    Point centroid;
    Vector mean_normal;
    double area;
    double points_std;
    std::string color;

    Metaplane(Mesh& mesh, Face_set& faces);
    Metaplane() = default;
    Plane get_plane() const;
    void print_info() const;
    double compute_points_std();
    void add_free_vertex(vertex_descriptor& vertex);
    void init_plane_ids();
    void init_plane_area();
    void init_plane_normal();
private:
    Mesh *mesh_;
    Vertices free_vertices_;
};
#endif
