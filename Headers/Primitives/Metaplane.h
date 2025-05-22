#ifndef METAPLANE_HPP
#define METAPLANE_HPP

#include "../typedefs.h"

class Metaplane {
public:
    std::string id;
    Face_set faces;
    Point centroid;
    Vector mean_normal;
    float area;
    std::string color;

    Metaplane(Mesh& mesh, Face_set& faces);
    Plane get_plane() const;
    void print_info() const;
    double compute_points_std();
    double get_points_std();
    void add_free_vertex(vertex_descriptor& vertex);
private:
    Mesh *mesh_;
    Vertices vertices_;
    Vertices free_vertices_;
    double points_std;


};
#endif
