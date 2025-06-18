#ifndef METAPLANE_HPP
#define METAPLANE_HPP

#include "../typedefs.h"

class Metaplane {
public:
    std::string id;
    Vertex_set vertices;
    Face_set faces;
    Point centroid;
    Vector normal;
    double area;
    double points_std;
    CGAL::IO::Color color;

    Metaplane(Mesh& mesh, Face_set& faces);
    Metaplane() = default;
    friend std::ostream& operator<<(std::ostream& os, const Metaplane& metaplane);
    double compute_points_std();
    void calculate_area();
private:
    Mesh *mesh_;

    void init_indices();
    void init_area();
    void init_normal();
    void extract_vertices();
};
#endif
