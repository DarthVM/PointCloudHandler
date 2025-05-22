#ifndef SCENE_HPP
#define SCENE_HPP

#include "../typedefs.h"
#include "Metaplane.h"
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/draw_surface_mesh.h>

struct Colored_planes:
        public CGAL::Graphics_scene_options<Mesh,
                typename Mesh::Vertex_index,
                typename Mesh::Edge_index,
                typename Mesh::Face_index>
{
    std::vector<Metaplane> metaplanes_;


    Colored_planes(Mesh& sm, std::vector<Metaplane> metaplanes) : metaplanes_(metaplanes) {}

    bool colored_face(const Mesh &, typename Mesh::Face_index) const
    {
        return true;
    };

    CGAL::IO::Color face_color(const Mesh& sm, typename Mesh::Face_index fi) const
    {

        std::size_t planeId;
        for (auto metaplane : metaplanes_) {
            auto result = std::find(metaplane.plane.begin(), metaplane.plane.end(), fi);

            if (result != metaplane.plane.end()) {
                planeId = result->id();

                CGAL::Random random(planeId);

                return CGAL::get_random_color(random);
            }
        }

        return CGAL::IO::gray();
    };
};

#endif