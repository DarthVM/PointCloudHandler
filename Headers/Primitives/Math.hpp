#ifndef MATH_HPP
#define MATH_HPP

#include "../typedefs.h"

#define PRECISION_EPSILON 0.1

namespace Math {
    static double cosine_between(const Vector& v1, const Vector& v2) {
        double dot_product = v1 * v2;
        double magnitude_v1 = std::sqrt(v1 * v1);
        double magnitude_v2 = std::sqrt(v2 * v2);
        return dot_product / (magnitude_v1 * magnitude_v2);
    }

    static bool angle_less_than(Vector& v1, Vector& v2, double max_angle) {

        double local_cos = cosine_between(v1, v2);

        double abs = std::abs(local_cos - std::cos(max_angle * (M_PI / 180)));
        if (abs < PRECISION_EPSILON) {
            return true;
        }

        return false;
    }

    static Point face_centroid(Mesh& mesh, face_descriptor& face) {
        auto halfedge = mesh.halfedge(face);
        auto vertices = mesh.vertices_around_face(halfedge);

        double sum_x = .0, sum_y = .0, sum_z = .0;

        for (auto vertex : vertices) {
            Point point = mesh.point(vertex);
            sum_x += point.x();
            sum_y += point.y();
            sum_z += point.z();
        }

        return {sum_x / 3, sum_y / 3, sum_z / 3};
    }

    static double distance_to_plane(Plane& plane, Point& p) {
        return std::sqrt(CGAL::squared_distance(p, plane));
    }

    static double weight(double area = 1) { return area; }

    static double points_to_plane_dist_std(Mesh& mesh, Plane& plane, Vertex_set& vertices) {
        const auto size = vertices.size();
        double distance[size];
        double sum = 0;
        int i = 0;

        for (auto vertex : vertices) {
            auto point = mesh.point(vertex);
            distance[i] = distance_to_plane(plane, point);
            sum += distance[i++];
        }

        auto mean = sum / size;
        double sigma_sqr;

        for (i = 0; i <= size; i++)
            sum += (distance[i] - mean) * (distance[i] - mean);

        sigma_sqr = sum / (size-1);

        return sqrt(sigma_sqr);
    }

}
#endif
