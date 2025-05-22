#ifndef UTILITY_H
#define UTILITY_H

#include <CGAL/Timer.h>

struct Timeout_callback {

    mutable int nb;
    mutable CGAL::Timer timer;
    const double limit;

    Timeout_callback(double limit) :
    nb(0), limit(limit) {
        timer.start();
    }

    bool operator()(double advancement) const {

        // Avoid calling time() at every single iteration, which could
        // impact performances very badly.
        ++nb;
        if (nb % 1000 != 0)
            return true;

        // If the limit is reached, interrupt the algorithm.
        if (timer.time() > limit) {
            std::cerr << "Algorithm takes too long, exiting ("
                      << 100.0 * advancement << "% done)" << std::endl;
            return false;
        }
        return true;
    }
};

class Parameters {
public:
    float maximum_distance;
    float maximum_angle;
    int k_neighbors;
    int min_region_size;
    bool debug;
    bool verbose;
    int max_octree_depth;
    int max_octree_node_size;
    bool reorient;
    bool regparallel;
    bool regcoplanar;
    bool regorthogonal;
    bool regsymmetric;
    float angle_tolerance;
    float maximum_offset;
};



#endif //UTILITY_H
