#include "../../../Headers/typedefs.h"
#include "../../../Headers/Primitives/Model.h"


#include <CGAL/Timer.h>

std::pair<assignedType, unassignedType> Model::regionGrowingShapeDetection(
        const float epsilon, const float max_angle, const int min_size) {
    CGAL::Timer t;
    t.start();

    std::cout << "Detecting Shapes..." << std::flush;
    const auto& face_range = faces(mesh_);

    // Default parameter values for the data file building.off.
    const FT max_distance = FT(epsilon);
    const FT angle = FT(max_angle);
    const std::size_t min_region_size = min_size;

    // Create instances of the classes Neighbor_query and Region_type.
    Neighbor_query neighbor_query(mesh_);

    Region_type region_type(
            mesh_,
            CGAL::parameters::
                    maximum_distance(max_distance).
            maximum_angle(angle).
            minimum_region_size(min_region_size));

    // Sort face indices.
    Sorting sorting(
            mesh_, neighbor_query);
    sorting.sort();

    // Create an instance of the region growing class.
    Region_growing region_growing(
            face_range, sorting.ordered(), neighbor_query, region_type);

    // Run the algorithm.
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    t.stop();
    std::cout << " Done. Time: " << t.time() << " sec" << std::endl;
    std::cout << "* number of found planes: " << regions.size() << std::endl;

    const Region_growing::Region_map& map = region_growing.region_map();

    for (std::size_t i = 0; i < regions.size(); i++)
        for (auto& item : regions[i].second) {
            if (i != get(map, item)) {
                std::cout << "Region map incorrect" << std::endl;
            }
        }

    std::vector<typename Region_growing::Item> unassigned;
    region_growing.unassigned_items(face_range, std::back_inserter(unassigned));

    for (auto& item : unassigned) {
        if (std::size_t(-1) != get(map, item)) {
            std::cout << "Region map for unassigned incorrect" << std::endl;
        }
    }

    return std::make_pair(regions, unassigned);
}