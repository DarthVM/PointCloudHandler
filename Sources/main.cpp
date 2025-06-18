#include "../Headers/Primitives/Model.h"
#include "../Headers/Primitives/PointCloud.h"
#include "../Headers/Primitives/MeshPlaneDetector.h"

#include <CGAL/Timer.h>

#include <iostream>

int main(int argc, char* argv[]) {

    const std::string cloud = "C:/Users/xdoyo/Desktop/TestData/cube.xyz";
    const std::string pre_output = "C:/Users/xdoyo/Desktop/TestData/big_metaplanes_test_2.ply";
    const std::string post_output = "C:/Users/xdoyo/Desktop/TestData/Points/big_retriangulation.ply";

    CGAL::Timer t;
    t.start();

    PointCloud input_cloud(cloud);
    Mesh triangulated_cloud = input_cloud.reconstructFront();

    MeshPlaneDetector plane_detector(triangulated_cloud);

    auto metaplanes = plane_detector.detect(20, 1);
    std::cout << metaplanes.size() << std::endl;
    Model::save(triangulated_cloud, pre_output);

    plane_detector.process_free_objects(metaplanes);

    auto total_mesh = plane_detector.retriangulate(metaplanes);

    MeshPlaneDetector plane_detector2(total_mesh);
    plane_detector2.detect(20, 1);

    Model::save(total_mesh, post_output);

    // Model final_mesh(triangulated_cloud);
    // final_mesh.save(post_output);

    t.stop();

    std::cout << "Done. Total time: ";
    std::cout << static_cast<int>(t.time() / 60) << " min\t"  << static_cast<int>(t.time()) % 60 << " sec:\t" << std::endl;

    return 0;
}