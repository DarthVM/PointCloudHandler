#include "../Headers/Primitives/Model.h"
#include "../Headers/Primitives/PointCloud.h"
#include "../Headers/Primitives/MeshPlaneDetector.h"

#include <CGAL/Timer.h>


#include <iostream>

int main(int argc, char* argv[]) {

    const std::string cloud = "C:/Users/xdoyo/Desktop/TestData/cube.xyz";

    const std::string output = "C:/Users/xdoyo/Desktop/TestData/post_metaplanes_test.ply";
    CGAL::Timer t;
    t.start();


    PointCloud input_cloud(cloud);

    Mesh triangulated_cloud = input_cloud.reconstructFront();

    MeshPlaneDetector plane_detector(triangulated_cloud);
    auto metaplanes = plane_detector.detect(5, 1);

    plane_detector.post_process(metaplanes);

    Model final_mesh(triangulated_cloud);
    final_mesh.save(output);

    t.stop();

    std::cout << "Done. Total time: ";
    std::cout << static_cast<int>(t.time() / 60) << "\tmin\t"  << static_cast<int>(t.time()) % 60 << "\tsec:\t" << std::endl;


    return 0;
}