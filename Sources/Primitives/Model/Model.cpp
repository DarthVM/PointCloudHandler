#include "../../../Headers/Primitives/model.h"

#include <CGAL/Timer.h>

Model::Model() = default;

Model::Model(Mesh& mesh) : mesh_(mesh) {

}

Model::Model(const std::string& model_filepath) {
    fs::path filePath = model_filepath;
    std::string extension = filePath.extension().string();
    // supported types: .ply, .obj, .stl
    if(!CGAL::IO::read_polygon_mesh(model_filepath, mesh_))
    {
        std::cerr << "Invalid input file." << std::endl;
        throw std::runtime_error("Model File not found/can't be processed");
    }

}

Mesh Model::get_mesh() {
    return mesh_;
}

void Model::save(const std::string& output_filepath) {
    CGAL::Timer t;
    t.start();
    std::cout << "Saving model..." << std::flush;

    if (CGAL::IO::write_polygon_mesh(output_filepath, mesh_))
        std::cout << " Done. Saved to " << output_filepath << ". Time: " << t.time() << " sec." << std::endl;
    else {
        std::cerr << " Failed saving file." << std::endl;
    }
}

