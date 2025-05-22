#ifndef MODEL_H
#define MODEL_H
#pragma once

#include "../typedefs.h"
#include "Metaplane.h"


class Model {
private:
    Mesh mesh_;
public:
    Model();
    Model(Mesh& mesh);
    Model(const std::string& model_filepath);
    Mesh get_mesh();
    void save(const std::string& output_filepath);
    std::pair<assignedType, unassignedType> regionGrowingShapeDetection(
            const float epsilon = 0.024,
            const float max_angle = 1,
            const int min_size = 50
            );
    std::vector<Metaplane> formMetaplanes(assignedType& regions);
private:
    void estimatePolygonsNormals();

};




#endif
