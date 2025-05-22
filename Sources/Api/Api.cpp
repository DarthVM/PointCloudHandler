#include "../../Headers/Api.h"

Api::Api(const std::string &model_filepath, const std::string &cloud_filepath) :
    input_model_(model_filepath), input_cloud_(cloud_filepath) {}

