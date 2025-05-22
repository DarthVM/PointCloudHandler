#ifndef API_H
#define API_H

#pragma once

#include "Primitives/Model.h"
#include "Primitives/PointCloud.h"

class Api {
private:
	Model input_model_;
	PointCloud input_cloud_;

	Model triangulated_cloud_;
public:
	Api(const std::string& model_filepath, const std::string& cloud_filepath);
	void perform_Model_Overlay();
};

#endif
