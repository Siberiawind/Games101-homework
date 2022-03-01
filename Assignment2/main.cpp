#include "triangle.h"
#include "rasterizer.h"
#include "transform.h"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
	float angle = 0.0f;
	bool command_line = false;
	std::string filename = "output.png";

	if (argc >= 3)
	{
		angle = std::stof(argv[2]);
		if (argc == 4)
		{
			filename = std::string(argv[3]);
		}
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = { 0, 0, 5 };

	std::vector<Eigen::Vector3f> pos{ {2,0,-2}, {0,2,-2}, {-2, 0, -2} };

	std::vector<Eigen::Vector3i> ind{ {0,1,2} };

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	char key = 0;
	int frame_count = 0;
	
	if (command_line)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(60, 0.1, 0.2, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());

		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());

		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imshow("image", image);

		key = cv::waitKey(0);

		std::cout << "key:" << key << " frame count: " << frame_count++ << "\n";

		if (key == 'a')
		{
			angle += 10;
		}
		else if (key == 'd')
		{
			angle -= 10;
		}
	}

	return 0;
}