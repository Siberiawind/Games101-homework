#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "triangle.h"
#include "rasterizer.h"
#include "transform.h"
#include "obj_loader.h"
#include "shader.h"
#include "texture.h"

int main(int argc, char* argv[])
{
	std::vector<Triangle*> triangleList;

	float angle = 180.0f;
	bool command_line = false;
	std::string filename = "output.png";

	obj1::Loader loader;
	std::string obj_path = "models/spot/";

	// Load .obj file
	bool loadout = loader.LoadFile("models/spot/spot_triangulated_good.obj");
	if (!loadout)
	{
		fprintf(stderr, "Unable to load spot file!\n");
	}

	for (auto mesh : loader.LoadedMeshes)
	{
		for (int i = 0; i < mesh.Vertices.size(); i+=3)
		{
			Triangle* t = new Triangle();
			for (int j = 0; j < 3; j++)
			{
				t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
				t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
				t->setTexCoord(j, Vector2f(std::clamp(mesh.Vertices[i + j].TextureCoordinate.X, 0.f ,1.f), std::clamp(mesh.Vertices[i + j].TextureCoordinate.Y, 0.f, 1.f)));
			}

			triangleList.push_back(t);
		}
	}
	rst::rasterizer r(700, 700);

	auto texture_path = "hmap.jpg";
	r.set_texture(Texture(obj_path + texture_path));

	std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

	if (argc >= 2)
	{
		command_line = true;
		filename = std::string(argv[1]);
		
		if (argc == 3 && std::string(argv[2]) == "texture")
		{
			std::cout << "Rasterizing using the texture shader\n";
			active_shader = texture_fragment_shader;
			texture_path = "spot_texture_conv.png";
			r.set_texture(Texture(obj_path + texture_path));
		}
		else if (argc == 3 && std::string(argv[2]) == "normal")
		{
			std::cout << "Rasterizing using the normal shader\n";
			active_shader = normal_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "phong")
		{
			std::cout << "Rasterizing using the phong shader\n";
			active_shader = phong_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "bump")
		{
			std::cout << "Rasterizing using the bump shader\n";
			active_shader = bump_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "displacement")
		{
			std::cout << "Rasterizing using the displacement shader\n";
			active_shader = displacement_fragment_shader;
		}
	}

	r.set_vertex_shader(vertex_shader);
	r.set_fragment_shader(active_shader);

	Eigen::Vector3f eye_pos = { 0, 0, 10 };

	char key = 0;
	int frame_count = 0;

	if (command_line)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		r.draw(triangleList);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(0);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		r.draw(triangleList);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);

		key = cv::waitKey(10);
		std::cout << "key:" << key << " frame count: " << frame_count++ << "\n";

		if (key == 'a')
		{
			angle -= 10.0;
		}
		else if (key == 'd')
		{
			angle += 10.0;
		}
	}

	for (auto t : triangleList)
	{
		delete t;
		t = nullptr;
	}

	return 0;
}