#pragma once

#include <eigen3/Eigen/Eigen>
#include "texture.h"

struct fragment_shader_payload
{
	fragment_shader_payload()
	{
		texture = nullptr;
	}

	fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,
		const Eigen::Vector2f& tc, Texture* tex) : color(col), normal(nor), tex_coords(tc), texture(tex) {}

	Eigen::Vector3f view_pos;
	Eigen::Vector3f color;
	Eigen::Vector3f normal;
	Eigen::Vector2f tex_coords;

	Texture* texture;
};

struct vertex_shader_payload
{
	Eigen::Vector3f position;
};

struct light
{
	Eigen::Vector3f position;
	Eigen::Vector3f intensity;
};

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload);

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload);

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload);

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload);

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload);

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload);