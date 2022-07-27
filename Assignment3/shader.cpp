#include "shader.h"

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
	return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.0f;
	Eigen::Vector3f result;
	result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
	return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
	auto costheta = vec.dot(axis);
	return (2 * costheta * axis - vec).normalized();
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f return_color = { 0, 0, 0 };
	if (payload.texture)
	{
		//TODO: Get the texture value at the texture coordinates of the current fragment
		return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
	}

	Eigen::Vector3f texture_color;
	texture_color << return_color.x(), return_color.y(), return_color.z();

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = texture_color / 255.f;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = texture_color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };

	Eigen::Vector3f view_dir = (eye_pos - point).normalized();

	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the "ambient", "diffuse", and "specular"
		//	components are. Then, accumulate that result on the "result_color" object
		float rr = (light.position - point).squaredNorm();

		Eigen::Vector3f diffuse(0, 0, 0);
		Eigen::Vector3f specular(0, 0, 0);
		Eigen::Vector3f ambient(0, 0, 0);

		Eigen::Vector3f light_dir = (light.position - point).normalized();

		for (size_t i = 0; i < 3; i++)
		{
			Eigen::Vector3f h = (view_dir + light_dir).normalized();	// half
			float intensity = light.intensity[i] / rr;
			diffuse[i] = kd[i] * intensity * std::max(0.0f, normal.dot(light_dir));
			specular[i] = ks[i] * intensity * std::pow(std::max(0.0f, normal.dot(h)), p);
			ambient[i] = ka[i] * amb_light_intensity[i];
		}

		result_color += diffuse;
		result_color += specular;
		result_color += ambient;
	}

	return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };


	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };

	Eigen::Vector3f view_dir = (eye_pos - point).normalized();

	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the "ambient", "diffuse", and "specular"
		//	components are. Then, accumulate that result on the "result_color" object
		float rr = (light.position - point).squaredNorm();
		Eigen::Vector3f diffuse(0, 0, 0);
		Eigen::Vector3f specular(0, 0, 0);
		Eigen::Vector3f ambient(0, 0, 0);
		Eigen::Vector3f light_dir = (light.position - point).normalized();

		for (size_t i = 0; i < 3; i++)
		{
			Eigen::Vector3f h = (view_dir + light_dir).normalized();
			float intensity = light.intensity[i] / rr;
			diffuse[i] = kd[i] * intensity * std::max(0.0f, normal.dot(light_dir));
			specular[i] = ks[i] * intensity * std::pow(std::max(0.0f, normal.dot(h)), p);
			ambient[i] = ka[i] * amb_light_intensity[i];
		}

		result_color += diffuse;
		result_color += specular;
		result_color += ambient;
	}
	return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };


	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	float kh = 0.2, kn = 0.1;

	Eigen::Vector3f result_color = { 0, 0, 0 };

	Eigen::Vector3f view_dir = (eye_pos - point).normalized();

	// TODO: Implement displacement mapping here
	// Let n = normal = (x, y, z)
	// Vector t = (x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z))
	// Vector b = n cross_product t
	// Matrix TBN = [t b n]
	// dU = kh * kn * (h(u+1/w, v) - h(u, v))
	// dV = kh * kn * (h(u, v+1/h) - h(u, v))
	// Vector ln = (-dU, -dV, 1)
	// Position p = p + kn * n * h(u,v)
	// Normal n = normalize(TBN * ln)

	Eigen::Vector3f n = normal;
	Eigen::Vector3f t;
	t << n.x() * n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z()),
		std::sqrt(n.x() * n.x() + n.z() * n.z()),
		n.z()* n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z());

	Eigen::Vector3f b = n.cross(t);
	Eigen::Matrix3f TBN;

	TBN << t, b, n;

	// finite difference
	float u = payload.tex_coords[0], v = payload.tex_coords[1];
	float w = payload.texture->width;
	float h = payload.texture->height;

	// w: width of texture
	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
	// h: height of texture
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

	Eigen::Vector3f ln = { -dU, -dV, 1.0f };

	// displacement
	point += kn * n * payload.texture->getColor(u, v).norm();
	normal = (TBN * ln).normalized();

	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the "ambient", "diffuse", and "specular"
		//	components are. Then, accumulate that result on the "result_color" object
		float rr = (light.position - point).squaredNorm();
		Eigen::Vector3f diffuse(0, 0, 0);
		Eigen::Vector3f specular(0, 0, 0);
		Eigen::Vector3f ambient(0, 0, 0);
		Eigen::Vector3f light_dir = (light.position - point).normalized();

		for (size_t i = 0; i < 3; i++)
		{
			Eigen::Vector3f h = (view_dir + light_dir).normalized();
			float intensity = light.intensity[i] / rr;
			diffuse[i] = kd[i] * intensity * std::max(0.0f, normal.dot(light_dir));
			specular[i] = ks[i] * intensity * std::pow(std::max(0.0f, normal.dot(h)), p);
			ambient[i] = amb_light_intensity[i] * ka[i];
		}

		result_color += diffuse;
		result_color += specular;
		result_color += ambient;
	}
	return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	float kh = 0.2, kn = 0.1;

	// TODO: Implement bump mapping here
	// Let n = normal = (x, y, z)
	// Vector t = (x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z))
	// Vector b = n cross_product t
	// Matrix TBN = [t b n]
	// dU = kh * kn * (h(u+1/w, v) - h(u, v))
	// dV = kh * kn * (h(u, v+1/h) - h(u, v))
	// Vector ln = (-dU, -dV, 1)
	// Position p = p + kn * n * h(u,v)
	// Normal n = normalize(TBN * ln)

	Eigen::Vector3f n = normal;
	Eigen::Vector3f t;
	t << n.x() * n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z()),
		std::sqrt(n.x() * n.x() + n.z() * n.z()),
		n.z()* n.y() / std::sqrt(n.x() * n.x() + n.z() * n.z());

	Eigen::Vector3f b = n.cross(t);
	Eigen::Matrix3f TBN;

	TBN << t, b, n;

	// finite difference
	float u = payload.tex_coords(0), v = payload.tex_coords(1);
	float w = payload.texture->width;
	float h = payload.texture->height;

	// w: width of texture
	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
	// h: height of texture
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

	Eigen::Vector3f ln = { -dU, -dV, 1.0f };

	normal = (TBN * ln).normalized();

	Eigen::Vector3f result_color = { 0, 0, 0 };

	Eigen::Vector3f view_dir = (eye_pos - point).normalized();

	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the "ambient", "diffuse", and "specular"
		//	components are. Then, accumulate that result on the "result_color" object
		float rr = (light.position - point).squaredNorm();
		Eigen::Vector3f diffuse(0, 0, 0);
		Eigen::Vector3f specular(0, 0, 0);
		Eigen::Vector3f ambient(0, 0, 0);
		Eigen::Vector3f light_dir = (light.position - point).normalized();

		for (size_t i = 0; i < 3; i++)
		{
			Eigen::Vector3f h = (view_dir + light_dir).normalized();
			float intensity = light.intensity[i] / rr;
			diffuse[i] = kd[i] * intensity * std::max(0.0f, normal.dot(light_dir));
			specular[i] = ks[i] * intensity * std::pow(std::max(0.0f, normal.dot(h)), p);
			ambient[i] = amb_light_intensity[i] * ka[i];
		}

		result_color += diffuse;
		result_color += specular;
		result_color += ambient;
	}
	return result_color * 255.f;
}
