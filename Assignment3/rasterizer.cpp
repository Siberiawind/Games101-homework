#include "rasterizer.h"
#include "opencv2/opencv.hpp"
#include <iostream>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	int id = get_next_id();
	pos_buf.emplace(id, positions);
	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	int id = get_next_id();
	ind_buf.emplace(id, indices);
	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& colors)
{
	int id = get_next_id();
	col_buf.emplace(id, colors);
	return { id };
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
	int id = get_next_id();
	nor_buf.emplace(id, normals);
	normal_id = id;
	return { id };
}

Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Eigen::Vector4f* _v)
{
	//Implement this function to check if the point (x,y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Vector3f v[3];
	for (int i = 0; i < 3; i++)
	{
		v[i] = { _v[i].x(), _v[i].y(), 1.0 };
	}

	Vector3f f0, f1, f2;
	f0 = v[1].cross(v[0]);
	f1 = v[2].cross(v[1]);
	f2 = v[0].cross(v[2]);

	Vector3f p(x, y, 1.);
	if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
	{
		return true;
	}

	return false;
}

// FIXME: the returned centroid coordinates are infinite!
static std::tuple<float, float, float> computeBaryCentric2D(float x, float y, const Eigen::Vector4f* v)
{
	double a1 = x * (v[1].y() - v[2].y());
	double a2 = (v[2].x() - v[1].x()) * y;
	double a3 = v[1].x() * v[2].y() - v[2].x() * v[1].y();
	double b1 = v[0].x() * (v[1].y() - v[2].y());
	double b2 = (v[2].x() - v[1].x()) * v[0].y();
	double b3 = v[1].x() * v[2].y() - v[2].x() * v[1].y();
	float c1 = (a1 + a2 + a3) / (b1 + b2 + b3);

	a1 = x * (v[2].y() - v[0].y());
	a2 = (v[0].x() - v[2].x()) * y;
	a3 = v[2].x() * v[0].y() - v[0].x() * v[2].y();
	b1 = v[1].x() * (v[2].y() - v[0].y());
	b2 = (v[0].x() - v[2].x()) * v[1].y();
	b3 = v[2].x() * v[0].y() - v[0].x() * v[2].y();
	float c2 = (a1 + a2 + a3) / (b1 + b2 + b3);

	a1 = x * (v[0].y() - v[1].y());
	a2 = (v[1].x() - v[0].x()) * y;
	a3 = v[0].x() * v[1].y() - v[1].x() * v[0].y();
	b1 = v[2].x() * (v[0].y() - v[1].y());
	b2 = (v[1].x() - v[0].x()) * v[2].y();
	b3 = v[0].x() * v[1].y() - v[1].x() * v[0].y();
	float c3 = (a1 + a2 + a3) / (b1 + b2 + b3);

	return { c1, c2, c3 };
}

static void getBBox(Eigen::Vector4f* vecs, const std::array<Vector4f, 3>& v)
{
	int x1, x2, y1, y2;
	x1 = v[0].x();
	x2 = v[1].x();
	y1 = v[0].y();
	y2 = v[1].y();

	// retrieve the min and max value in vertex.x(), y()
	for (auto& vertex : v)
	{
		x1 = vertex.x() > x1 ? x1 : vertex.x();
		y1 = vertex.y() > y1 ? y1 : vertex.y();
		x2 = vertex.x() < x2 ? x2 : vertex.x();
		y2 = vertex.y() < y2 ? y2 : vertex.y();
	}

	vecs[0] << x1, y1, v[0].z(), v[0].w();
	vecs[1] << x2, y2, v[0].z(), v[0].w();
}

void rst::rasterizer::set_pixel(const Eigen::Vector2i& point, const Eigen::Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width
		|| point.y() < 0 || point.y() >= height)
	{
		return;
	}

	// 转换坐标系坐标（原点在左下角）到矩阵坐标（原点在左上角）
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;
}

void rst::rasterizer::drawDDALine(Eigen::Vector3f begin, Eigen::Vector3f end)
{
	auto x1 = begin.x();
	auto y1 = begin.y();
	auto x2 = end.x();
	auto y2 = end.y();

	Eigen::Vector3f line_color = { 0, 255, 255 };

	int dx, dy;
	int dx_abs, dy_abs;
	int px, py;
	int x_end, y_end;
	int x, y;

	dx = x2 - x1;
	dy = y2 - y1;
	dx_abs = fabs(dx);
	dy_abs = fabs(dy);
	px = 2 * dy_abs - dx_abs;
	py = 2 * dx_abs - dy_abs;

	if (dy_abs <= dx_abs)
	{
		if (dx >= 0)
		{
			x = x1;
			y = y1;
			x_end = x2;
		}
		else
		{
			x = x2;
			y = y2;
			x_end = x1;
		}

		Eigen::Vector2i point = Eigen::Vector2i(x, y);
		set_pixel(point, line_color);

		while (x < x_end)
		{
			x++;
			if (px < 0)
			{
				px = px + 2 * dy_abs;
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
				{
					y++;
				}
				else
				{
					y--;
				}

				px = px + 2 * (dy_abs - dx_abs);
			}

			Eigen::Vector2i point = Eigen::Vector2i(x, y);
			set_pixel(point, line_color);
		}
	}
	else
	{
		if (dy >= 0)
		{
			x = x1;
			y = y1;
			y_end = y2;
		}
		else
		{
			x = x2;
			y = y2;
			y_end = y1;
		}

		Eigen::Vector2i point = Eigen::Vector2i(x, y);
		set_pixel(point, line_color);

		while (y < y_end)
		{
			y++;
			if (py <= 0)
			{
				py = py + 2 * dx_abs;
			}
			else
			{
				if ((dx < 0 && dy < 0)
					|| (dx > 0 && dy > 0))
				{
					x++;
				}
				else
				{
					x--;
				}
				py = py + 2 * (dx_abs - dy_abs);
			}

			Eigen::Vector2i point = Eigen::Vector2i(x, y);
			set_pixel(point, line_color);
		}
	}
}

void rst::rasterizer::drawBresenhamLine(Eigen::Vector4f begin, Eigen::Vector4f end)
{
	auto x1 = begin.x();
	auto y1 = begin.y();
	auto x2 = end.x();
	auto y2 = end.y();

	Eigen::Vector3f line_color = { 0, 255, 255 };

	int dx, dy;
	int dx_abs, dy_abs;
	int px, py;
	int x_end, y_end;
	int x, y;

	dx = x2 - x1;
	dy = y2 - y1;
	dx_abs = fabs(dx);
	dy_abs = fabs(dy);
	px = 2 * dy_abs - dx_abs;
	py = 2 * dx_abs - dy_abs;

	if (dy_abs <= dx_abs)
	{
		if (dx >= 0)
		{
			x = x1;
			y = y1;
			x_end = x2;
		}
		else
		{
			x = x2;
			y = y2;
			x_end = x1;
		}

		Eigen::Vector2i point = Eigen::Vector2i(x, y);
		set_pixel(point, line_color);

		while (x < x_end)
		{
			x++;
			if (px < 0)
			{
				px = px + 2 * dy_abs;
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
				{
					y++;
				}
				else
				{
					y--;
				}

				px = px + 2 * (dy_abs - dx_abs);
			}

			Eigen::Vector2i point = Eigen::Vector2i(x, y);
			set_pixel(point, line_color);
		}
	}
	else
	{
		if (dy >= 0)
		{
			x = x1;
			y = y1;
			y_end = y2;
		}
		else
		{
			x = x2;
			y = y2;
			y_end = y1;
		}

		Eigen::Vector2i point = Eigen::Vector2i(x, y);
		set_pixel(point, line_color);

		while (y < y_end)
		{
			y++;
			if (py <= 0)
			{
				py = py + 2 * dx_abs;
			}
			else
			{
				if ((dx < 0 && dy < 0)
					|| (dx > 0 && dy > 0))
				{
					x++;
				}
				else
				{
					x--;
				}
				py = py + 2 * (dx_abs - dy_abs);
			}

			Eigen::Vector2i point = Eigen::Vector2i(x, y);
			set_pixel(point, line_color);
		}
	}
}

void rst::rasterizer::draw_line(Eigen::Vector4f begin, Eigen::Vector4f end)
{
	drawBresenhamLine(begin, end);
}

void rst::rasterizer::draw(std::vector<Triangle*>& TriangleList)
{
	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;

	for (const auto& t : TriangleList)
	{
		Triangle newtri = *t;

		std::array<Eigen::Vector4f, 3> mm{
			(view * model * t->v[0]),
			(view * model * t->v[1]),
			(view * model * t->v[2])
		};

		std::array<Eigen::Vector3f, 3> viewspace_pos;

		std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
			return v.template head<3>();
		});

		Eigen::Vector4f v[] = {
			mvp * t->v[0],
			mvp * t->v[1],
			mvp * t->v[2]
		};

		// Homogeneous division
		for (auto& vec : v)
		{
			vec.x() /= vec.w();
			vec.y() /= vec.w();
			vec.z() /= vec.w();
		}

		Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
		Eigen::Vector4f n[] = {
			inv_trans * to_vec4(t->normal[0], 0.0f),
			inv_trans * to_vec4(t->normal[1], 0.0f),
			inv_trans * to_vec4(t->normal[2], 0.0f)
		};

		// Viewport transformation
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; i++)
		{
			//screen space coordinates
			newtri.setVertex(i, v[i]);
		}

		for (int i = 0; i < 3; i++)
		{
			//view space normal
			newtri.setNormal(i, n[i].head<3>());
		}

		newtri.setColor(0, 148, 121.0, 92.0);
		newtri.setColor(1, 148, 121.0, 92.0);
		newtri.setColor(2, 148, 121.0, 92.0);

		// Also pass view space vertex position
		rasterize_triangle(newtri, viewspace_pos);
	}
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
	draw_line(t.c(), t.a());
	draw_line(t.a(), t.b());
	draw_line(t.b(), t.c());
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
	auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
	auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

	u /= weight;
	v /= weight;
	
	return Eigen::Vector2f(u, v);
}

// screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos)
{
	auto v = t.toVector4();

	// Find out the bounding box of current triangle
	Eigen::Vector4f bboxes[2];
	getBBox(bboxes, v);

	// Iterate through the pixel and find if the current pixel is inside the triangle
	for (int y = bboxes[0].y(); y <= bboxes[1].y(); y++)
	{
		for (int x = bboxes[0].x(); x <= bboxes[1].x(); x++)
		{
			if (!insideTriangle(x, y, t.v))
			{
				continue;
			}

			// Get the interpolated z value
			auto [alpha, beta, gamma] = computeBaryCentric2D(x, y, t.v);

			float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
			float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();

			z_interpolated *= w_reciprocal;

			int buf_index = get_index(x, y);

			// Only record the closest depth value
			if (z_interpolated >= depth_buf[buf_index])
			{
				continue;
			}

			depth_buf[buf_index] = z_interpolated;

			//TODO 
			auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1);
			auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1);
			auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);
			auto interpolated_viewpos = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);

			fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
			payload.view_pos = interpolated_viewpos;
		
			auto pixel_color = fragment_shader(payload);
			set_pixel(Eigen::Vector2i(x, y), pixel_color);
		}
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f(0, 0, 0));
	}

	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}


void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
	vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
	fragment_shader = frag_shader;
}