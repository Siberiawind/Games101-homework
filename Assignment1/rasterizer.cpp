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

Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Eigen::Vector3f* _v)
{
	//Implement this function to check if the point (x,y) is inside the triangle represented by _v[0], _v[1], _v[2]

	Eigen::Vector3f P(x + 0.5f, y + 0.5f, 1.0f);

	const Eigen::Vector3f& A = _v[0];
	const Eigen::Vector3f& B = _v[1];
	const Eigen::Vector3f& C = _v[2];

	Eigen::Vector3f AB = B - A;
	Eigen::Vector3f BC = C - B;
	Eigen::Vector3f CA = A - C;

	Eigen::Vector3f AP = P - A;
	Eigen::Vector3f BP = P - B;
	Eigen::Vector3f CP = P - C;

	float z1 = AB.cross(AP).z();
	float z2 = BC.cross(BP).z();
	float z3 = CA.cross(CP).z();

	return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

static std::tuple<float, float, float> computeBaryCentric2D(float x, float y, const Eigen::Vector3f * v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());

	return { c1, c2, c3 };
}

static void getBBox(Eigen::Vector4f* vecs, const std::array<Vector4f, 3>& v)
{
	int x1, x2, y1, y2;
	x1 = v[0].x();
	x2 = v[1].x();
	y1 = v[0].y();
	y2 = v[1].y();

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

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
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

		Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
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

			Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
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

		Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
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

			Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
			set_pixel(point, line_color);
		}
	}
}

void rst::rasterizer::drawBresenhamLine(Eigen::Vector3f begin, Eigen::Vector3f end)
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

		Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
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

			Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
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

		Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
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

			Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
			set_pixel(point, line_color);
		}
	}
}

void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
	drawBresenhamLine(begin, end);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::col_buf_id col_buffer, rst::Primitive type)
{
	if (type != rst::Primitive::Triangle)
	{
		throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
	}

	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;

	for (auto& i : ind)
	{
		Triangle t;
		Eigen::Vector4f v[] = {
			mvp * to_vec4(buf[i[0]], 1.0f),
			mvp * to_vec4(buf[i[1]], 1.0f),
			mvp * to_vec4(buf[i[2]], 1.0f)
		};

		for (auto& vec : v)
		{
			std::cout << "vec:" << vec << "w:" << vec.w() << std::endl;
			vec /= vec.w();
		}

		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; i++)
		{
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
	draw_line(t.c(), t.a());
	draw_line(t.a(), t.b());
	draw_line(t.b(), t.c());
}

// screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
	auto v = t.toVector4();
	
	// Find out the bounding box of current triangle
	Eigen::Vector4f bboxes[2];
	getBBox(bboxes, v);

	// Iterate through the pixel and find if the current pixel is inside the triangle
	for (int y = bboxes[0].y(); y < bboxes[1].y(); y++)
	{
		for (int x = bboxes[0].x(); x < bboxes[1].x(); x++)
		{
			if (!insideTriangle(x, y, t.v))
			{
				continue;
			}

			// Get the interpolated z value
			auto[alpha, beta, gamma] = computeBaryCentric2D(x, y, t.v);

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

			set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
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
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

