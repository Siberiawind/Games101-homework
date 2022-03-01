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

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width
		|| point.y() < 0 || point.y() >= height)
	{
		return;
	}

	// 转换坐标系坐标（原点在左下角）到矩阵坐标（原点在左上角）
	auto ind = (height - point.y()) * width + point.x();
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

static inline auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
	if (type != rst::Primitive::Triangle)
	{
		throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
	}

	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];

	float f1 = (100 - 0.1) / 2.0;
	float f2 = (100 + 0.1) / 2.0;

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

		t.setColor(0, 255.0, 0.0, 0.0);
		t.setColor(1, 0.0, 255.0, 0.0);
		t.setColor(2, 0.0, 0.0, 255.0);

		rasterize_wireframe(t);
	}
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
	draw_line(t.c(), t.a());
	draw_line(t.a(), t.b());
	draw_line(t.b(), t.c());
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
	return (height - y) * width + x;
}

