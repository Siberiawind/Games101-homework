#pragma once

#include "triangle.h"

namespace rst
{
	enum class Buffers
	{
		Color = 1,
		Depth = 2
	};


	inline Buffers operator|(Buffers a, Buffers b)
	{
		return Buffers((int)a | (int)b);
	}

	inline Buffers operator&(Buffers a, Buffers b)
	{
		return Buffers((int)a & (int)b);
	}

	enum class Primitive
	{
		Line,
		Triangle,
	};

	struct pos_buf_id
	{
		int pos_id = 0;
	};

	struct ind_buf_id
	{
		int ind_id = 0;
	};

	struct col_buf_id
	{
		int col_id = 0;
	};

	class rasterizer
	{
	public:
		rasterizer(int w, int h);

		pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
		ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
		col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);

		void set_model(const Eigen::Matrix4f& m);
		void set_view(const Eigen::Matrix4f& v);
		void set_projection(const Eigen::Matrix4f& p);

		void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

		void clear(Buffers buf);

		void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, rst::col_buf_id col_buffer, Primitive type);

		std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

	private:
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
		void drawDDALine(Eigen::Vector3f begin, Eigen::Vector3f end);
		void drawBresenhamLine(Eigen::Vector3f begin, Eigen::Vector3f end);

		void rasterize_wireframe(const Triangle& t);
		void rasterize_triangle(const Triangle& t);

		int get_index(int x, int y);

		int get_next_id()
		{
			return next_id++;
		}

	private:
		Eigen::Matrix4f model;
		Eigen::Matrix4f view;
		Eigen::Matrix4f projection;

		std::map<int, std::vector<Eigen::Vector3f> > pos_buf;
		std::map<int, std::vector<Eigen::Vector3i> > ind_buf;
		std::map<int, std::vector<Eigen::Vector3f> > col_buf;

		std::vector<Eigen::Vector3f> frame_buf;

		std::vector<float> depth_buf;

		int width;
		int height;
		int next_id = 0;
	};
}