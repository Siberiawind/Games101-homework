#pragma once

#include <eigen3/Eigen/Eigen>

using namespace Eigen;

class Triangle
{
public:
	Triangle();

	Eigen::Vector4f a() const { return v[0]; }
	Eigen::Vector4f b() const { return v[1]; }
	Eigen::Vector4f c() const { return v[2]; }

	void setVertex(int ind, Vector4f ver);	/* set i-th vertex coordinates */
	void setNormal(int ind, Vector3f n);	/* set i-th vertex normal vector */
	void setColor(int ind, float r, float g, float b); /* set i-th vertex color */
	Eigen::Vector3f getColor() const { return color[0] * 255; }
	void setTexCoord(int ind, Vector2f uv);	/* set i-th vertex texture coordinate */
	std::array<Vector4f, 3> toVector4() const;

	Vector4f v[3];	/* the original coordinates of the triangle, v0, v1, v2 in counter clockwise order */
	Vector3f color[3];	/* color at each vectex*/
	Vector2f tex_coords[3];	/* texture u, v */
	Vector3f normal[3];	/* normal vector for each vectex */

};