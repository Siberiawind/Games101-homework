#pragma once

#include <eigen3/Eigen/Eigen>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;

	translate << 1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, 1, -eye_pos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
	Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();

	Eigen::Matrix3f innerproduct = Eigen::Matrix3f::Identity();
	innerproduct << axis.x() * axis.x(), axis.x()* axis.y(), axis.x()* axis.z(),
		axis.y()* axis.x(), axis.y()* axis.y(), axis.y()* axis.z(),
		axis.z()* axis.x(), axis.z()* axis.y(), axis.z()* axis.z();

	innerproduct *= (1 - cos(angle));

	Eigen::Matrix3f crossproduct = Eigen::Matrix3f::Identity();

	crossproduct << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(), 0;

	crossproduct *= sin(angle);

	rotationMatrix << cos(angle) + innerproduct(0) + crossproduct(0), innerproduct(1) + crossproduct(1), innerproduct(2) + crossproduct(2), 0,
		innerproduct(3) + crossproduct(3), cos(angle) + innerproduct(4) + crossproduct(4), innerproduct(5) + crossproduct(5), 0,
		innerproduct(6) + crossproduct(6), innerproduct(7) + crossproduct(7), cos(angle) + innerproduct(8) + crossproduct(8), 0,
		0, 0, 0, 1;

	return rotationMatrix;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f rotate;

	float arc = rotation_angle / 180.0 * MY_PI;

	rotate = get_rotation(Vector3f({ 0, 0, 1 }), arc);

	model = rotate * model;

	return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();

	P2O << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zFar * zNear,
		0, 0, 1, 0;

	float halfEyeAngleRadian = eye_fov / 2.0 / 180.0 * MY_PI;

	float t = zNear * tan(halfEyeAngleRadian);

	float b = -t;

	float r = t * aspect_ratio;

	float l = -r;

	Eigen::Matrix4f ortho1 = Eigen::Matrix4f::Identity();

	ortho1 << 2 / (r - l), 0, 0, 0,
		0, 2 / (t - b), 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;

	Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Identity();

	ortho2 << 1, 0, 0, -(r + l) / 2,
		0, 1, 0, -(t + b) / 2,
		0, 0, 1, -(zNear + zFar) / 2,
		0, 0, 0, 1;

	projection = ortho1 * ortho2 * P2O;

	return projection;
}

