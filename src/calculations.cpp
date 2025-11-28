#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>
#include <optional>

#include <Eigen/Dense>

#include "render.h"
#include "calculations.h"
#include "arm_attributes.h"
#include "get_path.h"

//for math constants
#define _USE_MATH_DEFINES


using Eigen::MatrixXd;
using namespace std;

//random seeding
unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
std::mt19937 gen(seed);

float l1 = 5.0; //cm
float l2 = 10.0; //cm
float l3 = 5.0; //cm

//Forward kinematics: joint angles to cartesian
//Formula: x = Ax + d, where A is rotation matrix, d is translation vector
vector<tuple<float, float, float>> fk(float theta1, float theta2, float theta3) {
	Eigen::Vector3d link1(0, l1, 0);
	Eigen::Vector3d link2(l2, 0,0);
	Eigen::Vector3d link3(l3 , 0,0);

	Eigen::Vector3d target_vec = { 0.0, 0.0, 0.0 };

	//Rotation transformation matrices
	//Rotation about Y axis
	MatrixXd rot1(3, 3);
	rot1 << cos(theta1), 0, sin(theta1),
		0, 1, 0,
		-sin(theta1), 0, cos(theta1);

	// Rotation about Z
	MatrixXd rot2(3, 3);
	rot2 << cos(theta2), -sin(theta2),0,
		sin(theta2), cos(theta2), 0,
		0, 0, 1;

	// Rotation about Z
	MatrixXd rot3(3, 3);
	rot3 << cos(theta3), -sin(theta3), 0,
		sin(theta3), cos(theta3), 0,
		0, 0, 1;

	//defining joint positons after transformations
	Eigen::Vector3d link1_end = rot1 * link1;
	Eigen::Vector3d link2_end = rot1 * rot2 * link2 + link1_end;
	Eigen::Vector3d link3_end = rot1 * rot2 * rot3 * link3 + link2_end;

	return { {link1_end(0), link1_end(1), link1_end(2)},  {link2_end(0), link2_end(1), link2_end(2)},
		{link3_end(0), link3_end(1), link3_end(2)}};
}

void fk_test(float t1, float t2, float t3) {
	tuple<float, float, float> tar_chords = fk(t1, t2, t3).back();
	cout << "Target angles = ( " << t1 << ", " << t2 << ", " << t3 << ", " << " )" << endl;
	cout << "(x, y, z) = " << "(" << get<0>(tar_chords) << ", " << get<1>(tar_chords) << ", " << get<2>(tar_chords) << ")" << endl;
}

bool target_reached(float x, float y, float z, float t1, float t2, float t3) {
	tuple<float, float, float> tar = fk(t1, t2, t3).back();
	Eigen::Vector3d tar_vec = { get<0>(tar), get<1>(tar), get<2>(tar) };
	float tolerance = 1.0f;
	cout << "Given point: (" << x << " ," << y << ", " << z << " ,), found (" << t1 << ", " << t2 << " " << t3 << ").\n";
	cout << "This will reach: " << tar_vec(0) << ", " << tar_vec(1) << ", " << tar_vec(2) << ")\n";
	if (fabs(tar_vec(0) - x) < tolerance && fabs(tar_vec(1) - y) < tolerance && fabs(tar_vec(2) - z) < tolerance) {
		return true;
	}
	return false;
}

//Inverse kinematics: cartesian to joint angles
std::tuple<float, float, float> ik(float x, float y, float z) {
	//first we check if a solution exists:
	if (sqrt(x * x + y * y + z * z) > l1 + l2 + l3) {
		cout << "No solution found. Outside workspace.\n";
		return { 0,0,0 };
	}
	//angle away from XY plane
	float theta = atan2(z, x);

	//move down by length of first link
	Eigen::Vector3d p_target(x, y-l1, z);

	MatrixXd proj_max(3, 3);
	proj_max << cos(theta), 0, sin(theta),
		0, 1, 0,
		-sin(theta), 0, cos(theta);

	//projected point 
	Eigen::Vector3d p_proj = proj_max * p_target;

	cout << "Projected point at: ("<<p_proj(0) << " " << p_proj(1) << " " << p_proj(2) << ")\n";

	float x1 = p_proj(0), y1 = p_proj(1), r1 = sqrt(x1*x1 + y1 * y1);
	//angle from base to projected point
	float phi = atan2(y1, x1);

	float cos_theta2 = (l2 * l2 + r1 * r1 - l3 * l3) / (2 * l2 * r1);
	cos_theta2 = min(1.0f, max(-1.0f, cos_theta2));
	
	float cos_theta3 = (l2 * l2 + l3 * l3 - r1 * r1) / (2 * l2 * l3);
	cos_theta3 = min(1.0f, max(-1.0f, cos_theta3));


	float theta2 = phi - acos(cos_theta2);
	float theta3 = EIGEN_PI - acos(cos_theta3);
	float theta1 = -theta;


	//case 1: theta 1 and theta 2 >0
	if (target_reached(x, y, z, theta1, theta2, theta3)) return std::make_tuple(theta1, theta2, theta3);

	//case 2: 1 > 0, 2 < 0
	if (target_reached(x, y, z, theta1, -theta2, theta3)) return std::make_tuple(theta1, -theta2, theta3);
	
	cout << "Target is unreachable.\n";
	return make_tuple(0.0f,0.0f,0.0f);
}

vector<float> simulate_angles(float theta1, float theta2, float theta3) {
	vector<float> vertices;

	//fetch joint positions
	vector<tuple<float, float, float>> end_verts = fk(theta1, theta2, theta3);

	//write to vertices
	tuple<float, float, float> l1 = end_verts[0];
	tuple<float, float, float> l2 = end_verts[1];
	tuple<float, float, float> l3 = end_verts[2];

	float x1 = get<0>(l1), y1 = get<1>(l1), z1 = get<2>(l1);
	float x2 = get<0>(l2), y2 = get<1>(l2), z2 = get<2>(l2);
	float x3 = get<0>(l3), y3 = get<1>(l3), z3 = get<2>(l3);

	// line from origin to end of link 1
	vertices.push_back(0.0f); vertices.push_back(0.0f); vertices.push_back(0.0f);
	vertices.push_back(1.0f); vertices.push_back(1.0f); vertices.push_back(1.0f);		//color
	vertices.push_back(x1); vertices.push_back(y1); vertices.push_back(z1);
	vertices.push_back(1.0f); vertices.push_back(1.0f); vertices.push_back(1.0f);		//color


	// line from end of link1 to end of link 2
	vertices.push_back(x1); vertices.push_back(y1); vertices.push_back(z1);
	vertices.push_back(1.0f); vertices.push_back(1.0f); vertices.push_back(1.0f);		//color
	vertices.push_back(x2); vertices.push_back(y2); vertices.push_back(z2);
	vertices.push_back(1.0f); vertices.push_back(1.0f); vertices.push_back(1.0f);		//color

	// line from end of link2 to end of link 3
	vertices.push_back(x2); vertices.push_back(y2); vertices.push_back(z2);
	vertices.push_back(1.0f); vertices.push_back(1.0f); vertices.push_back(1.0f);		//color
	vertices.push_back(x3); vertices.push_back(y3); vertices.push_back(z3);
	vertices.push_back(1.0f); vertices.push_back(1.0f); vertices.push_back(1.0f);		//color

	return vertices;
}
