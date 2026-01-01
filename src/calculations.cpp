#include <cmath>
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <optional>
#include <algorithm>
#include <Eigen/Dense>
#include "render.h"
#include "calculations.h"
#include "arm_attributes.h"
#include "get_path.h"


using Eigen::MatrixXd, Eigen::Vector3d;
using namespace std;

Vector3d link1(0, l1, 0);
Vector3d link2(l2, 0, 0);
Vector3d link3(l3, 0, 0);

Eigen::Vector3d link4 = { l4, 0,0, };
Eigen::Vector3d link5(0, l5, 0);
Eigen::Vector3d link5_1(l5, 0, 0);
Eigen::Vector3d link6(l6, 0, 0);

//Forward kinematics: joint angles to cartesian
vector<Vector3d> fk(float t1, float t2, float t3, float t4, float t5, float t6) {

	//Rotation transformation matrices
	//Rotation about Y axis
	MatrixXd rot1(3, 3);
	rot1 << cos(t1), 0, sin(t1),
		0, 1, 0,
		-sin(t1), 0, cos(t1);

	// Rotation about Z
	MatrixXd rot2(3, 3);
	rot2 << cos(t2), -sin(t2), 0,
		sin(t2), cos(t2), 0,
		0, 0, 1;

	// Rotation about Z
	MatrixXd rot3(3, 3);
	rot3 << cos(t3), -sin(t3), 0,
		sin(t3), cos(t3), 0,
		0, 0, 1;

	//roll about X
	MatrixXd rot4(3, 3);
	rot4 << 1, 0, 0,
		0, cos(t4), -sin(t4),
		0, sin(t4), cos(t4);

	//roll about Y
	MatrixXd rot5(3, 3);
	rot5 << cos(t5), 0, sin(t5),
		0, 1, 0,
		-sin(t5), 0, cos(t5);

	//pitch about Z
	MatrixXd  rot6(3, 3);
	rot6 << cos(t6), -sin(t6), 0,
		sin(t6), cos(t6), 0,
		0, 0, 1;

	//defining joint positons after transformations
	Eigen::Vector3d link1_end = rot1 * link1;
	Eigen::Vector3d link2_end = rot1 * rot2 * link2 + link1_end;
	Eigen::Vector3d link3_end = rot1 * rot2 * rot3 * link3 + link2_end;

	Eigen::Vector3d link4_end = rot4 * link4 + link3_end;
	Eigen::Vector3d link5_end = rot4 * rot5 * link5 + link4_end;

	Eigen::Vector3d link5_1 = -l5 * (Eigen::Vector3d{ 0.0,0.0,1.0 }.cross(link5_end - link4_end).normalized());
	Eigen::Vector3d link5_1_end = link5_1 + link5_end;
	Eigen::Vector3d link5_2 = -l5 * (Eigen::Vector3d{ 0.0,0.0,1.0 }.cross(link5_1).normalized());
	Eigen::Vector3d link5_2_end = link5_2 + link5_1_end;
	Eigen::Vector3d link6_end = rot4 * rot5 * rot6 * link6 + link5_2_end;

	return { link1_end, link2_end, link3_end, link4_end, link5_end, link5_1_end, link5_2_end, link6_end };
}
static bool target_reached(float x, float y, float z, float t1, float t2, float t3) {
	Eigen::Vector3d tar_vec = fk(t1, t2, t3, 0, 0, 0).back();

	float tolerance = 3.f;		//3 cm tolerance
	cout << "Given point: (" << x << " ," << y << ", " << z << " ,), found (" << t1 << ", " << t2 << " " << t3 << ").\n";
	cout << "This will reach: " << tar_vec(0) << ", " << tar_vec(1) << ", " << tar_vec(2) << ")\n";

	cout << "Difference: (" << fabs(tar_vec(0) - x) << ", " << fabs(tar_vec(1) - y) << ", " << fabs(tar_vec(2) - z) << ")\n";
	cout << " Tolerance: " << tolerance << "\n";

	if ((fabs(tar_vec(0) - x) < tolerance) && (fabs(tar_vec(1) - y) < tolerance) && (fabs(tar_vec(2) - z) < tolerance)) {
		return true;
	}
	else return false;
}

//Inverse kinematics: cartesian to joint angles
Eigen::Vector3d ik(float x, float y, float z) {
	Eigen::Vector3d p_target = Eigen::Vector3d(x, y, z);

	p_target = p_target - link1 - link4 - link5_1 - link6;

	//angle away from XY plane
	float t = atan2(p_target.z(), p_target.x());

	//rotate on the y axis to project onto XY plane
	MatrixXd proj_max(3, 3);
	proj_max << cos(t), 0, sin(t),
		0, 1, 0,
		-sin(t), 0, cos(t);

	//projected point 
	Eigen::Vector3d p_proj = proj_max * p_target;

	//defining new coordinates 
	float x1 = p_proj.x(), y1 = p_proj.y();
	
	float r1 = sqrt(x1 * x1 + y1 * y1);			//new length
	float phi = atan2(y1, x1);					//angle from base to projected point

	//angle from the new point using cosine law
	float cos_t2 = (l2 * l2 + r1 * r1 - (l3) * (l3)) / (2.f * l2 * r1);
	cos_t2 = min(1.0f, max(-1.0f, cos_t2));						//constraints

	float cos_t3 = (l2 * l2 + (l3) * (l3)-r1 * r1) / (2.f * l2 * (l3));
	cos_t3 = min(1.0f, max(-1.0f, cos_t3));						//constraints

	float t2 = phi - acos(cos_t2);
	float t3 = PI - acos(cos_t3);
	float t1 = -t;

	std::cout << "[ik] x,y,z: " << x << "," << y << "," << z
		<< "  r1:" << r1
		<< "  cos_t2:" << cos_t2 << " cos_t3:" << cos_t3
		<< "  t1:" << t1 << " t2:" << t2 << " t3:" << t3 << std::endl;

	Eigen::Vector3d tar_vec = fk(t1, t2, t3, 0, 0, 0).back();

	cout << "Given point: (" << x << " ," << y << ", " << z << " ,), found (" << t1 << ", " << t2 << " " << t3 << ").\n";
	cout << "This will reach: " << tar_vec(0) << ", " << tar_vec(1) << ", " << tar_vec(2) << ")\n";

	return Eigen::Vector3d(t1, t2, t3);

	/*
	//case 1: t 1 and t 2 >0
	if (target_reached(x, y, z, t1, t2, t3)) {
		cout << "Target is reachable.\n";
		cout << "Returning angles: (" << t1 << ", " << -t2 << ", " << t3 << ")\n";
		return Eigen::Vector3d(t1, t2, t3);
	}
	//case 2: 1 > 0, 2 < 0
	else if (target_reached(x, y, z, t1, -t2, t3)) {
		cout << "Target is reachable.\n";
		cout << "Returning angles: (" << t1 << ", " << -t2 << ", " << t3 << ")\n";
		return Eigen::Vector3d(t1, -t2, t3);
	}

	else {
		cout << "Target is unreachable.\n";
		return Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	}
	*/
}

Eigen::Vector3d wristIk(float x, float y, float z, float roll, float pitch, float yaw) {

	//getting desired rotation matrices
	//roll about X
	MatrixXd R_roll(3, 3);
	R_roll << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);

	//Y
	MatrixXd R_yaw(3, 3);
	R_yaw << cos(yaw), 0, sin(yaw),
		0, 1, 0,
		-sin(yaw), 0, cos(yaw);

	//pitch about Z
	MatrixXd R_pitch(3, 3);
	R_pitch << cos(pitch), -sin(pitch), 0,
		sin(pitch), cos(pitch), 0,
		0, 0, 1;

	Eigen::Vector3d wrist_angles = ik(x, y, z);
	float theta1 = wrist_angles(0); float theta2 = wrist_angles(1); float theta3 = wrist_angles(2);

	//Y
	MatrixXd rot1(3, 3);
	rot1 << cos(theta1), 0, sin(theta1),
		0, 1, 0,
		-sin(theta1), 0, cos(theta1);

	// Z
	MatrixXd rot2(3, 3);
	rot2 << cos(theta2), -sin(theta2), 0,
		sin(theta2), cos(theta2), 0,
		0, 0, 1;

	// Z
	MatrixXd rot3(3, 3);
	rot3 << cos(theta3), -sin(theta3), 0,
		sin(theta3), cos(theta3), 0,
		0, 0, 1;

	MatrixXd targetRot = rot1 * rot2 * rot3 * R_roll * R_pitch * R_yaw;     // full FK pose
	MatrixXd goalRot = (rot1 * rot2 * rot3).transpose() * targetRot; // relative wrist rotation

	cout << "\n ==========Wrist IK Results==========\n";
	cout << "Goal wrist rotation: (" << goalRot << ")\n";

	float t4, t5, t6;

	//I CHATTED THIS GOTTA FIX!!!

	// Clamp to valid domain
	float s5 = clamp(goalRot(0, 2), -1.0, 1.0);
	t5 = asinf(s5);

	// Detect wrist singularity directly
	const float EPS = 1e-6f;
	bool gimbalLock = fabsf(fabs(s5) - 1.0f) < 1e-4f;

	if (gimbalLock) {
		// Snap exactly to ±90°
		t5 = (s5 > 0 ? PI / 2 : -PI / 2);

		// One DOF lost → choose a convention
		t4 = 0.0f;
		t6 = atan2f(goalRot(1, 0), goalRot(1, 1));
	}
	else {
		t4 = atan2f(-goalRot(1, 2), goalRot(2, 2));
		t6 = atan2f(-goalRot(0, 1), goalRot(0, 0));
	}

	//roll about X
	MatrixXd rot4(3, 3);
	rot4 << 1, 0, 0,
		0, cos(t4), -sin(t4),
		0, sin(t4), cos(t4);

	//roll about Y
	MatrixXd rot5(3, 3);
	rot5 << cos(t5), 0, sin(t5),
		0, 1, 0,
		-sin(t5), 0, cos(t5);

	//pitch about Z
	MatrixXd rot6(3, 3);
	rot6 << cos(t6), -sin(t6), 0,
		sin(t6), cos(t6), 0,
		0, 0, 1;

	MatrixXd calcRot = rot4 * rot5 * rot6;


	cout << "Calculated wrist rotation: (" << calcRot << ")\n";
	cout << "\nWrist angles: (" << t4 << ", " << t5 << ", " << t6 << ")\n";

	return Eigen::Vector3d(t4, t5, t6);
}