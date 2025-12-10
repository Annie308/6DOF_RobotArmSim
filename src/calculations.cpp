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


using Eigen::MatrixXd, Eigen::Vector3d;
using namespace std;

//random seeding
unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
std::mt19937 gen(seed);

//Forward kinematics: joint angles to cartesian
//Formula: x = Ax + d, where A is rotation matrix, d is translation vector
vector<Vector3d> fk(float theta1, float theta2, float theta3, float roll, float pitch, float yaw) {
	Vector3d link1(0, l1, 0);
	Vector3d link2(l2, 0,0);
	Vector3d link3(l3, 0,0);

	Eigen::Vector3d link4 = { l4, 0,0, };
	Eigen::Vector3d link5(l5, 0, 0);
	Eigen::Vector3d link6(l6, 0, 0);

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

	//roll about Y
	MatrixXd rot4(3, 3);
	rot4 << cos(roll), 0, sin(roll),
		0, 1, 0,
		-sin(roll), 0, cos(roll);

	//pitch about Z
	MatrixXd rot5(3, 3);
	rot5 << cos(pitch), -sin(pitch), 0,
		sin(pitch), cos(pitch), 0,
		0, 0, 1;

	//roll about Y
	MatrixXd rot6(3, 3);
	rot6 << cos(yaw), 0, sin(yaw),
		0, 1, 0,
		-sin(yaw), 0, cos(yaw);

	//defining joint positons after transformations
	Eigen::Vector3d link1_end = rot1 * link1;
	Eigen::Vector3d link2_end = rot1 * rot2 * link2 + link1_end;
	Eigen::Vector3d link3_end = rot1 * rot2 * rot3 * link3 + link2_end;
	
	Eigen::Vector3d link4_end = rot4 * link4 + link3_end;
	Eigen::Vector3d link5_end = rot4 * rot5 * link5 + link4_end;
	Eigen::Vector3d link6_end = rot4 * rot5 * rot6 * link6 + link5_end;

	return { link1_end, link2_end, link3_end, link4_end, link5_end, link6_end};
}
static bool target_reached(float x, float y, float z, float t1, float t2, float t3) {
	Eigen::Vector3d tar_vec = fk(t1, t2, t3,0,0,0).back();

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
	//first we check if a solution exists:
	/*
	if (sqrt(x * x + y * y + z * z) > l1 + l2 + l3) {
		cout << "No solution found. Outside workspace.\n";
		return { 0,0,0 };
	}
	*/
	Eigen::Vector3d p_target = Eigen::Vector3d(x-l4-l5-l6, y-l1, z);
	//angle away from XY plane
	float theta = atan2(z, x);

	//rotate on the y axis to project onto XY plane
	MatrixXd proj_max(3, 3);
	proj_max << cos(theta), 0, sin(theta),
		0, 1, 0,
		-sin(theta), 0, cos(theta);

	//projected point 
	Eigen::Vector3d p_proj = proj_max * p_target;
	//cout << "Projected point at: ("<<p_proj(0) << " " << p_proj(1) << " " << p_proj(2) << ")\n";

	//defining new coordinates 
	float x1 = p_proj(0), y1 = p_proj(1);
	float r1 = sqrt(x1*x1 + y1 * y1);			//new length
	float phi = atan2(y1, x1);					//angle from base to projected point

	//angle from the new point using cosine law
	float cos_theta2 = (l2 * l2 + r1 * r1 - l3 * l3) / (2 * l2 * r1);
	cos_theta2 = min(1.0f, max(-1.0f, cos_theta2));						//constraints
	
	float cos_theta3 = (l2 * l2 + l3 * l3 - r1 * r1) / (2 * l2 * l3);
	cos_theta3 = min(1.0f, max(-1.0f, cos_theta3));						//constraints

	float theta2 = phi - acos(cos_theta2);
	float theta3 = PI - acos(cos_theta3);
	float theta1 = -theta;

	//case 1: theta 1 and theta 2 >0
	if (target_reached(x, y, z, theta1, theta2, theta3)) { 
		cout << "Target is reachable.\n";
		cout << "Returning angles: (" << theta1 << ", " << -theta2 << ", " << theta3 << ")\n";
		return Eigen::Vector3d(theta1, theta2, theta3); 
	}
	//case 2: 1 > 0, 2 < 0
	else if (target_reached(x, y, z, theta1, -theta2, theta3)) { 
		cout << "Target is reachable.\n";
		cout << "Returning angles: (" << theta1 << ", " << -theta2 << ", " << theta3 << ")\n";
		return Eigen::Vector3d(theta1, -theta2, theta3); 
	}
	
	else {
		cout << "Target is unreachable.\n";
		return Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	}
}

Eigen::Vector3d wristIK(float x, float y, float z, float roll, float pitch, float yaw) {
	//convert roll and pitch to rotation matrices: R4 * R5 * R6
	//roll about X
	Eigen::Vector3d wrist_angles = ik(x, y, z);
	float theta1 = wrist_angles(0); float theta2 = wrist_angles(1); float theta3 = wrist_angles(2);

	MatrixXd rot1(3, 3);
	rot1 << cos(theta1), 0, sin(theta1),
		0, 1, 0,
		-sin(theta1), 0, cos(theta1);

	// Rotation about Z
	MatrixXd rot2(3, 3);
	rot2 << cos(theta2), -sin(theta2), 0,
		sin(theta2), cos(theta2), 0,
		0, 0, 1;

	// Rotation about Z
	MatrixXd rot3(3, 3);
	rot3 << cos(theta3), -sin(theta3), 0,
		sin(theta3), cos(theta3), 0,
		0, 0, 1;

	//roll about X
	MatrixXd R_roll(3, 3);
	R_roll << 1, 0 ,0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);

	//pitch about Z
	MatrixXd R_pitch(3, 3);
	R_pitch << cos(pitch), -sin(pitch), 0,
		sin(pitch), cos(pitch), 0,
		0, 0, 1;

	//roll about Y
	MatrixXd R_yaw(3, 3);
	R_yaw << cos(yaw), 0, sin(yaw),
		0, 1, 0,
		-sin(yaw), 0, cos(yaw);

	MatrixXd goalRot = R_roll * R_pitch * R_yaw;

	cout << "\n ==========Wrist IK Results==========\n";
	cout << "Goal wrist rotation: (" << goalRot << ")\n";

	//Rot =  |A D G|
	//       |B E H|
	//       |C F I|

	float t4 = atan2(-goalRot(2, 1), goalRot(0, 1));		//-F/D
	float t5 = acos(goalRot(1, 1));				//E
	float t6 = atan2(goalRot(1, 2), goalRot(1, 0));			//H/B

	// Deubugging
	//roll about Y
	MatrixXd rot4(3, 3);
	rot4 << cos(t4), 0, sin(t4),
		0, 1, 0,
		-sin(t4), 0, cos(t4);

	//pitch about Z
	MatrixXd rot5(3, 3);
	rot5 << cos(t5), -sin(t5), 0,
		sin(t5), cos(t5), 0,
		0, 0, 1;

	//roll about Y
	MatrixXd rot6(3, 3);
	rot6 << cos(t6), 0, sin(t6),
		0, 1, 0,
		-sin(t6), 0, cos(t6);

	MatrixXd calcRot = rot4 * rot5 * rot6;

	if (isnan(t4)) t4 = 0;
	if (isnan(t5)) t5 = 0;
	if (isnan(t6)) t6 = 0;

	
	cout << "Calculated wrist rotation: (" << calcRot << ")\n";
	cout << "\nWrist angles: (" << t4 << ", " << t5 << ", " << t6 << ")\n";	

	return Eigen::Vector3d(t4, t5, t6);
}


