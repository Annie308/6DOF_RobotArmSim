#pragma once

#include <vector>
#include <random>
#include <Eigen/Dense>

extern std::mt19937 gen;

std::vector <Eigen::Vector3d> fk(float theta1, float theta2, float theta3, float roll1, float pitch, float yaw);
Eigen::Vector3d ik(float x, float y, float z);
Eigen::Vector3d wristIK(float x, float y, float z, float roll, float pitch, float yaw);
void fk_test(float t1, float t2, float t3);