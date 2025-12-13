#pragma once

#include <vector>
#include <random>
#include <Eigen/Dense>

extern std::mt19937 gen;

std::vector <Eigen::Vector3d> fk(float t1, float t2, float t3, float t4, float t5, float t6);
Eigen::Vector3d ik(float x, float y, float z);
Eigen::Vector3d wristIk(float x, float y, float z, float t4, float t5, float t6);
void fk_test(float t1, float t2, float t3);