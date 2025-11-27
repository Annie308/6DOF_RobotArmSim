#pragma once

#include <vector>
#include <random>

extern std::mt19937 gen;

std::vector<std::tuple<float, float, float>> fk(float theta1, float theta2, float theta3);
std::tuple<float,float,float> ik(float x, float y, float z);
std::vector<float> simulate_angles(float theta1, float theta2, float theta3);
void fk_test(float t1, float t2, float t3);