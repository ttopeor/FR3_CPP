#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

void matrixToPose(const std::array<double, 16> &matrix, double &x, double &y, double &z, double &Rx, double &Ry, double &Rz);
std::vector<std::array<double, 16>> traj_to_matrices(const std::vector<std::vector<double>> &traj);
void printVector(const std::vector<std::vector<double>> &vec);
void printVectorOfArrays16(const std::vector<std::array<double, 16>> &vec);
void printArray16(std::array<double, 16> arr);