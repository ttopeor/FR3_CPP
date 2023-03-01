
#include <cmath>
#include <iostream>

#include "matplotlibcpp.h"

#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>

#include "Kalman.h"
#include "math_tools.h"

int plot_motion(std::array<double, 16> &initial_position, std::vector<std::array<double, 16>> &motion);
std::vector<std::array<double, 16>> trajactory_generator(const std::vector<std::vector<double>> &traj);