#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

class Controller
{
public:
    Controller(size_t dq_filter_size,
               const std::array<double, 7> &K_P,
               const std::array<double, 7> &K_D);
    franka::Torques step(const franka::RobotState &state);

private:
    void updateDQFilter(const franka::RobotState &state);
    double getDQFiltered(size_t index) const;

    size_t dq_current_filter_position_;
    size_t dq_filter_size_;

    const std::array<double, 7> K_P_;
    const std::array<double, 7> K_D_;

    std::array<double, 7> dq_d_;
    std::unique_ptr<double[]> dq_buffer_;
};