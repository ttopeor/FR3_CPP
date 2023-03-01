
#include "math_tools.h"

void matrixToPose(const std::array<double, 16> &matrix, double &x, double &y, double &z, double &Rx, double &Ry, double &Rz)
{
    // Extract translation components of matrix
    x = matrix[12];
    y = matrix[13];
    z = matrix[14];

    // Extract rotation components of matrix
    double sy = std::sqrt(matrix[0] * matrix[0] + matrix[1] * matrix[1]);
    bool singular = sy < 1e-6;
    if (!singular)
    {
        Rx = std::atan2(matrix[9], matrix[10]);
        Ry = std::atan2(-matrix[8], sy);
        Rz = std::atan2(matrix[4], matrix[0]);

        Rx = std::atan2(matrix[6], matrix[10]);
        Ry = std::atan2(-matrix[2], sy);
        Rz = std::atan2(matrix[1], matrix[0]);
    }
    else
    {
        Rz = 0.0;
        if (abs(matrix[2] - (-1)) < 1e-6)
        {
            Ry = M_PI / 2;
            Rx = Rz + atan2(matrix[4], matrix[8]);
        }
        else
        {
            Ry = -M_PI / 2;
            Rx = -1 * Ry + atan2(-1 * matrix[4], -1 * matrix[8]);
        }
    }
}

std::vector<std::array<double, 16>> traj_to_matrices(const std::vector<std::vector<double>> &traj)
{
    std::vector<std::array<double, 16>> matrices;
    for (const auto &pose : traj)
    {
        std::array<double, 16> mat{};
        // Extract pose elements
        const double x = pose[0];
        const double y = pose[1];
        const double z = pose[2];
        const double rx = pose[3];
        const double ry = pose[4];
        const double rz = pose[5];

        // Compute transformation matrix
        const double ca = cos(rx);
        const double sa = sin(rx);
        const double cb = cos(ry);
        const double sb = sin(ry);
        const double cc = cos(rz);
        const double sc = sin(rz);

        mat[0] = cb * cc;
        mat[1] = sa * sb * sc + ca * sc;
        mat[2] = -ca * sb * cc + sa * sc;
        mat[3] = 0;
        mat[4] = -cb * sc;
        mat[5] = -sa * sb * sc + ca * cc;
        mat[6] = ca * sb * sc + sa * cc;
        mat[7] = 0;
        mat[8] = sb;
        mat[9] = -sa * cb;
        mat[10] = ca * cb;
        mat[11] = 0;
        mat[12] = x;
        mat[13] = y;
        mat[14] = z;
        mat[15] = 1;

        matrices.push_back(mat);
    }
    return matrices;
}

void printArray16(std::array<double, 16> arr)
{
    for (double element : arr)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}

void printVector(const std::vector<std::vector<double>> &vec)
{
    for (int i = 0; i < vec.size(); i++)
    {
        for (int j = 0; j < vec[i].size(); j++)
        {
            std::cout << vec[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void printVectorOfArrays16(const std::vector<std::array<double, 16>> &vec)
{
    for (const auto &arr : vec)
    { // iterate over each array in the vector
        for (const auto &val : arr)
        {                            // iterate over each double value in the array
            std::cout << val << " "; // print the value
        }
        std::cout << std::endl; // move to the next line for the next array
    }
}
