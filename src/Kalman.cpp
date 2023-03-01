#include <iostream>
#include <cmath>
#include "Kalman.h"

using namespace std;

double KalmanFilter::update(double z)
{
    // Predict
    double x_pred = x;
    double P_pred = P + Q;

    // Update
    double K = P_pred / (P_pred + R);
    x = x_pred + K * (z - x_pred);
    P = (1 - K) * P_pred;

    return x;
}
