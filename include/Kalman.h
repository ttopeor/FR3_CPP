
class KalmanFilter
{
public:
    // constructor
    KalmanFilter(double Q, double R, double x0, double P0) : Q(Q), R(R), x(x0), P(P0){};
    double update(double z);

private:
    double Q; // process noise covariance
    double R; // measurement noise covariance
    double x; // state estimate
    double P; // estimate covariance
};
