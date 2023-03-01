#include "Motion_plan.h"

namespace plt = matplotlibcpp;
const int Traj_HZ = 50;

int plot_motion(std::array<double, 16> &initial_position, std::vector<std::array<double, 16>> &motion)
{
    int count = 0;
    std::vector<double> x, z;
    std::vector<double> x_raw, z_raw;
    for (const auto &pose : motion)
    {
        if (count < 1000)
        {
            if (count % 1 == 0)
            {
                x.push_back(pose[12]);
                z.push_back(pose[14]);
            }
            count = count + 1;
        }
    }
    count = 0;
    for (const auto &pose : motion)
    {
        if (count < 1000)
        {
            if (count % 20 == 0)
            {
                x_raw.push_back(pose[12]);
                z_raw.push_back(pose[14]);
            }
            count = count + 1;
        }
    }

    plt::figure();

    // Create a 3D scatter plot of the motion data
    plt::scatter(x, z, 1);
    plt::scatter(x_raw, z_raw, 5);
    plt::xlabel("x");
    plt::ylabel("z");
    // Show the plot
    plt::show();

    return 0;
}

// input {{x,y,z,Rx,Ry,Rz},{x,y,z,Rx,Ry,Rz}........} 30HZ  --> output{T1,T2,T3,T4...........} 1000HZ
std::vector<std::array<double, 16>> trajactory_generator(const std::vector<std::vector<double>> &traj)
{
    std::vector<std::array<double, 16>> final_trajectory;

    const int up_sample_times = int(1000.0 / Traj_HZ);
    const int traj_size = traj.size();
    const double dtime1 = 1.0 / Traj_HZ;
    const double dtime2 = 0.001;

    std::vector<double> time1(traj_size);
    std::vector<double> time2(traj_size * up_sample_times);
    for (int i = 0; i < traj_size; i++)
    {
        time1[i] = i * dtime1;
    }

    double start_time = 0.0;
    double end_time = time1.back();
    double step_size = (end_time - start_time) / (traj_size * up_sample_times - 1);

    for (int i = 0; i < traj_size * up_sample_times; i++)
    {
        time2[i] = start_time + i * step_size;
    }

    std::vector<std::vector<double>> generated_traj(traj_size * up_sample_times, std::vector<double>(6));
    std::vector<std::vector<double>> generated_traj_T(6, std::vector<double>(traj_size * up_sample_times));
    std::vector<std::vector<double>> traj_pos_T(6, std::vector<double>(traj_size));
    std::vector<std::vector<double>> traj_pos = traj;
    for (int i = 0; i < traj_size; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            traj_pos_T[j][i] = traj_pos[i][j];
        }
    }
    // cubic inteprelation
    for (int i = 0; i < 6; i++)
    {
        gsl_interp_accel *acc = gsl_interp_accel_alloc();
        gsl_spline *spline = gsl_spline_alloc(gsl_interp_cspline, traj_size);
        gsl_spline_init(spline, time1.data(), traj_pos_T[i].data(), traj_size);
        for (int j = 0; j < time2.size(); j++)
        {
            double t = time2[j];
            generated_traj_T[i][j] = gsl_spline_eval(spline, t, acc);
        }
        gsl_spline_free(spline);
        gsl_interp_accel_free(acc);
    }

    for (int i = 0; i < traj_size * up_sample_times; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            generated_traj[i][j] = generated_traj_T[j][i];
        }
    }
    // add filter to traj
    double Q = 0.01; // process noise covariance
    double R = 500;  // measurement noise covariance
    double x0;       // initial state estimate
    double P0 = 0.0; // initial estimate covariance

    KalmanFilter kal0(Q, R, generated_traj[0][0], P0);
    KalmanFilter kal1(Q, R, generated_traj[0][1], P0);
    KalmanFilter kal2(Q, R, generated_traj[0][2], P0);
    KalmanFilter kal3(Q, R, generated_traj[0][3], P0);
    KalmanFilter kal4(Q, R, generated_traj[0][4], P0);
    KalmanFilter kal5(Q, R, generated_traj[0][5], P0);
    std::vector<KalmanFilter> kal_list = {kal0, kal1, kal2, kal3, kal4, kal5};
    std::vector<std::vector<double>> generated_traj_kal(traj_size * up_sample_times, std::vector<double>(6));
    for (int i = 0; i < traj_size * up_sample_times; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            generated_traj_kal[i][j] = kal_list[j].update(generated_traj[i][j]);
        }
    }
    // transformation from {x,y,z,Rx,Ry,Rz} to T

    final_trajectory = traj_to_matrices(generated_traj_kal);

    return final_trajectory;
}