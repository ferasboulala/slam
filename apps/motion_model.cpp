#include "motion.h"
#include "thirdparty/json.h"

#include <cmath>
#include <vector>
#include <array>

int main()
{
    constexpr int N_POINTS = 100;
    constexpr int N_EPOCH = 5;
    constexpr std::array<double, 4> NOISY_PARAMS = { 0.01, 0.01, 0.01, 0.01 };
    constexpr std::array<double, 4> NO_NOISE_PARAMS = { 0, 0, 0, 0 };

    const slam::Odometry command{M_PI / 20, 0.25, 0};
    std::vector<double> xs(N_POINTS, 0);
    std::vector<double> ys(N_POINTS, 0);
    std::vector<double> thetas(N_POINTS, 0);
    slam::Pose real_position{0, 0, 0};

    nlohmann::json j;
    for (int i = 0; i < N_EPOCH; ++i)
    {
        real_position = slam::sample_motion_model_odometry(command, real_position, NO_NOISE_PARAMS);

        auto &j_epoch = j[std::to_string(i).c_str()];
        j_epoch["real_position"] = { real_position.x, real_position.y, real_position.theta };
        j_epoch["x"] = std::vector<double>();
        j_epoch["y"] = std::vector<double>();
        j_epoch["theta"] = std::vector<double>();

        for (int j = 0; j < N_POINTS; ++j)
        {
            const slam::Pose pose = slam::sample_motion_model_odometry(command, { xs[j], ys[j], thetas[j] }, NOISY_PARAMS);
            xs[j] = pose.x;
            ys[j] = pose.y;
            thetas[j] = pose.theta;

            j_epoch["x"].push_back(xs[j]);
            j_epoch["y"].push_back(ys[j]);
            j_epoch["theta"].push_back(thetas[j]);
        }
    }

    printf("%s", j.dump().c_str());

    return 0;
}
