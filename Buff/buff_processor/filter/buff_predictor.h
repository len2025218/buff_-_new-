#ifndef BUFF_PROCESSOR__PREDICTOR_HPP_
#define BUFF_PROCESSOR__PREDICTOR_HPP_
// c++
#include <iostream>
#include <ctime>
#include <future>
#include <random>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>

// ceres/eigen
#include <ceres/ceres.h>
#include <Eigen/Core>

#include "./linear_particle_filter.h"

using namespace std;
using namespace cv;

using namespace std;
using namespace cv;
namespace rm_buff
{
    struct TargetInfo
    {
        double speed;
        double dist;
        uint64_t timestamp;
    };
    class Predictor
    {
        struct CURVE_FITTING_COST
        {
            CURVE_FITTING_COST(double x, double t)
                : _x(x), _t(t) {}

            // 残差的计算
            template <typename T>
            bool operator()(
                const T *params, // 模型参数，有3维
                T *residual      // 残差
            ) const
            {
                residual[0] = T(_x) - params[0] * ceres::sin(params[1] * T(_t) + params[2]) - params[3]; // f(t) = a * sin(ω * t + θ) + b
                return true;
            }
            const double _x, _t; // x,t数据
        };

        struct CURVE_FITTING_COST_PHASE
        {
            CURVE_FITTING_COST_PHASE(double x, double t, double a, double omega, double dc)
                : _x(x), _t(t), _a(a), _omega(omega), _dc(dc) {}

            // 残差的计算
            template <typename T>
            bool operator()(
                const T *phase, // 模型参数，有1维
                T *residual     // 残差
            ) const
            {
                residual[0] = T(_x) - T(_a) * ceres::sin(T(_omega) * T(_t) + phase[0]) - T(_dc); // f(x) = a * sin(ω * t + θ)
                return true;
            }
            const double _x, _t, _a, _omega, _dc; // x,t数据
        };
        struct PredictStatus
        {
            bool xyz_status[3];
        };

        int mode_;
        int history_deque_len_long_ = 270;
        int history_deque_len_short_ = 470;
        double max_rmse_ = 3.5;

    public:
        double bullet_speed_ = 21;

        std::unique_ptr<ParticleFilter> particle_filter_; // 坐标系转化类

        bool is_params_confirmed_ = false;
        std::deque<TargetInfo> history_info; // 目标队列
        double delay_big_ = 10;
        double delay_small_ = 0.32;
        double params_[4] = {0.00, 0.00, 0.00, 0.00};
        double initial_omega;
        double param_bound_[8];
        uint64_t max_timespan_ = 60;
        Predictor();
        ~Predictor();
        double calcAimingAngleOffset(double t0, double t1, double speed[]);
        // bool update(double speed[], double dist, uint64_t timestamp, double &result);
        bool update(double speed[], double dist, uint64_t timestamp, double &result);
        bool init();
        double evalRMSE(double params[4]);
        // double analyzeFrequency();
    };
}
#endif
