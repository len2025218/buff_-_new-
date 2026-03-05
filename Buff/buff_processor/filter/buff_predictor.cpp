#include "./buff_predictor.h"

namespace rm_buff
{
    Predictor::Predictor()
    {
        particle_filter_ = std::make_unique<ParticleFilter>();
    }
    Predictor::~Predictor()
    {
    }
    /**
     * @param result 最初会先把模式位传入函数
     */
    bool Predictor::update(double speed[], double dist, uint64_t timestamp, double &result)
    {
        int mode = result;
        result = 0;
        TargetInfo target = {speed[0], dist, timestamp};

        // mode = 1;
        // std::cout << "mode: " << mode << std::endl;
        if (mode != mode_)
        {
            history_info.clear();
            particle_filter_->initParam();
            is_params_confirmed_ = false;
        }
        mode_ = mode;

        if ((history_info.size() < 1) || (((target.timestamp - history_info.front().timestamp) / 1e9) >= max_timespan_))
        {
            history_info.clear();
            history_info.push_back(target);
            params_[0] = 0.00;
            params_[1] = 0.00;
            params_[2] = 0.00;
            params_[3] = 0.00;
            particle_filter_->initParam();
            is_params_confirmed_ = false;
            result = 0;

            return false;
        }
        Eigen::VectorXd measure(1);
        measure << speed[0];

        particle_filter_->update(measure);
        if (particle_filter_->is_ready)
        {
            auto predict = particle_filter_->predict();
            target.speed = predict(0, 0);
            speed[1] = predict(0, 0);
        }
        // 维护队列
        int deque_len = history_deque_len_short_;

        // std::cout << "is_params_confirmed_" << is_params_confirmed_ << std::endl;
        if (mode_ == 2 && (!is_params_confirmed_)) //
        {
            deque_len = history_deque_len_long_;
        }
        std::cout << "history_info.size()" << history_info.size() << std::endl;
        std::cout << "deque_len" << deque_len << std::endl;
        if ((int)(history_info.size()) < deque_len)
        {
            if (mode_ == 1)
            {
                history_info.push_back(target);
            }
            else
            {
                history_info.push_back(target);
                result = 0;
                return false;
            }
        }
        else if ((int)(history_info.size()) == deque_len)
        {
            history_info.pop_front();
            history_info.push_back(target);
        }
        else if ((int)(history_info.size()) > deque_len)
        {
            while ((int)(history_info.size()) >= deque_len)
            {
                history_info.pop_front();
            }
            history_info.push_back(target);
        }
        // 计算旋转方向
        double rotate_speed_sum = 0;
        // int rotate_sign = 0;
        for (auto target_info : history_info)
        {
            rotate_speed_sum += target_info.speed;
        }
        auto mean_velocity = rotate_speed_sum / history_info.size();

        if (mode_ == 1) //------------> 小符
        {
            params_[3] = mean_velocity;
            cout << "b上" << mean_velocity << endl;
            is_params_confirmed_ = true;
        }
        else if (mode_ == 2) //---------------------> 大符
        {
            int rotate_sign = 0;
            cout << "is_params_confirmed_" << is_params_confirmed_ << endl;
            if (!is_params_confirmed_)
            {
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary; // 优化信息

                double params_fitting[4] = {1, 1, 1, mean_velocity};

                if (rotate_speed_sum / fabs(rotate_speed_sum) >= 0)
                {
                    rotate_sign = 1;
                }
                else
                {
                    rotate_sign = -1;
                }
                for (auto target_info : history_info)
                {
                    problem.AddResidualBlock( // 向问题中添加误差项
                                              // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                            new CURVE_FITTING_COST(
                                target_info.speed * rotate_sign,
                                target_info.timestamp / 1e9)),
                        new ceres::CauchyLoss(1.0),
                        params_fitting // 待估计参数
                    );
                }
                // 设置上下限

                // FIXME:参数需根据场上大符实际调整
                problem.SetParameterLowerBound(params_fitting, 0, param_bound_[0]);
                problem.SetParameterUpperBound(params_fitting, 0, param_bound_[1]);
                problem.SetParameterLowerBound(params_fitting, 1, param_bound_[2]);
                problem.SetParameterUpperBound(params_fitting, 1, param_bound_[3]);
                problem.SetParameterLowerBound(params_fitting, 2, -CV_PI);
                problem.SetParameterUpperBound(params_fitting, 2, CV_PI);
                problem.SetParameterLowerBound(params_fitting, 3, param_bound_[6]);
                problem.SetParameterUpperBound(params_fitting, 3, param_bound_[7]);
                ceres::Solve(options, &problem, &summary);
                double params_tmp[4] = {params_fitting[0] * rotate_sign, params_fitting[1], params_fitting[2], params_fitting[3] * rotate_sign};
                auto rmse = evalRMSE(params_tmp);
                std::cout << "rmse:" << rmse << std::endl;
                if (rmse > max_rmse_)
                {
                    return false;
                }
                else
                {
                    params_[0] = params_fitting[0] * rotate_sign;
                    params_[1] = params_fitting[1];
                    params_[2] = params_fitting[2];
                    params_[3] = params_fitting[3] * rotate_sign;
                    is_params_confirmed_ = true;
                }
            }
            else
            {
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary; // 优化信息
                double phase;

                for (auto target_info : history_info)
                {
                    problem.AddResidualBlock( // 向问题中添加误差项
                                              // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1>(
                            new CURVE_FITTING_COST_PHASE(
                                (target_info.speed - params_[3]) * rotate_sign,
                                target_info.timestamp / 1e9,
                                params_[0],
                                params_[1],
                                params_[3])),
                        new ceres::CauchyLoss(1e1),
                        &phase // 待估计参数
                    );
                }
                // 设置上下限
                problem.SetParameterUpperBound(&phase, 0, CV_PI);
                problem.SetParameterLowerBound(&phase, 0, -CV_PI);

                ceres::Solve(options, &problem, &summary);
                double params_new[4] = {params_[0], params_[1], phase, params_[3]};
                auto old_rmse = evalRMSE(params_);
                auto new_rmse = evalRMSE(params_new);
                if (new_rmse < old_rmse && new_rmse <= max_rmse_)
                {
                    params_[2] = phase;
                }
            }
        }
        double delay = (mode == 2 ? delay_big_ : delay_small_);
        // double delay = 0.32;
        double delta_time_estimate = ((dist / bullet_speed_) + delay);
        // cout<<"delta_time_estimate"<<delta_time_estimate<<endl;
        // 1
        double timespan = (history_info.back().timestamp / 1e9);
        // 2
        double time_estimate = delta_time_estimate + timespan;
        // 角度差
        result = calcAimingAngleOffset(timespan, time_estimate, speed);

        // std::cout << "result" << result << std::endl;
        return true;
    }



    /**
     * @brief 计算RMSE指标
     *
     * @param params 参数首地址指针
     * @return RMSE值
     */
    double Predictor::evalRMSE(double params[4])
    {
        double rmse_sum = 0;
        double rmse = 0;
        for (auto target_info : history_info)
        {
            double t = (target_info.timestamp) / 1e9;
            double pred = params[0] * sin(params[1] * t + params[2]) + params[3];
            double measure = target_info.speed;
            rmse_sum += pow((pred - measure), 2);
            // cout << "dt:"<< t << " pre:" << pred << " measure:" << measure << endl;
        }
        rmse = sqrt(rmse_sum / history_info.size());
        return rmse;
    }
    double Predictor::calcAimingAngleOffset(double t0, double t1, double speed[])
    {
        auto a = params_[0];
        auto omega = params_[1];
        auto theta = params_[2];
        auto b = params_[3];

        double theta1 = 0.0;
        double theta0 = 0.0;

        cout << "t0:" << t0 << " t1:" << t1 << endl;
        // cout << "a: " << a << " omega:" << omega << " theta:" << theta << " b:" << b << endl;
        cout << "params_[3]:" << params_[3] << endl;
        // f(t) = a * sin(ω * t + θ) + b
        // 对目标函数进行积分
        if (mode_ == 1) // 适用于小符模式
        {
            if (params_[3] > 0)
            {
                b = 1.0 / 3.0 * CV_PI;
            }
            else
            {
                b = -1.0 / 3.0 * CV_PI;
            }
            cout << "b" << b << endl;
            theta0 = b * t0;
            theta1 = b * t1;
        }
        else if (mode_ == 2)
        {
            theta0 = (b * t0 - (a / omega) * cos(omega * t0 + theta));
            theta1 = (b * t1 - (a / omega) * cos(omega * t1 + theta));
        }
        return theta1 - theta0;
    }
    bool Predictor::init()
    {
        return true;
    }

}