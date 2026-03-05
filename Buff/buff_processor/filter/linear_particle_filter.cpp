#include "./linear_particle_filter.h"

namespace rm_buff
{
    ParticleFilter::ParticleFilter()
    {
        vector_len_ = 1;
        num_particle_ = 400;
        process_noise_cov_(0, 0) = 0.10;
        observe_noise_cov_(0, 0) = 0.15;
        initParam();
    }
    ParticleFilter::~ParticleFilter()
    {
    }
    /**
     * @brief 生成正态分布矩阵
     *
     * @param matrix
     * @param cov
     * @return true
     * @return false
     */

    bool ParticleFilter::randomlizedGaussianColwise(Eigen::MatrixXd &matrix, Eigen::MatrixXd &cov)
    {

        std::random_device rd;
        std::default_random_engine e(rd());
        std::vector<std::normal_distribution<double>> normal_distribution_list;

        // 假设各个变量不相关
        for (int i = 0; i < cov.cols(); i++)
        {
            std::normal_distribution<double> n(0, cov(i, i));
            normal_distribution_list.push_back(n);
        }

        for (int col = 0; col < matrix.cols(); col++)
        {
            // cout<<normal_distribution_list[col](e)<<endl;
            for (int row = 0; row < matrix.rows(); row++)
            {
                auto tmp = normal_distribution_list[col](e);
                matrix(row, col) = tmp;
                // matrix(row,col) = 1;
            }
        }

        return true;
    }

    Eigen::VectorXd ParticleFilter::predict()
    {
        Eigen::VectorXd particles_weighted = matrix_particle_.transpose() * matrix_weights_;
        return particles_weighted;
    }

    bool ParticleFilter::update(Eigen::VectorXd measure)
    {
        Eigen::MatrixXd gaussian = Eigen::MatrixXd::Zero(num_particle_, vector_len_);
        Eigen::MatrixXd mat_measure = measure.replicate(1, num_particle_).transpose();
        auto err = ((measure - (matrix_particle_.transpose() * matrix_weights_)).norm());

        if (is_ready)
        {
            // 序列重要性采样
            matrix_weights_ = Eigen::MatrixXd::Ones(num_particle_, 1);
            // 按照高斯分布概率密度函数曲线右半侧计算粒子权重
            for (int i = 0; i < matrix_particle_.cols(); i++)
            {
                auto sigma = observe_noise_cov_(i, i);
                Eigen::MatrixXd weights_dist = (matrix_particle_.col(i) - mat_measure.col(i)).rowwise().squaredNorm();
                Eigen::MatrixXd tmp = ((-(weights_dist / pow(sigma, 2)) / matrix_particle_.cols()).array().exp() / (sqrt(CV_2PI) * sigma)).array();
                matrix_weights_ = matrix_weights_.array() * tmp.array();
            }
            matrix_weights_ /= matrix_weights_.sum();
            double n_eff = 1.0 / (matrix_weights_.transpose() * matrix_weights_).value();
            // TODO:有效粒子数少于一定值时进行重采样,该值需在实际调试过程中修改
            //  if (n_eff < 0.5 * num_particle)
            if (err > observe_noise_cov_(0, 0) || (n_eff < 0.5 * num_particle_))
            {
                // cout<<"res"<<num_particle<<endl;
                resample();
            }
        }
        else
        {
            matrix_particle_ += mat_measure;
            is_ready = true;
            return false;
        }
        return true;
    }
    bool ParticleFilter::resample()
    {

        // 重采样采用低方差采样,复杂度为O(N),较轮盘法的O(NlogN)更小,实现可参考<Probablistic Robotics>
        std::random_device rd;
        std::default_random_engine e(rd());
        std::uniform_real_distribution<> random{0.0, 1. / num_particle_};

        int i = 0;
        double c = matrix_weights_(0, 0);
        auto r = random(e);
        Eigen::MatrixXd matrix_particle_tmp = matrix_particle_;

        for (int m = 0; m < num_particle_; m++)
        {
            auto u = r + m * (1. / num_particle_);
            // 当 u > c 不进行采样
            while (u > c)
            {
                i++;
                c = c + matrix_weights_(i, 0);
            }
            matrix_particle_tmp.row(m) = matrix_particle_.row(i);
        }
        Eigen::MatrixXd gaussian = Eigen::MatrixXd::Zero(num_particle_, vector_len_);
        randomlizedGaussianColwise(gaussian, process_noise_cov_);
        matrix_particle_ = matrix_particle_tmp + gaussian;
        matrix_weights_ = Eigen::MatrixXd::Ones(num_particle_, 1) / float(num_particle_);
        return true;
    }

    /*
        生成初始粒子群：通过 randomlizedGaussianColwise 生成服从 process_noise_cov_ 的粒子。
    权重初始化为均匀分布（和为1）。
细节：
    初始粒子围绕0值分布（Zero + 高斯噪声）。
    */
    bool ParticleFilter::initParam()
    {
        matrix_particle_ = Eigen::MatrixXd::Zero(num_particle_, vector_len_);
        randomlizedGaussianColwise(matrix_particle_, process_noise_cov_);
        matrix_weights_ = Eigen::MatrixXd::Ones(num_particle_, 1) / float(num_particle_);
        is_ready = false;
        return true;
    }
}
