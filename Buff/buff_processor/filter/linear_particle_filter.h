/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 02:36:09
 * @LastEditTime: 2023-05-29 22:28:28
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/particle_filter.hpp
 */
#pragma once 
#ifndef BUFF_PROCESSOR__PARTICLE_FILTER_H_
#define BUFF_PROCESSOR__PARTICLE_FILTER_H_
// #include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <random>
namespace rm_buff
{
    class ParticleFilter
    {

    public:
        bool initParam();

        ParticleFilter();
        ~ParticleFilter();
        Eigen::VectorXd predict();
        bool update(Eigen::VectorXd measure);
        bool resample();
        ///////////////////////////
        bool randomlizedGaussianColwise(Eigen::MatrixXd &matrix, Eigen::MatrixXd &cov);

        bool is_ready;
        //
        int vector_len_;
        int num_particle_;
        Eigen::MatrixXd process_noise_cov_=Eigen::MatrixXd::Identity(1, 1);
        Eigen::MatrixXd observe_noise_cov_=Eigen::MatrixXd::Identity(1, 1);
        // vector_len_ = 1;
        // num_particle_ = 400;
        // process_noise_cov_ << 0.1;
        // observe_noise_cov_ << 0.15;

        Eigen::MatrixXd weights_;

        Eigen::MatrixXd matrix_estimate_;
        Eigen::MatrixXd matrix_particle_;
        Eigen::MatrixXd matrix_weights_;
    };
}

#endif
