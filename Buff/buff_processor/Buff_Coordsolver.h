#pragma once
#ifndef COORDSOLVER_H
#define COORDSOLVER_H
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp>
#include <array>
#include <vector>
#include <string>

namespace buff_processor
{

    class coordsolver
    {
        std::string coord_path = "../Buff/buff_processor/config/info_config.yaml";
        std::vector<cv::Point3d> points_world =
            {
                {0, -0.7, -0.05},
                {-0.0, -0.13399, 0.0},
                {-0.13399, -0., 0.0},
                {-0.0, 0.13399, 0.0},
                {0.13399, 0.0, 0.0},
        };
        Eigen::Vector3d R_center_world = {0, -0.7, -0.5};

        Eigen::Matrix4d T_ci_;
        Eigen::Matrix4d T_ic_; // 转移矩阵
        Eigen::Vector3d T_iw_; // 补偿向量
        Eigen::Matrix3d T_cc_; // 相机参数
        cv::Mat intrinstic = cv::Mat(3, 3, CV_64F);
        cv::Mat coeff = cv::Mat(1, 5, CV_64F);

    public:
        struct PnPInfo
        {
            Eigen::Vector3d armor_cam;
            Eigen::Vector3d armor_world;
            Eigen::Vector3d R_cam;
            Eigen::Vector3d R_world;
            // Eigen::Vector3d euler;
            Eigen::Matrix3d rmat;
            cv::Mat rvec;
            cv::Mat tvec;
        };
        coordsolver::PnPInfo result;
        std::vector<double> dis_coeff; // 假设 dis_coeff 是畸变系数
        Eigen::Vector2d angle_offset;
        Eigen::Vector3d xyz_offset;
        int max_iter = 10;
        int R_K_iter = 50;
        float stop_error = 0.001;
        const double k = 0.01903; // 25°C,1atm,小弹丸
        const double g = 9.781;
        double bullet_speed = 27.0;
        coordsolver();
        ~coordsolver();
        void InitConfig();
        bool initMatrix(Eigen::MatrixXd &matrix, cv::FileNode vector);
        cv::Point2f reproject(Eigen::Vector3d &xyz);
        cv::Point2f reproject(cv::Mat &xyz);
        Eigen::Vector3d worldToCam(const Eigen::Vector3d &point_world, const Eigen::Matrix3d &rmat);
        Eigen::Vector3d camToWorld(const Eigen::Vector3d &point_camera, const Eigen::Matrix3d &rmat);
        // Eigen::Matrix3d camToWorld(const Eigen::Matrix3d &martirx_camera, const Eigen::Matrix3d &rmat);
        Eigen::Vector2d getAttitude(Eigen::Vector3d &xyz, Eigen::Matrix3d &rmat);
        inline double gainBallisticPitch(Eigen::Vector3d &xyz);
        inline Eigen::Vector2d gainAngle(Eigen::Vector3d &xyz_cam);
        Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
        inline double gainYaw(Eigen::Vector3d &xyz);
        inline double gainPitch(Eigen::Vector3d &xyz);
        PnPInfo pnp(const std::vector<cv::Point2f> &points_pic, Eigen::Matrix3d &rmat_imu,
                    int method);
    };
}

#endif