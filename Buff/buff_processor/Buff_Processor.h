#pragma once
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <Eigen/Dense>
#include <Eigen/Core>
#include "./filter/buff_predictor.h"
#include "./Buff_Coordsolver.h"
#include "../../general.h"
#include <filesystem>
#include <cmath> // 提供 M_PI 常量
namespace buff_processor
{
    namespace fs = std::filesystem; // 正确声明命名空间别名
    struct TrackedFan
    {
        uint64_t timestamp;       // 时间戳
        Eigen::Matrix3d rotation; // 旋转矩阵
    };
    class BuffProcessor
    {
        //保存
        int frame_count = 0; // 总帧计数器（持续递增）
        int saved_count = 0; // 实际保存的帧数
        std::string output_dir = "./frames/";

        std::unique_ptr<rm_buff::Predictor> predictor_;
        std::unique_ptr<coordsolver> coordsolver_;
        std::string init_config_path = "../Buff/buff_processor/config/init_config.yaml";
        cv::Mat fan2center_ = (cv::Mat_<float>(3, 1) << 0, -0.7, -0.05);
        std::map<std::string, double> dictionary; // 调控相关
        cv::Point2f center_[3];
        double fan_length_ = 0.7;
        coordsolver::PnPInfo pnp_result;
        double speed_[2];
        Eigen::Vector2d predict_;
        std::vector<buff_processor::TrackedFan> tracker_; // 追踪器
        cv::Point2f calculate_;
        Eigen::Vector3d center_xyz_;
        cv::Mat camera_matrix_ = cv::Mat(3, 3, CV_64F);

    public:
        bool InitConfig();
        BuffProcessor();
        ~BuffProcessor();
        bool processArmors(BuffTaskData &buffdata, MCUData &mcudata, VisionData &visiondata);
        void process2DPoints(BuffTaskData &buffdata, Eigen::Matrix3d &rmat_imu);
        void calculateSpinSpeed(uint64_t now, const Eigen::Matrix3d &current_rotation);
        void solveResult(VisionData &visiondata, Eigen::Matrix3d &rmat_imu, BuffTaskData &buffdata);
        void drawcenter(cv::Mat buff_center, cv::Mat &src, int type);
        cv::Point2f reproject(cv::Mat &xyz);
        void showImage(cv::Mat &src);
        cv::Point2f calculate_rotated_point(
            const cv::Point2f &center,
            const cv::Point2f &reference,
            double angle_offset = -M_PI / 3.0 // 此处指定默认值
        );
    };
}