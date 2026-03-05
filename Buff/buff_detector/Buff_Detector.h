// #pragma once
#ifndef BUFF_DETECTOR_H
#define BUFF_DETECTOR_H
#include <future>
#include <vector>

#include <opencv2/opencv.hpp>
// #include "./inference_api2.h"
#include "./inference_api2.h"
#include "../../general.h"
namespace buff_detector
{
    class BuffDetector
    {
        int debug;
        int enemy;
        std::unique_ptr<rm_buff::Detector> detector_ = std::make_unique<rm_buff::Detector>();
        std::vector<rm_buff::BuffObject> Objects;
        std::string network_path = "../Buff/buff_detector/model/yolox.xml";
        // std::string network_path = "../Buff/buff_detector/model/yolox_4.28.xml";
        std::string init_config_path = "../Buff/buff_detector/init_config.yaml";
        // 帧率计算
        std::chrono::steady_clock::time_point start_time_, now_time_;
        std::chrono::duration<double> elapsed_time_;
        double frame_rate_ = 0;
        int frame_rate_flag_ = 0;
        std::map<std::string, double> dictionary;

    public:
        BuffDetector();
        ~BuffDetector();
        void InitConfig();
        bool detectbuff(CameraData &cameradata, BuffTaskData &buffdata);
        void drawBuff(cv::Point2f apex2d[5], cv::Mat &src, int num);
        void drawFrameRate(cv::Mat &src);
    };
}
#endif