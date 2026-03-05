// #pragma once 
#ifndef GENERAL_H
#define GENERAL_H
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include"./Buff/buff_detector/inference_api2.h"
// 电控数据
typedef struct MCUData {
    unsigned char mode; // 模式
    unsigned char change_flag; // 是否统一时间戳的标志位
    float timestamp; // 时间戳
    float shoot_speed; // 射速
    float quat[4]; // 四元数
} MCUData;

// 视觉数据
typedef struct VisionData {
    bool isShooting;    // 是否开火（自动开火控制）
    float yaw_angle;   // 偏航角
    float pitch_angle; // 俯仰角
    bool is_detect;     // 视觉是否检测到合适的目标
    bool is_spinning;   // 敌方车辆是否处于小陀螺状态 
    float measure_yaw; // 测量值yaw
    float measure_pitch; // 测量值pitch
    float delay;
    float distance;
    float x;
    float y;
    float z;
    float image_point_x;
    float image_point_y;
    bool hand; // 告诉操作手手动开火
} VisionData;

struct ArmorObject {
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};

// 相机传给前端
struct CameraData {
    cv::Mat mat;
    std::chrono::steady_clock::time_point timestamp_cam;
    MCUData mcudata;
};

// 前端传给后端
struct TaskData {
    cv::Mat mat;
    std::chrono::steady_clock::time_point timestamp_cam;
    std::vector<ArmorObject> objects;
    bool is_detect;
    MCUData mcudata;
};



// //大符相机传前端
// struct BuffCameraData{
//     cv::Mat mat;
//     std::chrono::steady_clock::time_point timestamp_cam;
// }

//大符前端传后端
struct BuffTaskData{
    cv::Mat mat;
    MCUData mcudata;
    std::chrono::steady_clock::time_point timestamp_cam; 
    // cv::Point2f apex[4];
    std::vector<rm_buff::BuffObject> objects;
};

// 装甲板结构体
struct Armor {
    cv::Point2f apex2d[4];
    cv::Rect rect;
    cv::Point2f center2d;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    int cls;
    double distance_to_image_center;
    std::string type;
    Eigen::Matrix3d rmat_eigen;
    // 重载 '==' 运算符
    bool operator==(const Armor& other) const {
        for (int i = 0; i < 4; ++i) {
            if (apex2d[i] != other.apex2d[i]) return false;
        }
        if (rect != other.rect) return false;
        if (center2d != other.center2d) return false;
        if (center3d_cam != other.center3d_cam) return false;
        if (center3d_world != other.center3d_world) return false;
        if (euler != other.euler) return false;
        if (cls != other.cls) return false;
        if (distance_to_image_center != other.distance_to_image_center) return false;
        if (type != other.type) return false;
        return rmat_eigen.isApprox(other.rmat_eigen); // 使用 isApprox 比较矩阵
    }
};

// 后端内部传递给processor
struct Armors {
    std::vector<Armor> armors;
    std::chrono::steady_clock::time_point timestamp_cam;
    cv::Mat mat;
};

struct PnPInfo
{
    Eigen::Vector3d armor_cam;
    Eigen::Vector3d armor_world;
    Eigen::Vector3d euler;
    Eigen::Matrix3d rmat_eigen;
};

enum Last_cls {
    Hero,    
    Infantry,  
    Sentry    
};

#endif