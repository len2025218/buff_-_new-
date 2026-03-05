#ifndef BUFF_DETECOTR__INFERENCE_API2_HPP_
#define BUFF_DETECOTR__INFERENCE_API2_HPP_

// C++
#include <iterator>

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>
#include <unistd.h>
#include <future>
#include <fstream>
#include <yaml-cpp/yaml.h>

// opencv
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

// eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
namespace rm_buff
{
    struct Object
    {
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };
    struct BuffObject : Object
    {
        cv::Point2f apex[5];
        float cls_prob[2]; // cls_prob[0]=待打击概率, cls_prob[1]=已打击概率
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    class Detector
    {
    public:
        Detector();
        ~Detector();

        bool detect(cv::Mat &src, std::vector<BuffObject> &objects);
        bool initModel(std::string path);

    private:
        int dw, dh;
        float rescale_ratio;

        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;

        std::string input_name;
        std::string output_name;

        Eigen::Matrix<float, 3, 3> transfrom_matrix;
    };
}

#endif