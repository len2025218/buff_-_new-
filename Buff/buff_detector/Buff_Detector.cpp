#include "./Buff_Detector.h"
namespace buff_detector
{
    BuffDetector ::BuffDetector()
    {
        InitConfig();
        // std::cout << "前端已获得参数" << std::endl;
        detector_->initModel(network_path);
    }

    BuffDetector::~BuffDetector()
    {
    }
    void BuffDetector::InitConfig()
    {
        cv::FileStorage fs(init_config_path, cv::FileStorage::READ);

        fs["enemy"] >> enemy;
        fs["debug"] >> dictionary["debug"];
        fs.release();
        // std::cout << "敌方颜色" << enemy << std::endl;
    }
    bool BuffDetector::detectbuff(CameraData &cameradata, BuffTaskData &buffdata)
    {
        if (dictionary["debug"] == 1)
        {
            InitConfig();
        }
        std::vector<rm_buff::BuffObject> objects;
        cv::Mat src;
        src = cameradata.mat;

        auto start_time = std::chrono::steady_clock::now();

        if (detector_->detect(src, objects))
        {
            auto end_time = std::chrono::steady_clock::now();
            // 计算耗时（单位：毫秒）
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            char time_text[32];
            sprintf(time_text, "Detect: %ld ms", duration);
            cv::putText(src, time_text, cv::Point(100, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 1);
            if (objects.size() > 0)
            {

                for (auto object : objects)
                {

                    if (object.color == enemy)
                    {
                        drawBuff(object.apex, src, object.cls);
                        // cv::imshow("大复",src);
                        // cv::imwrite("output.jpg", src);
                        cv::waitKey(1);
                        if (object.cls == 0)
                        {
                            buffdata.objects.push_back(object);
                        }

                        buffdata.mat = src;
                        buffdata.timestamp_cam = cameradata.timestamp_cam;
                    }
                }
            }
            else
            {
                return false;
            }
        }
        drawFrameRate(src);
        return true;
    }

    void BuffDetector::drawBuff(cv::Point2f apex2d[5], cv::Mat &src, int num)
    {
        if (num)
        {
            // 已打击
            for (int i = 0; i < 5; i++)
            {
                line(src, apex2d[i % 5], apex2d[(i + 1) % 5], {255, 255, 255}, 1, cv::LINE_AA);
            }
            cv::putText(src, "activated", apex2d[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
        }
        else
        {
            // 带打击
            for (int i = 0; i < 5; i++)
            {
                line(src, apex2d[i % 5], apex2d[(i + 1) % 5], {0, 255, 0}, 1, cv::LINE_AA);
            }
            cv::putText(src, "target", apex2d[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1, cv::LINE_AA);
        }
    }

    void BuffDetector::drawFrameRate(cv::Mat &src)
    {
        // 定义计时器
        now_time_ = std::chrono::steady_clock::now();

        // 计算帧率
        if (frame_rate_flag_ == 7)
        {
            frame_rate_flag_ = 0;
            elapsed_time_ = now_time_ - start_time_;
            double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time_).count();
            frame_rate_ = 7.0 / elapsed_seconds;
        }

        char FPS[11];
        sprintf(FPS, "%.1f", frame_rate_);

        std::string fps = FPS;
        cv::putText(
            src, "fps:" + fps, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 1);
        cv::putText(
            src, "fps:" + fps, cv::Point(21, 21), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 1);
        frame_rate_flag_++;
        start_time_ = now_time_;
        // std::cout << "fps:    " << frame_rate_ << std::endl;
    }

}