#include "./Buff_Processor.h"
namespace buff_processor
{
    BuffProcessor::BuffProcessor()
    {
        predictor_ = std::make_unique<rm_buff::Predictor>();
        coordsolver_ = std::make_unique<coordsolver>();
        InitConfig();
    }
    BuffProcessor::~BuffProcessor()
    {
    }
    bool BuffProcessor::InitConfig()
    {
        cv::FileStorage init_config(init_config_path, cv::FileStorage::READ);
        if (!init_config.isOpened())
        {
            std::cout << "无法打开配置文件" << std::endl;
            return false;
        }

        init_config["debug"] >> dictionary["debug"];
        cv::FileNode xyz_offset_node = init_config["xyz_offset"]; // 获取xyz_offset节点的子节点x、y、z的值
        coordsolver_->xyz_offset = Eigen::Vector3d{(double)xyz_offset_node["x"], (double)xyz_offset_node["y"], (double)xyz_offset_node["z"]};
        cv::FileNode angle_offset_node = init_config["angle_offset"]; // 获取angle_offset节点的子节点pitch、yaw的值
        coordsolver_->angle_offset = Eigen::Vector2d{(double)angle_offset_node["yaw"], (double)angle_offset_node["pitch"]};
        // 跟踪器参数
        cv::FileNode tracker = init_config["tracker"];
        dictionary["max_delta_t"] = (double)tracker["max_delta_t"];
        dictionary["max_delta_t"] = dictionary["max_delta_t"] * 1e9;
        dictionary["buff_max_v"] = (double)tracker["buff_max_v"];
        // 预测参数
        cv::FileNode pred = init_config["predictor"];
        predictor_->delay_big_ = (double)pred["delay_big"];
        predictor_->delay_small_ = (double)pred["delay_small"];
        auto read_vector = pred["params_bound"];
        Eigen::MatrixXd mat_iw(2, 4);
        coordsolver_->initMatrix(mat_iw, read_vector);
        // std::cout << mat_iw << std::endl;
        for (int i = 0; i < 4; i++)
        {
            predictor_->param_bound_[2 * i] = mat_iw(0, i);
            predictor_->param_bound_[2 * i + 1] = mat_iw(1, i);
        }
        init_config.release();
        return true;
    }

    bool BuffProcessor::processArmors(BuffTaskData &buffdata, MCUData &mcudata, VisionData &visiondata)
    {
        // showImage(buffdata.mat);
        if (buffdata.objects.size() == 0)
        {
            std::cout << "未检测到" << std::endl;
            visiondata.is_detect = 0;
            return false;
        }
        else
        {
            visiondata.is_detect = 1;
        }
        // cv::Point2f center_;
        // cv::Point2f center_new;
        // center_ = (buffdata.objects[0].apex[1] + buffdata.objects[0].apex[3]) / 2;
        // center_new = calculate_rotated_point(center_, buffdata.objects[0].apex[0], -M_PI / 3.0);
        // cv::circle(buffdata.mat, center_new, 3, {255, 0, 0}, -1);
        // int mode = mcudata.mode;
        // cv::putText(buffdata.mat, std::to_string(mode), cv::Point(75, 47), cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
        // cv::putText(buffdata.mat, "mode[ ]", cv::Point(7, 47), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 0, 255}, 2.3);
        cout << "弹速" << mcudata.shoot_speed << std::endl;
        if (mcudata.shoot_speed >= 18 && mcudata.shoot_speed <= 30)
        {
            coordsolver_->bullet_speed = (coordsolver_->bullet_speed + mcudata.shoot_speed) / 2.0; // 射击延迟
            predictor_->bullet_speed_ = mcudata.shoot_speed;
        }
        if (dictionary["debug"])
        {
            InitConfig();
        }
        speed_[0] = 0;
        speed_[1] = 0;
        predict_[0] = 0;
        predict_[1] = 0;
        Eigen::Quaterniond quat = {mcudata.quat[0], mcudata.quat[1], mcudata.quat[2], mcudata.quat[3]};
        Eigen::Matrix3d rmat_imu = quat.toRotationMatrix();
        process2DPoints(buffdata, rmat_imu);
        center_xyz_.x() = pnp_result.tvec.at<double>(0);
        center_xyz_.y() = pnp_result.tvec.at<double>(1);
        center_xyz_.z() = pnp_result.tvec.at<double>(2);
        uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 buffdata.timestamp_cam.time_since_epoch())
                                 .count();
        double result = mcudata.mode;

        if (predictor_->update(speed_, center_xyz_.norm(), timestamp, result))
        {

            // std::cout << "initial_omega:    ++++++++++++++++" << predictor_->initial_omega << std::endl;
            std::cout << "result:    " << result << std::endl;
            ////////////
            ////跟随////
            ///////////
            //    result=0;

            Eigen::Vector3d hit_point_world = {sin(result) * fan_length_, (cos(result) - 1) * fan_length_, 0};
            center_xyz_ = pnp_result.rmat * hit_point_world + center_xyz_;
            double speed = speed_[0];
            double speed_1 = speed * (180.0 / M_PI);
            cv::putText(buffdata.mat, "|" + std::to_string(speed_1), cv::Point2f(72, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 2);
            speed = speed_[1];
            double speed_2 = speed * (180.0 / M_PI);
            cv::putText(buffdata.mat, "|" + std::to_string(speed_2), cv::Point2f(72, 86), cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 0, 255}, 2);
            std::cout << "------------------------------------------------------------------------------->" << std::endl;
            solveResult(visiondata, rmat_imu, buffdata);
        }
        showImage(buffdata.mat);
        return true;
    }
    // 调试用
    cv::Point2f BuffProcessor::calculate_rotated_point(
        const cv::Point2f &center,
        const cv::Point2f &reference,
        double angle_offset)
    {
        // 1. 平移坐标系到参考点
        const float dx = center.x - reference.x;
        const float dy = center.y - reference.y;

        // 2. 计算极坐标参数
        //// 计算半径
        const double radius = std::hypot(dx, dy);
        const double theta = std::atan2(dy, dx);

        // 3. 应用角度偏移
        const double new_theta = theta + angle_offset;

        // 4. 计算新坐标并返回
        return cv::Point2f(
            static_cast<float>(radius * std::cos(new_theta)) + reference.x,
            static_cast<float>(radius * std::sin(new_theta)) + reference.y);
    }

    void BuffProcessor::drawcenter(cv::Mat buff_center, cv::Mat &src, int type)
    {

        if (type == 2)
        {
            center_[0] = coordsolver_->reproject(buff_center); // 重投影
            cv::circle(src, center_[0], 7, {247, 247, 247}, 1);
            cv::circle(src, center_[0], 1, {247, 247, 247}, 2);
        }
        if (type == 0)
        {
            center_[1] = coordsolver_->reproject(buff_center); // 重投影
            cv::circle(src, center_[1], 1, {0, 255, 0}, -1);
        }
    }

    void BuffProcessor::process2DPoints(BuffTaskData &buffdata, Eigen::Matrix3d &rmat_imu)
    {
        cv::Mat buff_center = cv::Mat::zeros(3, 1, CV_32FC1);
        if (buffdata.objects.size() > 0)
        {
            std::cout << "buffdata.objects.size()" << buffdata.objects.size() << endl;
            for (const auto &object : buffdata.objects)
            {
                std::vector<Point2f> points_pic;
                points_pic.emplace_back(object.apex[0].x, object.apex[0].y);
                points_pic.emplace_back(object.apex[1].x, object.apex[1].y);
                points_pic.emplace_back(object.apex[2].x, object.apex[2].y);
                points_pic.emplace_back(object.apex[3].x, object.apex[3].y);
                points_pic.emplace_back(object.apex[4].x, object.apex[4].y);

                if (points_pic.size() < 5)
                { // PnP至少需要4个点
                    std::cout << "点数不够" << std::endl;
                    return;
                }
                cv::Mat imagePointsMat = cv::Mat(points_pic);
                if (imagePointsMat.type() != CV_32F)
                {
                    imagePointsMat.convertTo(imagePointsMat, CV_32F);
                }
                pnp_result = coordsolver_->pnp(imagePointsMat, rmat_imu, cv::SOLVEPNP_EPNP);
                // 装甲板中心
                drawcenter(pnp_result.tvec, buffdata.mat, object.cls);
                // std::cout << "444444444" << std::endl;

                ////////////////////
                ////////R点/////////
                ///////////////////
                // cv::Mat rotation_matrix;
                // cv::Rodrigues(pnp_result.rvec, rotation_matrix);
                // cv::Mat tvec_mat(3, 1, CV_32F, pnp_result.tvec);
                // buff_center += ((rotation_matrix * fan2center_) + pnp_result.tvec);

                uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   buffdata.timestamp_cam.time_since_epoch())
                                   .count();
                // 清理过期数据
                for (unsigned int i = 0; i < tracker_.size();)
                {
                    if ((now - tracker_[i].timestamp) > dictionary["max_delta_t"])
                    {
                        tracker_.erase(tracker_.begin() + i);
                    }
                    else
                    {
                        ++i;
                    }
                }
                Eigen::Matrix3d current_rotation = pnp_result.rmat;

                // 计算转速
                calculateSpinSpeed(now, current_rotation);
                //  创建 TrackedFan 对象并填充数据
                TrackedFan tracked_fan;
                tracked_fan.timestamp = now;             // 时间戳
                tracked_fan.rotation = current_rotation; // 旋转矩阵

                // 将当前姿态数据加入历史数据列表
                tracker_.push_back(tracked_fan);
            }
        }
        else
        {
            std::cout << "未检测到目标" << std::endl;
            return;
        }
    }

    void BuffProcessor::calculateSpinSpeed(uint64_t now, const Eigen::Matrix3d &current_rotation)
    {
        Eigen::Vector3d center_xyz;
        center_xyz.x() = pnp_result.tvec.at<double>(0);
        center_xyz.y() = pnp_result.tvec.at<double>(1);
        center_xyz.z() = pnp_result.tvec.at<double>(2);
        int match_num = 0;
        int update_num = -1;
        double min_v = DBL_MAX;
        int min_last_delta_t = INT_MAX;
        // 遍历历史数据，寻找匹配
        for (unsigned int k = 0; k < tracker_.size(); k++)
        {
            // 获取历史数据
            TrackedFan &historical_fan = tracker_[k];

            // 计算时间差
            double delta_t = now - historical_fan.timestamp;
            // 跳过无效时间差
            if (delta_t == 0)
            {
                std::cout << "时间差为零跳过" << std::endl;
                continue;
            }
            // 计算相对旋转矩阵
            Eigen::Matrix3d relative_rot = historical_fan.rotation.transpose() * current_rotation;
            Eigen::AngleAxisd angle_axisd(relative_rot);
            auto rotate_axis_world = current_rotation * angle_axisd.axis();
            int sign = (center_xyz.dot(rotate_axis_world) > 0) ? -1 : 1;
            double rotate_speed = sign * (angle_axisd.angle()) / (delta_t / 1e9);

            // 筛选最佳匹配
            if (abs(rotate_speed) <= abs(min_v) && abs(rotate_speed) <= dictionary["buff_max_v"] && delta_t <= min_last_delta_t)
            {
                update_num = k;
                min_v = rotate_speed;
                min_last_delta_t = delta_t;
            }
            if (min_v != DBL_MAX)
            {
                tracker_.erase(tracker_.begin() + update_num);
                speed_[0] += min_v;
                match_num++;
            }
        }

        // 输出结果
        if (match_num > 0)
        {
            speed_[0] = speed_[0] / match_num;
            std::cout << "[INFO] Average speed: "
                      << std::fixed << std::setprecision(2)
                      << speed_[0]
                      << " rad/s" << std::endl; // 输出：3.14
        }
        else
        {
            std::cout << "没有目标" << std::endl;
        }
    }
    void BuffProcessor::solveResult(VisionData &visiondata, Eigen::Matrix3d &rmat_imu, BuffTaskData &buffdata)
    {
        Eigen::Vector3d hit_point_cam = center_xyz_;
        std::cout << "hit_point_cam" << hit_point_cam << std::endl;
        calculate_ = coordsolver_->reproject(hit_point_cam);
        std::cout << "calculate_" << calculate_ << std::endl;
        cv::circle(buffdata.mat, calculate_, 3, {0, 255, 0}, -1);
        predict_ = coordsolver_->getAttitude(hit_point_cam, rmat_imu);
        visiondata.pitch_angle = predict_(1);
        // visiondata.yaw_angle = predict_(0);
        // 绝对角yaw
        float yaw_;
        Eigen::Vector3d center_xyz_world;
        center_xyz_world = coordsolver_->camToWorld(center_xyz_, rmat_imu);
        yaw_ = std::atan2(center_xyz_world(1, 0), center_xyz_world(0, 0)) * (180.0 / CV_PI) + coordsolver_->angle_offset(0);
        visiondata.yaw_angle = yaw_;
        cv::putText(buffdata.mat, "PRD:", cv::Point(7, 131), cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0}, 3);
        cv::putText(buffdata.mat, "pitch", cv::Point(87, 117), cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
        cv::putText(buffdata.mat, "|" + std::to_string(predict_(1)), cv::Point(147, 117), cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
        cv::putText(buffdata.mat, "yaw", cv::Point(87, 138), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 0}, 2);
        // cv::putText(buffdata.mat, "|" + std::to_string(predict_(0)), cv::Point(147, 138), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 0}, 2);
        cv::putText(buffdata.mat, "|" + std::to_string(yaw_), cv::Point(147, 138), cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 0}, 2);
    }

    void BuffProcessor::showImage(cv::Mat &src)
    {
        if (dictionary["debug"] == 1)
        {
            // 参数配置
            const std::string format = ".jpg";
            const int start_frame = 0;   // 起始帧
            const int save_interval = 1; // 保存间隔
            const int jpeg_quality = 95; // 质量参数

            // 初始化输出目录
            if (!fs::exists(output_dir))
            {
                fs::create_directories(output_dir);
            }

            // 帧计数器递增
            frame_count++;

            // 判断是否满足保存条件
            if (frame_count >= start_frame &&
                (frame_count - start_frame) % save_interval == 0)
            {

                // 生成格式化文件名
                std::ostringstream filename;
                filename << output_dir << "frame_"
                         << std::setw(4) << std::setfill('0') << saved_count
                         << format;

                // 保存图像（带质量参数）
                std::vector<int> params;
                params.push_back(cv::IMWRITE_JPEG_QUALITY);
                params.push_back(jpeg_quality);

                if (!cv::imwrite(filename.str(), src, params))
                {
                    std::cerr << "保存失败: " << filename.str()
                              << std::endl;
                }
                else
                {
                    std::cout << "已保存: " << filename.str() << std::endl;
                    saved_count++;
                }
            }
        }
        cv::namedWindow("buff_det_new", cv::WINDOW_NORMAL);
        cv::imshow("buff_det_new", src);
        cv::waitKey(1);
    }
}