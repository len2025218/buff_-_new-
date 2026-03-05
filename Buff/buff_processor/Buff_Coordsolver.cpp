#include "./Buff_Coordsolver.h"
namespace buff_processor
{
    coordsolver::coordsolver()
    {
        InitConfig();
        // std::cout << "后端参数初始化完成" << std::endl;
    }
    coordsolver::~coordsolver()
    {
    }

    void coordsolver::InitConfig()
    {
        cv::FileStorage config(coord_path, cv::FileStorage::READ);

        auto read_vector = config["T_ci"];
        Eigen::MatrixXd mat_ci(4, 4);
        initMatrix(mat_ci, read_vector);
        T_ci_ = mat_ci;

        read_vector = config["T_ic"];
        Eigen::MatrixXd mat_ic(4, 4);
        initMatrix(mat_ic, read_vector);
        T_ic_ = mat_ic;

        read_vector = config["T_iw"];
        Eigen::MatrixXd mat_iw(1, 3);
        initMatrix(mat_iw, read_vector);
        T_iw_ = mat_iw.transpose();

        read_vector = config["T_cc"];
        Eigen::MatrixXd mat_cam(3, 3);
        initMatrix(mat_cam, read_vector);
        T_cc_ = mat_cam;

        config["Intrinsic_Matrix"] >> intrinstic; // 内参
        config["Distortion_Matrix"] >> coeff;     // 畸变

        config.release();
    }

    bool coordsolver::initMatrix(Eigen::MatrixXd &matrix, cv::FileNode vector)
    {
        int cnt = 0;
        for (int row = 0; row < matrix.rows(); row++)
        {
            for (int col = 0; col < matrix.cols(); col++)
            {
                matrix(row, col) = vector[cnt];
                cnt++;
            }
        }
        return true;
    }

    coordsolver::PnPInfo coordsolver::pnp(const std::vector<cv::Point2f> &points_pic, Eigen::Matrix3d &rmat_imu, int method = cv::SOLVEPNP_ITERATIVE)
    {
        // std::cout << "dian11" << points_pic[0] << std::endl;
        // std::cout << "dian22" << points_pic[1] << std::endl;
        // std::cout << "dian33" << points_pic[2] << std::endl;
        // std::cout << "dian44" << points_pic[3] << std::endl;
        // std::cout << "dian111" << points_world[0] << std::endl;
        // std::cout << "dian222" << points_world[1] << std::endl;
        // std::cout << "dian333" << points_world[2] << std::endl;
        // std::cout << "dian444" << points_world[3] << std::endl;
        // std::cout << "rmat_imu" << rmat_imu << std::endl;
        cv::Mat rvec = cv::Mat(1, 3, CV_64FC1);
        cv::Mat rmat = cv::Mat(3, 3, CV_64FC1);
        cv::Mat tvec = cv::Mat(1, 3, CV_64FC1);
        Eigen::Matrix3d rmat_eigen;
        Eigen::Vector3d R_center_world = {0, -0.7, -0.0};
        Eigen::Vector3d tvec_eigen;
        cv::solvePnP(points_world, points_pic, intrinstic, coeff, rvec, tvec, false, method);
        cv::Rodrigues(rvec, rmat);
        std::cout << "rmat" << rmat << std::endl;
        cv2eigen(rmat, rmat_eigen);
        std::cout << "rmat_eigen" << rmat_eigen << std::endl;
        cv2eigen(tvec, tvec_eigen);
        std::cout << "tvec_eigen" << tvec_eigen << std::endl;
        /////////////////////////
        result.rvec = rvec;
        result.tvec = tvec;

        result.armor_cam = tvec_eigen;
        result.armor_world = camToWorld(result.armor_cam, rmat_imu);
        result.R_cam = (rmat_eigen * R_center_world) + tvec_eigen;
        result.R_world = camToWorld(result.R_cam, rmat_imu);
        // Eigen::Matrix3d rmat_eigen_world = rmat_imu * (T_ic_.block(0, 0, 3, 3) * rmat_eigen);
        // std::cout << "rmat_eigen_world" << rmat_eigen << std::endl;
        // result.euler = rotationMatrixToEulerAngles(rmat_eigen_world);
        result.rmat = rmat_eigen;
        // result.rmat = rmat_eigen_world;

        return result;
    }
    /**
     * @brief 相机坐标系转到世界坐标系
     * @param T_ic imu转到车辆坐标系
     */
    Eigen::Vector3d coordsolver::camToWorld(const Eigen::Vector3d &point_camera, const Eigen::Matrix3d &rmat)
    {
        Eigen::Vector4d point_camera_tmp;
        Eigen::Vector4d point_imu_tmp;
        Eigen::Vector3d point_imu;
        Eigen::Vector3d point_world;

        point_camera_tmp << point_camera[0], point_camera[1], point_camera[2], 1;
        point_imu_tmp = T_ic_ * point_camera_tmp;
        point_imu << point_imu_tmp[0], point_imu_tmp[1], point_imu_tmp[2];
        // T_iw_从 中减去从 IMU 框架到世界框架（由 定义）的平移point_imu，从而有效地将其转换为世界坐标系。
        point_imu -= T_iw_;
        return rmat * point_imu;
    }

    Eigen::Vector3d coordsolver::worldToCam(const Eigen::Vector3d &point_world, const Eigen::Matrix3d &rmat)
    {
        Eigen::Vector4d point_camera_tmp;
        Eigen::Vector4d point_imu_tmp;
        Eigen::Vector3d point_imu;
        Eigen::Vector3d point_camera;

        // 该点旋转至IMU坐标。
        point_imu = rmat.transpose() * point_world;
        // 添加了从世界到 IMU 框架的转换。
        point_imu += T_iw_;
        // 准备齐次坐标：3D点转换为4D，以进行矩阵运算。
        point_imu_tmp << point_imu[0], point_imu[1], point_imu[2], 1;
        // IMU 到相机的转换：应用从 IMU 到相机坐标的转换。
        point_camera_tmp = T_ci_ * point_imu_tmp;
        // 提取相机坐标：检索相机坐标中得到的 3D 点。
        point_camera << point_camera_tmp[0], point_camera_tmp[1], point_camera_tmp[2];

        return point_camera;
    }
    cv::Point2f coordsolver::reproject(Eigen::Vector3d &xyz)
    {
        auto result = (1.f / xyz[2]) * T_cc_ * (xyz); // 解算前进行单位转换
        return cv::Point2f(result[0], result[1]);
    }
    cv::Point2f coordsolver::reproject(cv::Mat &xyz)
    {
        cv::Mat result = (1.f / xyz.at<double>(0, 2) * intrinstic * xyz);
        // auto result = (1.f / xyz[2]) * camera_matrix_ * (xyz); // 解算前进行单位转换
        return cv::Point2f(result.at<double>(0, 0), result.at<double>(0, 1));
    }

    Eigen::Vector2d coordsolver::getAttitude(Eigen::Vector3d &xyz_cam, Eigen::Matrix3d &rmat)
    {
        Eigen::Vector3d xyz = xyz_cam + xyz_offset;
        auto xyz_world = camToWorld(xyz, rmat);
        auto angle_cam = gainAngle(xyz);
        auto pitch_offset = gainBallisticPitch(xyz_world);
        angle_cam[1] = angle_cam[1] + pitch_offset;
        //绝对角pitch
        angle_cam[1] = angle_cam[1] + rotationMatrixToEulerAngles(rmat)(1, 0) * (180 / CV_PI);
        angle_cam += angle_offset;

        return angle_cam;
    }

    /**
 * @brief 将旋转矩阵转化为欧拉角
 * @param R 旋转矩阵
 * @return 欧拉角
*/
Eigen::Vector3d coordsolver::rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {z, y, x}; // yaw,pitch,roll
}

    inline double coordsolver::gainBallisticPitch(Eigen::Vector3d &xyz)
    {
        // TODO:根据陀螺仪安装位置调整距离求解方式
        // 降维，坐标系Y轴以垂直向上为正方向
        auto dist_vertical = xyz[2];
        auto vertical_tmp = dist_vertical;
        auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
        // auto dist_vertical = xyz[2];
        // auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
        double pitch = atan(dist_vertical / dist_horizonal) * 180 / CV_PI;
        auto pitch_new = pitch;
        // auto pitch_offset = 0.0;
        // 开始使用龙格库塔法求解弹道补偿
        for (int i = 0; i < max_iter; i++)
        {
            // R_K_iter = 50;
            // TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
            // 初始化
            auto x = 0.0;
            auto y = 0.0;
            auto p = tan(pitch_new / 180 * CV_PI);
            auto v = bullet_speed;
            auto u = v / sqrt(1 + pow(p, 2));
            auto delta_x = dist_horizonal / R_K_iter;
            for (int j = 0; j < R_K_iter; j++)
            {
                auto k1_u = -k * u * sqrt(1 + pow(p, 2));
                auto k1_p = -g / pow(u, 2);
                auto k1_u_sum = u + k1_u * (delta_x / 2);
                auto k1_p_sum = p + k1_p * (delta_x / 2);

                auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
                auto k2_p = -g / pow(k1_u_sum, 2);
                auto k2_u_sum = u + k2_u * (delta_x / 2);
                auto k2_p_sum = p + k2_p * (delta_x / 2);

                auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
                auto k3_p = -g / pow(k2_u_sum, 2);
                auto k3_u_sum = u + k3_u * (delta_x / 2);
                auto k3_p_sum = p + k3_p * (delta_x / 2);

                auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
                auto k4_p = -g / pow(k3_u_sum, 2);

                u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
                p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

                x += delta_x;
                y += p * delta_x;
            }
            // 评估迭代结果,若小于迭代精度需求则停止迭代
            auto error = dist_vertical - y;
            if (abs(error) <= stop_error)
            {
                break;
            }
            else
            {
                vertical_tmp += error;
                // xyz_tmp[1] -= error;
                pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / CV_PI;
            }
        }
        return pitch_new - pitch;
    }

    inline Eigen::Vector2d coordsolver::gainAngle(Eigen::Vector3d &xyz)
    {
        Eigen::Vector2d angle;
        // Yaw(逆时针)
        // Pitch(目标在上方为正)
        angle << gainYaw(xyz), gainPitch(xyz);
        return angle;
    }

    inline double coordsolver::gainYaw(Eigen::Vector3d &xyz)
    {
        return atan2(xyz[0], xyz[2]) * 180 / CV_PI;
    }

    inline double coordsolver::gainPitch(Eigen::Vector3d &xyz)
    {
        return -(atan2(xyz[1], sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])) * 180 / CV_PI);
        // return (atan2(xyz[1], sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])) * 180 / CV_PI);
    }

}