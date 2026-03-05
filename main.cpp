//视频测试
#include "./Buff/buff_detector/Buff_Detector.h"
#include "./Buff/buff_processor/Buff_Processor.h"
#include "./serial/serialport.h"

#include <thread>
#include <atomic>
#include <csignal>  // 用于捕获退出信号

// 全局退出标志（支持手动终止程序）
std::atomic<bool> g_quit(false);

// 全局模式控制（1=小符，2=大符）
std::atomic<int> current_mode(2);

// 全局变量
std::mutex mutex_camera;
std::mutex mutex_task;
std::mutex mutex_vision;
std::mutex mutex_mcu;
std::deque<CameraData> buffer;
std::deque<BuffTaskData> task_datas;
std::deque<VisionData> vision_data;
std::deque<MCUData> mcu_data;
std::atomic<int> flag_detect(0);
std::atomic<int> flag_autoaim(0);
std::atomic<int> flag_send(0);
std::atomic<int> flag_receive(0);

/**
 * 信号处理：捕获 Ctrl+C 手动退出程序
 */
void signalHandler(int signum) {
    std::cout << "\n收到退出信号（Ctrl+C），正在终止程序..." << std::endl;
    g_quit = true;  // 设置退出标志，所有线程会响应
}

/**
 * 生产者线程：视频循环播放（核心修改）
 */
bool producer() {
    // 打开视频文件
    // cv::VideoCapture cap("/home/len/下载/copy_9C6F465F-8452-420D-B003-C6D8D361D1F6.MP4");
    // cv::VideoCapture cap("/home/len/TUP_VISION_2025-main视频3/src/mmexport1742446806527.mp4");
    // cv::VideoCapture cap("/home/len/下载/copy_3A3F94B6-8B93-43EF-A3F7-FEF747AC3AD4.MP4");//小
    // cv::VideoCapture cap("/home/len/TUP_VISION_2025-main视频3/src/output_videonew.mp4");//大
    cv::VideoCapture cap("/home/len/buff_多线程_new（另一个复件）/copy_D13FEC9D-7087-489A-BE95-F6FBC9A66976.MP4");
    if (!cap.isOpened()) {
        std::cerr << "Error: 视频打开失败！\n";
        return false;
    }

    // 打印当前模式和操作提示
    std::cout << "===== 当前固定模式：" << (current_mode == 1 ? "小符模式" : "大符模式") << " =====" << std::endl;
    std::cout << "视频将持续循环播放，按 Ctrl+C 退出程序" << std::endl;

    cv::Mat frame;
    while (!g_quit) {  // 循环播放直到手动退出
        // 读取视频帧：失败表示播放到末尾，重新定位到开头
        if (!cap.read(frame)) {
            std::cout << "视频播放完毕，重新开始循环..." << std::endl;
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);  // 关键：定位到视频第0帧
            continue;  // 重新读取帧
        }

        // 构造相机数据
        CameraData src;
        src.mat = frame;
        src.timestamp_cam = std::chrono::steady_clock::now();
        
        // 构造MCU数据（传递固定模式）
        MCUData mcudata;
        mcudata.mode = current_mode;  // 传入当前模式
        float quat[4] = {1, 0, 0, 0};
        std::memcpy(mcudata.quat, quat, sizeof(quat));
        mcudata.shoot_speed = 27;
        mcudata.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            src.timestamp_cam.time_since_epoch()
        ).count();
        src.mcudata = mcudata;

        // 数据入队（线程安全）
        if (buffer.empty() || flag_detect) {
            std::lock_guard<std::mutex> lock(mutex_camera);
            buffer.push_back(src);
        }

        // 控制播放速度
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
    }

    // 手动退出时释放资源
    cap.release();
    std::cout << "[生产者线程] 退出" << std::endl;
    return true;
}

/**
 * 消费者线程：持续检测目标（响应退出信号）
 */
bool consumer() {
    buff_detector::BuffDetector buff_detector;
    while (!g_quit) {  // 仅在手动退出时终止
        if (buffer.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 取出相机数据
        CameraData src;
        {
            std::lock_guard<std::mutex> lock(mutex_camera);
            if (buffer.empty()) continue;
            src = buffer.back();
            buffer.clear();
        }
        flag_detect = 1;

        // 检测目标（根据模式筛选）
        BuffTaskData task_data;
        buff_detector.detectbuff(src, task_data);

        // 填充任务数据
        task_data.mat = src.mat;
        task_data.timestamp_cam = src.timestamp_cam;
        task_data.mcudata = src.mcudata;

        // 任务数据入队
        if (task_datas.empty() || flag_autoaim) {
            std::lock_guard<std::mutex> lock(mutex_task);
            task_datas.push_back(task_data);
        }

        flag_detect = 0;
    }

    std::cout << "[消费者线程] 退出" << std::endl;
    return true;
}

/**
 * 处理器线程：持续处理数据（响应退出信号）
 */
bool processor() {
    buff_processor::BuffProcessor autoaim;
    while (!g_quit) {  // 仅在手动退出时终止
        flag_autoaim = 0;
        if (task_datas.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 取出任务数据
        BuffTaskData task_data;
        {
            std::lock_guard<std::mutex> lock(mutex_task);
            if (task_datas.empty()) continue;
            task_data = task_datas.back();
            task_datas.clear();
        }
        flag_autoaim = 1;

        // 处理数据（根据模式切换逻辑）
        VisionData visiondata = {};
        autoaim.processArmors(task_data, task_data.mcudata, visiondata);

        // 预测结果入队
        if (vision_data.empty() || flag_send) {
            std::lock_guard<std::mutex> lock(mutex_vision);
            vision_data.push_back(visiondata);
        }
    }

    std::cout << "[处理器线程] 退出" << std::endl;
    return true;
}

int main(int argc, char **argv) {
    // 注册信号处理：支持 Ctrl+C 手动退出
    signal(SIGINT, signalHandler);

    // 初始化日志
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = false;
    FLAGS_colorlogtostderr = true;
    google::SetLogDestination(google::GLOG_INFO, "../Log/info/");
    google::SetLogDestination(google::GLOG_WARNING, "../Log/warning/");
    google::SetLogDestination(google::GLOG_ERROR, "../Log/error/");

    // 启动线程
    std::thread produce_task(producer);
    LOG(INFO) << "[MAIN] 生产者线程启动!";
    std::thread consume_task(consumer);
    LOG(INFO) << "[MAIN] 消费者线程启动!";
    std::thread processor_task(processor);
    LOG(INFO) << "[MAIN] 处理器线程启动!";

    // 等待线程退出（仅手动退出时触发）
    produce_task.join();
    consume_task.join();
    processor_task.join();

    // 关闭日志
    google::ShutdownGoogleLogging();
    LOG(INFO) << "[MAIN] 程序退出";
    return 0;
}

// //串口相机测试
// #include "./Buff/buff_detector/Buff_Detector.h"
// #include "./Buff/buff_processor/Buff_Processor.h"
// #include "./camera/MindVisionCamera.h"
// #include "./serial/serialport.h"

// #include <thread>

// std::mutex mutex_camera;
// std::mutex mutex_task;
// std::mutex mutex_vision;
// std::mutex mutex_mcu;
// std::deque<CameraData> buffer; // 相机传给前端
// std::deque<BuffTaskData> task_datas; // 前端传给后端
// std::deque<VisionData> vision_data;
// std::deque<MCUData> mcu_data;
// int flag_detect = 0;
// int flag_autoaim = 0;
// int flag_send = 0;
// int flag_receive = 0;

// bool producer() {
//     mindvision_camera::MVCamera mindvision;
//     mindvision.InitConfig();
//     mindvision.InitCamera();
//     mindvision.SetParam();
//     std::time_t initaltime;
//     mindvision.GetFileChangeTime(initaltime);
//     while (1) {
//         cv::Mat mat;
//         mindvision.DynamicParameter(initaltime);
//         mindvision.GetMat(mat);
//         if (mat.empty()) {
//             mindvision.ReConnect();
//         }
//         mindvision.GetMat(mat);
//         // cv::flip(mat, mat, -1);
//         CameraData src;
//         src.mat = mat;
//         cv::imshow("1111",mat);
//         std::chrono::milliseconds duration(60);
//         std::this_thread::sleep_for(duration);
//         waitKey(1);
//         src.timestamp_cam = std::chrono::steady_clock::now();
//         MCUData mcudata;
//         flag_receive = 0;
//         if (!mcu_data.empty()) {
//             mutex_mcu.lock();
//             if (!mcu_data.empty()) {
//                 mcudata = mcu_data.back();
//                 mcu_data.clear();
//                 mutex_mcu.unlock();
//                 flag_receive = 1;
//                 src.mcudata = mcudata;
//                 if (buffer.size() == 0) {
//                     mutex_camera.lock();
//                     buffer.push_back(src);
//                     mutex_camera.unlock();
//                 } else if (buffer.size() != 0 && flag_detect) {
//                     mutex_camera.lock();
//                     buffer.push_back(src);
//                     mutex_camera.unlock();
//                 }
//             }
//         }
//     }
//     return 0;
// }

// bool consumer() {
//     buff_detector::BuffDetector buff_detector;
//     while (1) {
//         if (!buffer.empty()) {
//             mutex_camera.lock();
//             if (!buffer.empty()) {
//                 CameraData src = buffer.back();
//                 buffer.clear();
//                 mutex_camera.unlock();
//                 flag_detect = 1;
//                 BuffTaskData task_data;
//                 buff_detector.detectbuff(src, task_data);
//                 task_data.mat = src.mat;
//                 task_data.timestamp_cam = src.timestamp_cam;
//                 task_data.mcudata = src.mcudata;
//                 if (task_datas.size() == 0){
//                     mutex_task.lock();
//                     task_datas.push_back(task_data);
//                     mutex_task.unlock();
//                 } else if (task_datas.size() != 0 && flag_autoaim) {
//                     mutex_task.lock();
//                     task_datas.push_back(task_data);
//                     mutex_task.unlock();
//                 }
//                 flag_detect = 0;
//             }
//         }
//     }
// }

// bool processor() {
//     buff_processor::BuffProcessor autoaim;
//     while(1) {
//         flag_autoaim = 0;
//         if (!task_datas.empty()) {
//             mutex_task.lock();
//             if (!task_datas.empty()) {
//                 BuffTaskData task_data = task_datas.back();
//                 task_datas.clear();
//                 mutex_task.unlock();
//                 flag_autoaim = 1;
//                 VisionData visiondata = {};
//                 autoaim.processArmors(task_data, task_data.mcudata, visiondata);
//                 // std::cout << "==========================================" << std::endl;
//                 // std::cout << "visiondata.is_detect" << visiondata.is_detect << std::endl;
//                 // std::cout << "visiondata.is_spinning" << visiondata.is_spinning << std::endl;
//                 // std::cout << "visiondata.isShooting" << visiondata.isShooting << std::endl;
//                 // std::cout << "visiondata.pitch_angle" << visiondata.pitch_angle << std::endl;
//                 // std::cout << "visiondata.yaw_angle" << visiondata.yaw_angle << std::endl;
//                 if (vision_data.size() == 0) {
//                     mutex_vision.lock();
//                     vision_data.push_back(visiondata);
//                     mutex_vision.unlock();
//                 } else if (vision_data.size() != 0 && flag_send) {
//                     mutex_vision.lock();
//                     vision_data.push_back(visiondata);
//                     mutex_vision.unlock();
//                 }
//             }
//         }
//     }
// }

// // 发送数据线程
// void dataSender(SerialPort &serialport) {
//     while (1) {
//         if (!vision_data.empty()) {
//             mutex_vision.lock();
//             if (!vision_data.empty()) {
//                 VisionData visiondata = vision_data.back();
//                 vision_data.clear();
//                 mutex_vision.unlock();
//                 flag_send = 1;

//                 serialport.sendData(visiondata);
//                 std::cout<<"pitch----------------(((((((((((((((((())))))))))))))))))"<<visiondata.pitch_angle<<std::endl;
//                 std::cout<<"yaw----------------(((((((((((((((((())))))))))))))))))"<<visiondata.yaw_angle<<std::endl;
//                 flag_send = 0;
//             }
//         }
//     }
// }

// // 接受线程
// // 接受数据线程
// void dataReceiver(SerialPort &serialport) {
//     while(1) {
//         MCUData mcudata;
//         serialport.receiveData(mcudata);
//         if (mcu_data.size() == 0) {
//             mutex_mcu.lock();
//             mcu_data.push_back(mcudata);
//             mutex_mcu.unlock();
//         } else if (mcu_data.size() != 0 && flag_receive) {
//             mutex_mcu.lock();
//             mcu_data.push_back(mcudata);
//             mutex_mcu.unlock();
//         }
//     }
// }

// int main(int argc, char **argv) {
//     google::InitGoogleLogging(argv[0]);
//     FLAGS_alsologtostderr = false;  //除了日志文件之外是否需要标准输出
//     FLAGS_colorlogtostderr = true;  //是否启用不同颜色显示
//     google::SetLogDestination(google::GLOG_INFO,"../Log/info/");  //设置日志级别
//     google::SetLogDestination(google::GLOG_WARNING,"../Log/warning/");
//     google::SetLogDestination(google::GLOG_ERROR,"../Log/error/");
//     std::thread produce_task(producer);
//     LOG(INFO) << "[MAIN] task_producer start!";
//     std::thread consume_task(consumer);
//     LOG(INFO) << "[MAIN] task_consumer start!";
//     std::thread processor_task(processor);
//     LOG(INFO) << "[MAIN] task_consumer start!";
//     SerialPort serialport;
//     std::thread send_task(dataSender, std::ref(serialport));
//     LOG(INFO) << "[MAIN] data_sender start!";
//     std::thread receive_task(dataReceiver, std::ref(serialport));
//     LOG(INFO) << "[MAIN] data_receiver start!";
//     produce_task.join();
//     consume_task.join();
//     processor_task.join();
//     google::ShutdownGoogleLogging();
//     return 0;
// }
