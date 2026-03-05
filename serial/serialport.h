#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <fcntl.h>
#include <termios.h>
#include <mutex>

#include "./CRC_Check.h"
#include "./data_transform.h"
#include "../general.h"

extern std::mutex mut;

class SerialPort {
public:
    int fd;             // 串口号
    unsigned char rdata[30]; // 电控发来的原始数据
    unsigned char Tdata[43]; // 视觉要发给电控的原始数据
    void receiveData(MCUData &data); // 接收电控数据
    void sendData(const VisionData &data); // 视觉发送数据
    SerialPort();
    ~SerialPort();
private:    
    const int band = B115200; // 波特率
    std::string init_config_path = "../serial/init_config.yaml";
    bool initConfig();
    std::string port_name;
    std::string password;
    bool readData();
    bool openPort();
};


#endif