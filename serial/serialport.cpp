#include "serialport.h"
#include <thread>

SerialPort::SerialPort() {
    initConfig();
    while(!openPort()) {
        sleep(1);
    }
}

bool SerialPort::initConfig() {
    YAML::Node config = YAML::LoadFile(init_config_path);
    port_name = config["PorFile"].as<std::string>();
    password = config["password"].as<std::string>();
    return true;
}

// 初始化串口
bool SerialPort::openPort() {
    // 寻找串口
    if (access(port_name.c_str(), 0) == -1)
    {
        LOG(ERROR) << "[SERIAL]serial port not found";
        return false;
    }

    // 给串口权限
    if (system(("echo " + password + " | sudo -S chmod a+rw " + "/dev/tty* ").c_str()) != 0) // 给超级权限
    {
        LOG(ERROR) << "[SERIAL] Failed to get permission!";
        return false;
    }

    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY); // 接入串口
    if (fd == -1)
    {
        LOG(ERROR) << "[SERIAL] open port defeat";
        return false;
    }
    
    // 设置波特率
    struct termios Opt;
    tcgetattr(fd, &Opt);
    tcflush(fd, TCIOFLUSH);  // 清空缓冲区的内容
    cfsetispeed(&Opt, band); // 设置接受和发送的波特率
    cfsetospeed(&Opt, band);
    // 使设置立即生效
    if (tcsetattr(fd, TCSANOW, &Opt) != 0) 
    {
        LOG(ERROR) << "[SERIAL] set param fail";
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    // 设置数据位，停止位和校验位
    tcgetattr(fd, &Opt);
    Opt.c_cflag |= (CLOCAL | CREAD); // 接受数据
    Opt.c_cflag &= ~CSIZE;           // 设置数据位数
    Opt.c_cflag |= CS8;
    // 设置校验位（禁用）
    Opt.c_cflag &= ~PARENB; 
    Opt.c_iflag &= ~INPCK;  
    // 设置停止位
    Opt.c_cflag &= ~CSTOPB; // 1位
    tcflush(fd, TCIFLUSH); // 清除输入缓存区
    Opt.c_cc[VTIME] = 150; // 设置超时15 seconds
    Opt.c_cc[VMIN] = 0;  // 最小接收字符
    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入
	Opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    Opt.c_iflag &= ~(ICRNL | IGNCR);
    Opt.c_oflag &= ~OPOST;   // 禁用输出处理
    if (tcsetattr(fd, TCSANOW, &Opt) != 0) // 使设置立即生效
    {
        LOG(ERROR) << "[SERIAL]set param fail";
        return false;
    }
    return true;
}


// 接收数据
bool SerialPort::readData() {
    // 如果rdata未清零，会导致函数返回成功读取的字节数，这些字节数包括了覆盖掉的旧数据和新读取的数据
    memset(rdata, 0, sizeof(rdata));
    int bytes;
    bytes = read(fd, rdata, 30);
    if ((rdata[0] == 0xA5 || rdata[0] == 0xB5 || rdata[0] == 0xC5) && Verify_CRC8_Check_Sum(rdata, 3) && Verify_CRC16_Check_Sum(rdata, 30)) {
        return true;
    }

    while (bytes < 30) {
        close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        openPort();
        if (fd != -1) {
            break;
        }
    }
    return false;
}

// 接受数据
void SerialPort::receiveData(MCUData &data) {
    bool is_receive_data = false;
    // 掉线重连逻辑
    while (!is_receive_data) {
        is_receive_data = readData();
    }
    // 信息提取
    data.mode = rdata[1];
    data.timestamp = Uchar2float__(&rdata[3]);
    data.shoot_speed = Uchar2float__(&rdata[7]);
    Uchar2float(data.quat, &rdata[11], 4);
    data.change_flag = rdata[27];
    return ;
}

// 发送数据
void SerialPort::sendData(const VisionData &data) {
    TransformData(data, Tdata);
    auto write_stauts = write(fd, Tdata, 43);
    if (!write_stauts)
    {
        LOG(ERROR) << "[SERIAL] write msg fail";
    }
    return ;
}

SerialPort::~SerialPort() {
    close(fd);
    LOG(INFO) << "[SERIAL] ~SerialPort";
}