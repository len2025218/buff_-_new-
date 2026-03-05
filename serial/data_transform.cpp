#include "./data_transform.h"
// 将4个uchar转换为float
float Uchar2float__(unsigned char *data)
{
    float float_data;
    float_data = *((float*)data);
    return float_data;
}

// 将uchar转换为为n个float
bool Uchar2float(float *target, unsigned char *data, int n) {
    for (int i = 0; i < n; i++) {
        unsigned char *f = &data[i * 4];
        target[i] = Uchar2float__(f);
    }
    return true;
}

// float转uchar
unsigned char *float2Uchar(float float_data)
{
    unsigned char *raw_data = nullptr;
    raw_data = (unsigned char *)(&float_data);
    return std::move(raw_data);
}

// float转uchar数组
bool float2UcharArray(float float_data[], int num, unsigned char *char_data)
{
    for (int i = 0; i < num; i++)
    {
        unsigned char *data = float2Uchar(float_data[i]);
        for (int j = 0; j < 4; j++)
        {
            char_data[i * 4 + j] = data[j];
        }
    }
    return true;
}

// 将视觉原始数据转换为uchar
void TransformData(const VisionData &data, unsigned char *Tdata)
{

    Tdata[0] = 0xA5;
    Tdata[1] = data.isShooting;

	Append_CRC8_Check_Sum(Tdata, 3);

    // yaw_offest、pitch_offest
    float float_data[] = {data.yaw_angle, data.pitch_angle};
    // std::cout<<data.yaw_angle<<std::endl;
    float2UcharArray(float_data, 2, &Tdata[3]);
    Tdata[11] = data.is_detect;
    Tdata[12] = data.hand;
    float float_data_m[] = {data.image_point_x, data.image_point_y, data.delay, data.distance, data.x, data.y, data.z};
    float2UcharArray(float_data_m, 7, &Tdata[13]);
    Append_CRC16_Check_Sum(Tdata, 43);
}