#include <iostream>

#include "./CRC_Check.h"
#include "../general.h"

float Uchar2float__(unsigned char *data);
bool Uchar2float(float *target, unsigned char *data, int n);
unsigned char *float2Uchar(float float_data);
void TransformData(const VisionData &data, unsigned char *Tdata);