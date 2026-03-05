电控->视觉（串口数据）
针头  1 byte
mode(模式)  1 byte
CRC8校验和  1 byte
时间戳timestamp  4 byte(float)
射速shoot_speed  4 byte(float)
四元数quat[4]  16 byte(float)
是否统一时间戳的标志位time_flag  1 byte
// 0x00(应该是，电控自己确定以前CRC16校验和前面发的是不是0x00) 1 byte
CRC16校验和  2 byte

视觉->电控（串口数据）
针头  1 byte
是否开火  1 byte
CRC8校验和  1 byte
yaw补偿  4 byte(float) // 注意顺序别和pitch弄反了
pitch补偿  4 byte(float)
is_detect  1 byte(int) // 视觉是否有检测到合适的目标
is_spinning 1 byte(int) // 0否，1是
CRC16校验和  2 byte

测试：
这里读写操作并发进行，没有进行线程安全的处理
原因是：大多数操作系统的串口驱动会维护一个输入缓冲区和输出缓冲区，用于暂存待读取和待发送的数据。这些缓冲区的使用可以使得读写操作在一定程度上独立于实际物理串口的操作，从而降低了直接的竞争条件可能性。
为保证大多数情况下的稳定性：开学用Log和电控记录数据，收发同样的数据，看看有没有出错的情况，如果没有，那么就不用加锁（毕竟加锁会导致运行效率减慢），如果有，为保证线程安全，还是要加锁滴。

