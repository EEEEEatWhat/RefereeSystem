/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include "DataType.h"
#include "MappingTables.h"
using std::hex;
int main()
{
    /*
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");
    std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
    */
    RM_referee::TypeMethodsTables Factory_;
    Factory_.AddTypeMethod<RM_referee::GameStatusPacket>(0x0001);
    Factory_.AddTypeMethod<RM_referee::ExtSupplyProjectileAction>(0x0102);
    Factory_.AddTypeMethod<RM_referee::PowerHeatDataPacket>(0x0202);
    Factory_.AddTypeMethod<RM_referee::CustomRobotDataPacket>(0x0302);
    while(1) {
        Factory_.testprocess();
    };
    return 0;
}

