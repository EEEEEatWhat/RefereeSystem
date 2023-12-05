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
    //TODO 提高复用性,类内成员保护
    /*
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");
    std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
    */
    RM_referee::TypeMethodsTables Factory_;
    // Factory_.AddTypeMethod<RM_referee::GameStatusPacket>(RM_referee::GameStatusPacket::GetID());
    Factory_.AddTypeMethod<RM_referee::ExtSupplyProjectileAction>(RM_referee::ExtSupplyProjectileAction::GetID());
    Factory_.AddTypeMethod<RM_referee::PowerHeatDataPacket>(RM_referee::PowerHeatDataPacket::GetID());
    Factory_.AddTypeMethod<RM_referee::CustomRobotDataPacket>(RM_referee::CustomRobotDataPacket::GetID());
    Factory_.read();
    while(1) {
        Factory_.testprocess();
        // sleep(1);
    };
    return 0;
}


