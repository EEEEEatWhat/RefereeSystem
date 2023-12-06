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
char ascii_art[] = R"(
   _____                   __   _ _____                       
  / ___/__  ______  __  __/ /__(_) ___/__  ______  _______  __
  \__ \/ / / /_  / / / / / //_/ /\__ \/ / / / __ \/ ___/ / / /
 ___/ / /_/ / / /_/ /_/ / ,< / /___/ / /_/ / / / / /__/ /_/ / 
/____/\__,_/ /___/\__,_/_/|_/_//____/\__,_/_/ /_/\___/\__, /  
                                                     /____/   
                author: suzukisuncy
    )";
using std::hex;
int main()
{
    //TODO 添加CRC校验
    //TODO:宏提高复用性
    //TODO:类内成员保护
    //TODO:导出子类插件给行为树做解包
    //TODO:服务端消息类型
    //请求uint16_t cmd_id
    //---
    //返回  uint16_t cmd_id
    //     uint16_t data_length
    //     uint8_t[] data_stream 
    /*

    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");
    std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
    */
    printf(ascii_art);
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


