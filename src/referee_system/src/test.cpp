/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>   

#include "DataType.h"
#include "MappingTables.h"
#include "my_msg_interface/srv/referee_msg.hpp"
 
using std::hex;
char ascii_art[] = R"(
  _________    ____  _____ __________ 
 /_  __/   |  / __ \/ ___// ____/ __ \
  / / / /| | / /_/ /\__ \/ / __/ / / /
 / / / ___ |/ _, _/___/ / /_/ / /_/ / 
/_/ /_/  |_/_/ |_|/____/\____/\____/  
        Author: SuzukiSuncy
    )";

class RefereeSystem : public rclcpp::Node {
    public:
        RefereeSystem(): Node("RefereeSystem") {
            service = this->create_service<my_msg_interface::srv::RefereeMsg>("RequesSerialize", std::bind(&RefereeSystem::ProcessSerialize,this,std::placeholders::_1,std::placeholders::_2));
            std::thread threadOne([&](){
                while(1) {
                    Factory_.testprocess();
                    sleep(1);
                };

            });
            threadOne.detach();
        }

    private:
        rclcpp::Service<my_msg_interface::srv::RefereeMsg>::SharedPtr service ;
        RM_referee::TypeMethodsTables Factory_;

        void ProcessSerialize(const my_msg_interface::srv::RefereeMsg::Request::SharedPtr request,const my_msg_interface::srv::RefereeMsg::Response::SharedPtr response) {
            Factory_.Mapserialize(reinterpret_cast<uint8_t*>(&response->data_stream), request->cmd_id);
            response->cmd_id = request->cmd_id;
            response->data_length = sizeof(response->data_stream);
        }
};

    //TODO:导出子类插件给行为树做解包
    //TODO:尝试使用模板函数重写
    //服务端消息类型
    //请求  uint16 cmd_id
    //---
    //返回  uint16 cmd_id
    //     uint16 data_length
    //     uint8[] data_stream 
    /*
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");
    std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
    */
int main(int argc, char * argv[])
{
    printf(ascii_art);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefereeSystem>());
    rclcpp::shutdown();
    return 0;
}


