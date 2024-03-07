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

class RefereeSystem : public rclcpp::Node {
    public:
        RM_referee::TypeMethodsTables Factory_;
        std::string serialport;
        int buffersize;

            RefereeSystem(): Node("RefereeSystem") {
                GetParam();
                service = this->create_service<my_msg_interface::srv::RefereeMsg>("RequestSerialize", std::bind(&RefereeSystem::ProcessSerialize,this,std::placeholders::_1,std::placeholders::_2));
                RCLCPP_INFO(this->get_logger(), "RefereeSystem has been started.");
            }

            void GetParam() {
                this->declare_parameter<std::string>("serialport","/dev/ttyUSB0");
                this->declare_parameter<int>("buffersize", 4096);
                this->get_parameter_or("serialport", serialport);
                this->get_parameter_or("buffersize", buffersize);
                RCLCPP_WARN(this->get_logger(), "Serialport: %s", serialport.c_str());
                RCLCPP_WARN(this->get_logger(), "buffersize: %d", buffersize);
            }
        private:
            rclcpp::Service<my_msg_interface::srv::RefereeMsg>::SharedPtr service ;
            uint16_t serialize_memcount = 0;
        void ProcessSerialize(const my_msg_interface::srv::RefereeMsg::Request::SharedPtr request,const my_msg_interface::srv::RefereeMsg::Response::SharedPtr response) {
            serialize_memcount = Factory_.MapSearchDataLength(request->cmd_id);
            RCLCPP_INFO(this->get_logger(), "request->cmd_id:0x%x",request->cmd_id);
            if(serialize_memcount == 0) { //cmd_id不存在
                RCLCPP_INFO(this->get_logger(), "cmd_id不存在");
                response->cmd_id = 0x0000;
                response->data_length = 0;
                response->data_stream.resize(0);
                return;
            }
            response->cmd_id = request->cmd_id;
            response->data_stream.resize(serialize_memcount);
            auto data = Factory_.Mapserialize(request->cmd_id);
            response->set__data_stream(data);
            response->data_length = serialize_memcount;
        }   
};

    //TODO:导出子类插件给行为树做解包
    //TODO:尝试使用模板函数重写
    /*
    服务端消息类型
    请求  uint16 cmd_id
    ---
    返回  uint16 cmd_id
         uint16 data_length
         uint8[] data_stream 
    当返回值为0x0000时，表示cmd_id不存在
    */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_service ioService;
    auto Prefereesystem = std::make_shared<RefereeSystem>();
    boost::asio::serial_port serialPort(ioService, Prefereesystem->serialport);
    std::vector<uint8_t> buffer(Prefereesystem->buffersize);  // 适当调整缓冲区大小

    Prefereesystem->Factory_.SerialReadAsync(serialPort ,buffer);
    // rclcpp::spin();
    rclcpp::shutdown();
    return 0;
}


