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

        RefereeSystem(boost::asio::io_service& ioService): 
        Node("RefereeSystem") 
        ,serialPort(ioService)
        {
            GetParam();
            bool success = false;
            for (const auto& port : serialport_arry) {
                RCLCPP_WARN(this->get_logger(), "Opened serial port %s", port.c_str());
                try {
                    serialPort.open(port);
                    success = true;
                    break;
                } catch (boost::system::system_error& e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s: %s", port.c_str(), e.what());                    
                }
            }

            if (!success) {
                // throw std::runtime_error("Failed to open any serial port");
                RCLCPP_ERROR(this->get_logger(), "Failed to open any serial port");
            } else {
                read_thread = std::thread(&RM_referee::TypeMethodsTables::SerialRead, &Factory_, std::ref(serialPort));
                RCLCPP_INFO(this->get_logger(), "read_thread has been started.");
                process_thread = std::thread(&RM_referee::TypeMethodsTables::ProcessData, &Factory_);
                RCLCPP_INFO(this->get_logger(), "process_thread has been started.");
            }

            service = this->create_service<my_msg_interface::srv::RefereeMsg>("RequestSerialize", std::bind(&RefereeSystem::ProcessSerialize,this,std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "RefereeSystem has been started.");
        }

        ~RefereeSystem() {
            Factory_.exitFlag_ = true;        
            if (read_thread.joinable()) {
                read_thread.join();
            }
            if (process_thread.joinable()) {
                process_thread.join();
            }
            RCLCPP_INFO(this->get_logger(), "RefereeSystem has been stopped.");
        }

        private:    
            rclcpp::Service<my_msg_interface::srv::RefereeMsg>::SharedPtr service ;
            uint16_t serialize_memcount = 0;
            RM_referee::TypeMethodsTables Factory_;
            std::vector<std::string> serialport_arry;
            boost::asio::serial_port serialPort;
            std::thread spin_thread;
            std::thread read_thread;
            std::thread process_thread;

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

        void spin() {
            rclcpp::spin(shared_from_this());
        }

        void GetParam() {
            this->declare_parameter<int>("buffersize", 4096);
            this->declare_parameter<std::vector<std::string>>("serialport_arry",{"ttyUSB0"});

            serialport_arry = this->get_parameter("serialport_arry").as_string_array();
            for (int i=0; i<serialport_arry.size(); i++) {
                RCLCPP_WARN(rclcpp::get_logger("TEST"), "Serialport: %s", serialport_arry[i].c_str());
            }
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
    boost::asio::io_service ioservice_;  
    rclcpp::spin(std::make_shared<RefereeSystem>(ioservice_));
    rclcpp::shutdown();
    return 0;
}


