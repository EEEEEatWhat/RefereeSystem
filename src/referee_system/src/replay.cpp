/**
 * @author suzukisuncy
 * @date 2024/7/3
 * @brief 将dump下来的串口数据回放，以便赛后DEBUG
*/

#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <ostream>
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
            //读取重放文件的路径
            GetParam();

            //开辟线程和分配任务
            //WTF？ Is this CPP ? ? ? 
            //(线程类接受类的重载成员函数)-suzukisuncy
            read_thread = std::thread(
                                static_cast<void(RM_referee::TypeMethodsTables::*)(std::string )>(&RM_referee::TypeMethodsTables::SerialRead),
                                &Factory_, 
                                replay_file_path
                            );
            RCLCPP_INFO(this->get_logger(), "read_thread has been started.");
            process_thread = std::thread(&RM_referee::TypeMethodsTables::ProcessData, &Factory_);
            RCLCPP_INFO(this->get_logger(), "process_thread has been started.");

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
            serialPort.close();
            if (file) {
                file->close();
                delete file;
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
            std::thread replay_thread;
            std::ofstream* file;    
            bool en_file_output = false;
            std::string file_path = "log/";
            std::string replay_file_path = "file/";
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
            this->declare_parameter<bool>("file_output", false);
            this->declare_parameter<std::string>("file_output_path", "log");
            this->declare_parameter<std::string>("replay_file_path", "file");
            file_path = this->get_parameter("file_output_path").as_string();
            replay_file_path = this->get_parameter("replay_file_path").as_string();
            en_file_output = this->get_parameter("file_output").as_bool();

            serialport_arry = this->get_parameter("serialport_arry").as_string_array();
            for (const auto& port : serialport_arry) {
                RCLCPP_INFO(rclcpp::get_logger("TEST"), "Serialport: %s", port.c_str());
            }
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_service ioservice_;  
    rclcpp::spin(std::make_shared<RefereeSystem>(ioservice_));
    rclcpp::shutdown();
    return 0;
}


