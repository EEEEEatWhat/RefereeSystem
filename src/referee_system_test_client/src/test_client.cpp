#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "DataType.h"
class RefereeSystem_Test_Client : public rclcpp::Node {
    public:
        RefereeSystem_Test_Client(): Node("RefereeSystemTestClient") {
            client = this->create_client<my_msg_interface::srv::RefereeMsg>("RequestSerialize");
            RCLCPP_INFO(this->get_logger(), "RefereeSystem_Test_Client has been started.");
        }
    
        bool connect_server() {
            while (!client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强制退出！");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(),"服务连接中，请稍候...");
            }
            return true;
        }

        rclcpp::Client<my_msg_interface::srv::RefereeMsg>::FutureAndRequestId send_request(uint16_t cmd_id){
            auto request = std::make_shared<my_msg_interface::srv::RefereeMsg::Request>();
            request->cmd_id = cmd_id;
            return client->async_send_request(request);
        }
    private:
        rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedPtr client;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<RefereeSystem_Test_Client>();
    bool flag = client->connect_server();
    if (!flag) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接失败！");
        return 0;
    }
    int i = 1 ; 
    while (i++) {
        uint16_t cmd_id = 0x102;
        cmd_id += (i%3)*0x100;
        cmd_id  =0x202;
        auto response = client->send_request(cmd_id);
        RCLCPP_INFO(client->get_logger(),"发送请求:0x%x",cmd_id);
        // 处理响应
        if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(client->get_logger(),"请求正常处理");
            auto result = response.get();
            if(result->data_stream.size() != 0) {
                RCLCPP_INFO(client->get_logger(),"响应正常11111");
                RCLCPP_INFO(client->get_logger(),"cmd_id:0x%x,data_length:%d",result->cmd_id,result->data_stream.size());
                if(result->cmd_id == 0x202) {
                    RM_referee::PowerHeatDataStruct T;
                    std::memcpy(&T,result->data_stream.data(),result->data_length);
                    RCLCPP_INFO(client->get_logger(),"chassis电压(mV):%d",T.chassis_voltage);
                    RCLCPP_INFO(client->get_logger(),"chassis电流(mA):%d",T.chassis_current);
                    RCLCPP_INFO(client->get_logger(),"底盘功率(W):%d",T.chassis_power);
                    RCLCPP_INFO(client->get_logger(),"缓冲能量(J):%d",T.buffer_energy);
                    RCLCPP_INFO(client->get_logger(),"第1个17mm枪口热量:%d",T.shooter_17mm_1_barrel_heat);
                    RCLCPP_INFO(client->get_logger(),"第2个17mm枪口热量:%d",T.shooter_17mm_2_barrel_heat);
                    RCLCPP_INFO(client->get_logger(),"42mm枪口热量:%d",T.shooter_42mm_barrel_heat);
                }
                
            } 
        } else {
            RCLCPP_INFO(client->get_logger(),"请求异常");
        }
        sleep(2);
    }
    rclcpp::shutdown();
    return 0;
}
