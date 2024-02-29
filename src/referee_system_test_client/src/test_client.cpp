#include "rclcpp/rclcpp.hpp"
#include "my_msg_interface/srv/referee_msg.hpp"

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
        auto response = client->send_request(cmd_id);
        RCLCPP_INFO(client->get_logger(),"发送请求:0x%x",cmd_id);
        // 处理响应
        if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(client->get_logger(),"请求正常处理");
            if(response.get()->data_length != 0) {
                RCLCPP_INFO(client->get_logger(),"响应正常");
            } 

        } else {
            RCLCPP_INFO(client->get_logger(),"请求异常");
        }
        sleep(1);
    }
    rclcpp::shutdown();
    return 0;
}
