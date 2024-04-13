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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <ostream>
#include <std_msgs/msg/float32.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <thread>   
#include <mutex>

#include "DataType.h"
#include "MappingTables.h"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "my_msg_interface/msg/power_heat.hpp"
#include "my_msg_interface/msg/sentry_cmd.hpp"

using std::hex;

class RefereeSystem : public rclcpp::Node {
    public:

        RefereeSystem(boost::asio::io_service& ioService): 
        Node("RefereeSystem") 
        ,serialPort(ioService)
        {
            GetParam();

            char cwd[1024];
            if (getcwd(cwd, sizeof(cwd)) != NULL) {
                RCLCPP_INFO(this->get_logger(), "Current working dir: %s", cwd);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get current working directory");
            }
            if(en_file_output){
                // 创建一个带有时间戳的文件名
                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                std::tm* now_tm = std::localtime(&now_c);
                char time_str[100];
                std::strftime(time_str, sizeof(time_str), "%Y%m%d%H%M%S", now_tm);
                std::ostringstream filename;
                filename << file_path << "serialPortdump_" << time_str << ".txt";

                file = new std::ofstream(filename.str(), std::ios::binary);
                if (!*file) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open the output file.");
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Output file opened: %s", filename.str().c_str());
                }
            }

            //打开串口列表中的串口
            bool success = false;
            for (const auto& port : serialport_arry) {
                try {
                    RCLCPP_WARN(this->get_logger(), "Opened serial port %s", port.c_str());
                    serialPort.open(port);
                    serialPort.set_option(boost::asio::serial_port::baud_rate(115200));
                    serialPort.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
                    serialPort.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
                    serialPort.set_option(boost::asio::serial_port::character_size(8));

                    success = true;
                    break;
                } catch (boost::system::system_error& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", port.c_str(), e.what());                    
                }
            }
            //开辟线程和分配任务
            if (!success) {
                // throw std::runtime_error("Failed to open any serial port");
                RCLCPP_ERROR(this->get_logger(), "Failed to open any serial port");
            } else {
                io_run = std::thread([&ioService](){
                            while (true) {
                                ioService.run();
                            }
                        });
                //WTF？ Is this CPP ? ? ? 
                //(线程类接受类的重载成员函数)-suzukisuncy
                if(en_file_output) {
                    read_thread = std::thread(
                                        static_cast<void(RM_referee::TypeMethodsTables::*)(boost::asio::serial_port& ,std::ofstream* )>(&RM_referee::TypeMethodsTables::SerialRead),
                                        &Factory_, 
                                        std::ref(serialPort) ,
                                        file
                                    );
                } else {
                    read_thread = std::thread(
                                        static_cast<void(RM_referee::TypeMethodsTables::*)(boost::asio::serial_port& )>(&RM_referee::TypeMethodsTables::SerialRead),
                                        &Factory_, 
                                        std::ref(serialPort) 
                                    );
                }
                RCLCPP_INFO(this->get_logger(), "read_thread has been started.");
                
                process_thread = std::thread(&RM_referee::TypeMethodsTables::ProcessData, &Factory_);
                RCLCPP_INFO(this->get_logger(), "process_thread has been started.");

                sentry_cmd_sub  = this->create_subscription<my_msg_interface::msg::SentryCmd>("sentry_cmd", 1 ,std::bind(&RefereeSystem::DecisionSerialWriteCallback, this , std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "SentryCmdService has been started.");

                plan_sub = this->create_subscription<nav_msgs::msg::Path>("/plan", 1, std::bind(&RefereeSystem::PlanSubCallback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "PlanSub has been started.");
                //以上全部与串口通信相关!
            }

            pBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            pTfListener = std::make_shared<tf2_ros::TransformListener>(*pBuffer,this);

            service = this->create_service<my_msg_interface::srv::RefereeMsg>("RequestSerialize", std::bind(&RefereeSystem::ProcessSerialize, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "RequestSerializeService has been started.");

            // client = this->create_client<nav2_msgs::action::NavigateToPose>("navigate_to_pose");
            //RCLCPP_INFO(this->get_logger(), "NavigateToPoseCancelClient has been started.");

            power_heat_pub = this->create_publisher<my_msg_interface::msg::PowerHeat>("PowerHeat",rclcpp::SystemDefaultsQoS());
            RCLCPP_INFO(this->get_logger(), "PowerHeatPub has been started.");

            timer = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&RefereeSystem::Callback,this));
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
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub;
            rclcpp::Subscription<my_msg_interface::msg::SentryCmd>::SharedPtr sentry_cmd_sub ;
            rclcpp::Service<my_msg_interface::srv::RefereeMsg>::SharedPtr service ;
            // rclcpp::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;
            rclcpp::Publisher<my_msg_interface::msg::PowerHeat>::SharedPtr power_heat_pub ;
            std::unique_ptr<tf2_ros::Buffer> pBuffer;
            std::shared_ptr<tf2_ros::TransformListener> pTfListener;
            rclcpp::TimerBase::SharedPtr timer;
            uint16_t serialize_memcount = 0;

            RM_referee::TypeMethodsTables Factory_;
            std::vector<std::string> serialport_arry;
            boost::asio::serial_port serialPort;
            std::mutex serial_write_mutex;
            std::thread spin_thread;
            std::thread read_thread;
            std::thread process_thread;
            std::thread io_run;

            std::ofstream* file;    
            bool en_file_output = false;
            std::string file_path = "log/";

            bool sentry_id_init_flag = false;
            int DecisionPacketHeaderSequenceNumber = 0;
            int PathPacketHeaderSequenceNumber = 0;
            struct pose_t {
                float x;
                float y;
                float theta;
            } red_path_start_position, blue_path_start_position;

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
                this->declare_parameter<float>("red_path_start_position_x", 0.00);
                this->declare_parameter<float>("red_path_start_position_y", 0.00);
                this->declare_parameter<float>("red_path_start_position_theta", 0.00);
                this->declare_parameter<float>("blue_path_start_position_x", 0.00);
                this->declare_parameter<float>("blue_path_start_position_y", 0.00);
                this->declare_parameter<float>("blue_path_start_position_theta", 0.00);
                this->declare_parameter<int>("sentry_id", 0x0007); /*red_sentry：0x0007 blue_sentry:0x0107 Get Grom Decision*/

                file_path = this->get_parameter("file_output_path").as_string();
                en_file_output = this->get_parameter("file_output").as_bool();

                serialport_arry = this->get_parameter("serialport_arry").as_string_array();
                for (const auto& port : serialport_arry) {
                    RCLCPP_INFO(rclcpp::get_logger("TEST"), "Serialport: %s", port.c_str());
                }
                red_path_start_position.x = this->get_parameter("red_path_start_position_x").as_double();
                red_path_start_position.y = this->get_parameter("red_path_start_position_y").as_double();
                red_path_start_position.theta = this->get_parameter("red_path_start_position_theta").as_double();
                blue_path_start_position.x = this->get_parameter("blue_path_start_position_x").as_double();
                blue_path_start_position.y = this->get_parameter("blue_path_start_position_y").as_double();
                blue_path_start_position.theta = this->get_parameter("blue_path_start_position_theta").as_double();
                tf2_ros::StaticTransformBroadcaster broadcaster(this);
                geometry_msgs::msg::TransformStamped transform;
                tf2::Quaternion q;
                transform.header.stamp = this->now();
                transform.header.frame_id = "map";
                transform.transform.translation.z = 0;
                q.setRPY(0 , 0 , red_path_start_position.theta);
                transform.transform.translation.x = red_path_start_position.x;
                transform.transform.translation.y = red_path_start_position.y;
                transform.transform.rotation = tf2::toMsg(q);
                transform.child_frame_id = "red_frame";
                broadcaster.sendTransform(transform);

                q.setRPY(0 , 0 , blue_path_start_position.theta);
                transform.transform.translation.x = blue_path_start_position.x;
                transform.transform.translation.y = blue_path_start_position.y;
                transform.transform.rotation = tf2::toMsg(q);
                transform.child_frame_id = "blue_frame";
                broadcaster.sendTransform(transform);
            }
            void Callback() {
                auto data_202 = Factory_.Mapserialize(0x202);
                
                RM_referee::PowerHeatDataStruct struct_202;
                std::memcpy(&struct_202,data_202.data(),data_202.size());
                
                //TODO：查询频率和发送频率不匹配丢包？
                auto data_207 = Factory_.Mapserialize(0x207);
                RM_referee::ShootEventSruct struct_207;
                std::memcpy(&struct_207,data_207.data(),data_207.size());

                auto data_201 = Factory_.Mapserialize(0x201);
                RM_referee::RobotStateStruct struct_201;
                std::memcpy(&struct_201,data_201.data(),data_201.size());

                my_msg_interface::msg::PowerHeat send_data;
                send_data.buffer_energy = struct_202.buffer_energy;
                send_data.chassis_power = struct_202.chassis_power;
                send_data.initial_speed = struct_207.initial_speed;
                send_data.launching_frequency = struct_207.launching_frequency;
                send_data.shooter_17mm_1_barrel_heat = struct_202.shooter_17mm_1_barrel_heat;
                send_data.shooter_17mm_2_barrel_heat = struct_202.shooter_17mm_2_barrel_heat;
                power_heat_pub->publish(send_data);

                if(!sentry_id_init_flag ) {
                    if(struct_201.robot_id == 7 || struct_201.robot_id ==107) {
                        if(this->get_parameter("sentry_id").as_int() ==  struct_201.robot_id ) {
                            RCLCPP_INFO(this->get_logger(),"Got sentry_id = %d",struct_201.robot_id );
                            sentry_id_init_flag = true ;
                        } else {
                            this->set_parameter(rclcpp::Parameter("sentry_id", struct_201.robot_id));
                            RCLCPP_INFO(this->get_logger(),"Got sentry_id = %d",struct_201.robot_id );
                            sentry_id_init_flag = true ;
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(),"sentry_id got aborted : %d ! current id is %d",struct_201.robot_id, this->get_parameter("sentry_id").as_int());
                    }
                }

                bool can_cancel_flag = 1 ;
                if(can_cancel_flag && struct_201.current_HP < 300) {    //最低血量->参数服务器
                    //发送cancel
                    
                    can_cancel_flag = 0;
                } else if( !can_cancel_flag && struct_201.current_HP < 300) {
                    can_cancel_flag = 1;
                }
            }

            /**
             * @brief   该数据包用于发送己方机器人的路径规划信息
             *          该数据依赖下列信息
             *              1.地图原点
             *              2.地图xy方向
             *              3.红蓝方
             * @brief 解决办法有两个：1.在地图的yaml中直接设置地图原点和xy方向和裁判系统端地图一致，可能对定位有影响：需要修改地图原点
             *                  2.设置参数服务器，通过ros2的参数服务器来获取地图原点和xy的旋转角度
             *                  3.添加额外坐标系，给定裁判系统地图坐标系通过tf变换获取 recommend
            */
            void PlanSubCallback(const nav_msgs::msg::Path::SharedPtr msg) {
                if( !pBuffer->canTransform("map", "red_frame", tf2::TimePointZero, tf2::durationFromSec(0.1)) ||
                !pBuffer->canTransform("map", "blue_frame", tf2::TimePointZero, tf2::durationFromSec(0.1)) ) 
                    return;
                #pragma pack(push, 1) 
                struct map_data_t {
                    RM_referee::PacketHeader header;
                    uint16_t cmd_id;
                    struct {
                        uint8_t intention; 
                        uint16_t start_position_x; 
                        uint16_t start_position_y; 
                        int8_t delta_x[49]; 
                        int8_t delta_y[49]; 
                        uint16_t sender_id;
                    }path;
                    uint16_t frame_tail;
                }map_data = {
                    .header = {
                        .SOF = 0xA5,
                        .DataLength = 105,
                        .SequenceNumber = static_cast<uint8_t>(PathPacketHeaderSequenceNumber++),
                        .CRC8 = 0x00,
                    },
                    .cmd_id = 0x0301,
                    .path = {
                        .intention = 0x03, //0x01:到目标点攻击 0x02:到目标点防御 0x03:到目标点
                        .start_position_x = 0,
                        .start_position_y = 0,
                        .delta_x = {0},
                        .delta_y = {0},
                        .sender_id = (uint16_t)(this->get_parameter("sentry_id").as_int()),
                    },
                    .frame_tail = 0x0000
                
                };
                #pragma pack(pop)
                static_assert(sizeof(map_data_t) == 114, "map_data_t size error");
                RCLCPP_INFO(this->get_logger(),"%zu",msg->poses.size());
                // std::vector<geometry_msgs::msg::PoseStamped> original = msg->poses; // 是否需要先拷贝，避免段错误？
                std::vector<geometry_msgs::msg::Pose> path_poses_in_red_frame;
                std::vector<geometry_msgs::msg::Pose> path_poses_in_blue_frame;
                if (msg->poses.size() > 49) {//降采样到49个点
                    float step = float(msg->poses.size()) / 49;
                    int count ;
                    for (float i = 0 ; i < msg->poses.size(); i += step) {
                        pBuffer->transform(msg->poses[int(i)].pose, path_poses_in_red_frame.at(count++), "red_frame", tf2::durationFromSec(0.1));
                    }
                } else {
                    for (float i = 0 ; i < msg->poses.size(); i ++) {
                        int count ;
                        pBuffer->transform(msg->poses[int(i)].pose, path_poses_in_red_frame.at(count++), "red_frame", tf2::durationFromSec(0.1));
                    }
                }
                if(!path_poses_in_red_frame.empty()) {
                    map_data.path.start_position_x = path_poses_in_red_frame.front().position.x;
                    map_data.path.start_position_y = path_poses_in_red_frame.front().position.y;
                }
                for(int i =1 ;i<path_poses_in_red_frame.size();i++) {
                    map_data.path.delta_x[i-1] = path_poses_in_red_frame[i].position.x - path_poses_in_red_frame[i-1].position.x;
                    map_data.path.delta_y[i-1] = path_poses_in_red_frame[i].position.y - path_poses_in_red_frame[i-1].position.y;
                }

                map_data.header.CRC8 = Factory_.crc8.Get_CRC8_Check_Sum((uint8_t*)&map_data.header,sizeof(RM_referee::PacketHeader)-2);
                map_data.frame_tail = Factory_.crc16.Get_CRC16_Check_Sum((uint8_t*)&map_data,sizeof(map_data_t)-2);
                

                std::lock_guard<std::mutex> lock(serial_write_mutex);
                serialPort.async_write_some(boost::asio::buffer(&map_data, sizeof(map_data_t)), [](const boost::system::error_code& error, std::size_t bytes_transferred) {
                    if (!error) {
                        RCLCPP_INFO(rclcpp::get_logger("sentry_cmd"), "Wrote %zu bytes", bytes_transferred);
                        //输出一些重要信息
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("sentry_cmd"), "Write failed: %s", error.message().c_str());
                    }
                });
            }

            void DecisionSerialWriteCallback(const my_msg_interface::msg::SentryCmd::SharedPtr msg) { 
                #pragma pack(push, 1) 
                struct decision_serial_write_t {
                    RM_referee::PacketHeader header;
                    uint16_t cmd_id;
                    struct {
                        uint16_t data_cmd_id; 
                        uint16_t sender_id; 
                        uint16_t receiver_id;
                        struct sentry_cmd_t {
                            unsigned int confirmRes : 1;            //是否确认复活：Confirm Resurrection
                            unsigned int confirmInstaRes : 1;       //是否确认兑换立即复活：Confirm Instant Resurrection Exchange
                            unsigned int pendingMissileExch : 11;   //即将兑换的发弹量：Pending Missile Exchange Quantity
                            unsigned int remoteMissileReqCount : 4; //远程兑换的发弹的请求次数：Remote Missile Exchange Request Count
                            unsigned int remoteHealthReqCount : 4;  //远程兑换血量的请求次数：Remote Health Exchange Request Count
                            unsigned int reserved : 11;             //保留：Reserved
                        }sentry_cmd;
                    }robot_interaction_data;
                    uint16_t frame_tail;
                }decision_serial_write = {
                    .header = {
                        .SOF = 0xA5,
                        .DataLength = 10,
                        .SequenceNumber = static_cast<uint8_t>(DecisionPacketHeaderSequenceNumber++),
                        .CRC8 = 0x00,
                    },
                    .cmd_id = 0x0301,
                    .robot_interaction_data = {
                        .data_cmd_id = 0x0120,  /*sentry_decision*/
                        .sender_id = (uint16_t)(this->get_parameter("sentry_id").as_int()),    /*red_sentry：007 blue_sentry:107 */
                        .receiver_id = 0x8080,  /*referee_system:0x8080 */
                        .sentry_cmd = {
                            .confirmRes = msg->confirm_res,
                            .confirmInstaRes = msg->confirm_insta_res,
                            .pendingMissileExch = msg->pending_missile_exch,
                            .remoteMissileReqCount = msg->remote_missile_req_count,
                            .remoteHealthReqCount = msg->remote_health_req_count,
                            .reserved = 0x0000,
                        }
                    },
                    .frame_tail = 0x0000
                };
                #pragma pack(pop)
                static_assert(sizeof(decision_serial_write_t) == 19, "decision_serial_write_t size error");
                decision_serial_write.header.CRC8 = Factory_.crc8.Get_CRC8_Check_Sum((uint8_t*)&decision_serial_write.header,sizeof(RM_referee::PacketHeader)-2);
                decision_serial_write.frame_tail = Factory_.crc16.Get_CRC16_Check_Sum((uint8_t*)&decision_serial_write,sizeof(decision_serial_write_t)-2);
                std::lock_guard<std::mutex> lock(serial_write_mutex);
                RCLCPP_INFO(this->get_logger(),"sentry_cmd ! ! ! ");
                // serialPort.async_write_some(boost::asio::buffer(&decision_serial_write, sizeof(decision_serial_write_t)), [](const boost::system::error_code& error, std::size_t bytes_transferred) {
                //     if (!error) {
                //         // RCLCPP_INFO(rclcpp::get_logger("sentry_cmd"), "Wrote %zu bytes", bytes_transferred);
                //         std::cout<< "Wrote "<< bytes_transferred <<"bytes";
                //         //输出一些重要信息
                //     } else {
                //         RCLCPP_ERROR(rclcpp::get_logger("sentry_cmd"), "Write failed: %s", error.message().c_str());
                //     }
                // });
                serialPort.write_some(boost::asio::buffer(&decision_serial_write, sizeof(decision_serial_write_t)));
                RCLCPP_INFO(this->get_logger(),"sentry_cmd 1 1 1  ");
            }

// ros2 topic info /plan
// Type: nav_msgs/msg/Path

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
    //创建一个work对象这将阻止io_service::run在空闲时返回；
    boost::asio::io_service::work work(ioservice_);
    rclcpp::spin(std::make_shared<RefereeSystem>(ioservice_));
    rclcpp::shutdown();
    return 0;
}


