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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <fstream>
#include <ostream>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <thread>   
#include <mutex>
#include <future>

#include "DataType.h"
#include "MappingTables.h"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "my_msg_interface/msg/power_heat.hpp"
#include "my_msg_interface/msg/sentry_cmd.hpp"
#include "my_msg_interface/msg/vision_cmd.hpp"
#include "my_msg_interface/msg/aerial_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

using std::hex;
#define IGNORED  1
#define DETECTED 0

#pragma pack(push, 1) 
struct BehaviorTopicMsg {
    struct target_t {
        float x;
        float y;
    }target;
    uint8_t GameForceStart:4;
    uint8_t tracking:4;
};
#pragma pack(pop)
static_assert(sizeof(BehaviorTopicMsg) == 9);
class RefereeSystem : public rclcpp::Node {
    public:

        RefereeSystem(boost::asio::io_service& ioService): 
        Node("RefereeSystem") 
        ,serialPort(ioService)
        {
            GetParam();
            bool success = false;

            //不使能回放功能时，判断是否有文件输出，和打开串口功能
            if(!en_en_replay) {
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

                if (!success) {
                    // throw std::runtime_error("Failed to open any serial port");
                    RCLCPP_ERROR(this->get_logger(), "Failed to open any serial port");
                } else{
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

                    //数据处理线程只与数据相关
                    process_thread = std::thread(&RM_referee::TypeMethodsTables::ProcessData, &Factory_);
                    RCLCPP_INFO(this->get_logger(), "process_thread has been started.");
                    
                    //以下全部与串口发送相关!
                    sentry_cmd_sub  = this->create_subscription<my_msg_interface::msg::SentryCmd>("sentry_cmd", 1 ,std::bind(&RefereeSystem::DecisionSerialWriteCallback, this , std::placeholders::_1));
                    RCLCPP_INFO(this->get_logger(), "SentryCmdService has been started.");

                    plan_sub = this->create_subscription<nav_msgs::msg::Path>("/plan", 1, std::bind(&RefereeSystem::PlanSubCallback, this, std::placeholders::_1));
                    RCLCPP_INFO(this->get_logger(), "PlanSub has been started.");

                }
            } else {
                //回放功能
                read_thread = std::thread(
                                    static_cast<void(RM_referee::TypeMethodsTables::*)(std::string )>(&RM_referee::TypeMethodsTables::SerialRead),
                                    &Factory_, 
                                    replay_file_path
                                );
                RCLCPP_INFO(this->get_logger(), "read_thread has been started.");

                //数据处理线程只与数据相关
                process_thread = std::thread(&RM_referee::TypeMethodsTables::ProcessData, &Factory_);
                RCLCPP_INFO(this->get_logger(), "process_thread has been started.");
            }

            // target_sub = std::make_shared<auto_aim_interfaces::msg::Target>("tracker/target",rclcpp::SensorDataQoS(),std::bind());
            control_id_sub = this->create_subscription<std_msgs::msg::Float64>("control_id",rclcpp::SensorDataQoS(),std::bind(&RefereeSystem::ControlIDCallbcak,this, std::placeholders::_1));

            pBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            pTfListener = std::make_shared<tf2_ros::TransformListener>(*pBuffer,this);

            service = this->create_service<my_msg_interface::srv::RefereeMsg>("RequestSerialize", std::bind(&RefereeSystem::ProcessSerialize, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "RequestSerializeService has been started.");

            // client = this->create_client<nav2_msgs::action::NavigateToPose>("navigate_to_pose");
            //RCLCPP_INFO(this->get_logger(), "NavigateToPoseCancelClient has been started.");

            power_heat_pub = this->create_publisher<my_msg_interface::msg::PowerHeat>("PowerHeat",rclcpp::SystemDefaultsQoS());
            RCLCPP_INFO(this->get_logger(), "PowerHeatPub has been started.");

            vision_cmd_pub = this->create_publisher<my_msg_interface::msg::VisionCmd>("VisionCmd",rclcpp::SystemDefaultsQoS());
            RCLCPP_INFO(this->get_logger(), "VisionCmdPub has been started.");

            aerial_cmd_pub = this->create_publisher<my_msg_interface::msg::AerialCmd>("AerialCmd",rclcpp::SystemDefaultsQoS());
            RCLCPP_INFO(this->get_logger(), "AerialCmdPub has been started.");
            
            std::memset(&last_struct_003,0,sizeof(last_struct_003));
            std::memset(&last_struct_303,0,sizeof(last_struct_303));
            ignore_cmd.set__base(IGNORED)
                .set__guard(IGNORED)
                .set__outpost(DETECTED)
                .set__hero(DETECTED)
                .set__miner(DETECTED)
                .set__infantry_3(DETECTED)
                .set__infantry_4(DETECTED)
                .set__infantry_5(DETECTED);
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
            rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub ;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_id_sub ;
            
            rclcpp::Service<my_msg_interface::srv::RefereeMsg>::SharedPtr service ;
            // rclcpp::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;
            rclcpp::Publisher<my_msg_interface::msg::PowerHeat>::SharedPtr power_heat_pub ;
            rclcpp::Publisher<my_msg_interface::msg::VisionCmd>::SharedPtr vision_cmd_pub ;
            rclcpp::Publisher<my_msg_interface::msg::AerialCmd>::SharedPtr aerial_cmd_pub ;
            std::unique_ptr<tf2_ros::Buffer> pBuffer;
            std::shared_ptr<tf2_ros::TransformListener> pTfListener;
            rclcpp::TimerBase::SharedPtr timer;
            uint16_t serialize_memcount = 0;
            RM_referee::GameRobotHPStruct last_struct_003;
            RM_referee::MinimapInteractionCommsMessageStruct last_struct_303;
            BehaviorTopicMsg behaviortopicmsg;

            my_msg_interface::msg::VisionCmd ignore_cmd;

            RM_referee::TypeMethodsTables Factory_;
            std::vector<std::string> serialport_arry;
            boost::asio::serial_port serialPort;
            std::mutex serial_write_mutex;
            std::mutex ignore_mutex;
            std::mutex behaviortopicmsg_mutex;

            std::thread spin_thread;
            std::thread read_thread;
            std::thread process_thread;
            std::thread io_run;

            std::ofstream* file;    
            bool en_file_output = false;
            bool en_en_replay = false;
            std::string file_path = "log/";
            std::string replay_file_path = "log/";

            bool sentry_id_init_flag = false;
            int DecisionPacketHeaderSequenceNumber = 0;
            int PathPacketHeaderSequenceNumber = 0;
            struct pose_t {
                float x;
                float y;
                float theta;
            } red_path_start_position, blue_path_start_position;

            void ProcessSerialize(const my_msg_interface::srv::RefereeMsg::Request::SharedPtr request,const my_msg_interface::srv::RefereeMsg::Response::SharedPtr response) {
                RCLCPP_INFO(this->get_logger(), "Got Request!");
                if(request->cmd_id == 0xAFF ) {
                    response->cmd_id = 0xAFF;
                    response->data_length = sizeof(BehaviorTopicMsg);
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&behaviortopicmsg);
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + sizeof(BehaviorTopicMsg);
                    response->data_stream.resize(sizeof(BehaviorTopicMsg));
                    response->data_stream.erase(response->data_stream.begin(),response->data_stream.end()); 
                    response->data_stream.insert(response->data_stream.end(), dataStart, dataEnd);
                    return;
                } else {
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
                en_en_replay = this->get_parameter("en_replay").as_bool();
                replay_file_path = this->get_parameter("replay_file_path").as_string();
                
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
                transform.child_frame_id = "aerial_map";
                broadcaster.sendTransform(transform);

            }
            void Callback() {
                auto data_003 = Factory_.Mapserialize(0x003);
                RM_referee::GameRobotHPStruct struct_003;
                std::memcpy(&struct_003,data_003.data(),data_003.size());

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

                auto data_303 = Factory_.Mapserialize(0x303);
                RM_referee::MinimapInteractionCommsMessageStruct struct_303;
                std::memcpy(&struct_303,data_303.data(),data_303.size());

                my_msg_interface::msg::PowerHeat send_data;
                send_data.buffer_energy = struct_202.buffer_energy;
                send_data.chassis_power = struct_202.chassis_power;
                send_data.initial_speed = struct_207.initial_speed;
                send_data.launching_frequency = struct_207.launching_frequency;
                send_data.shooter_17mm_1_barrel_heat = struct_202.shooter_17mm_1_barrel_heat;
                send_data.shooter_17mm_2_barrel_heat = struct_202.shooter_17mm_2_barrel_heat;
                power_heat_pub->publish(send_data);

                //避免冗余，获取数据后，回调函数即可返回，开启新线程处理数据
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

                auto pFunc = [&](uint16_t id){
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    std::lock_guard<std::mutex> lock(ignore_mutex);
                    switch (id) {
                        case 1:
                        case 101:
                            ignore_cmd.hero = DETECTED;
                            break;
                        case 2:
                        case 102:
                            ignore_cmd.miner = DETECTED;
                            break;
                        case 3:
                        case 103:
                            ignore_cmd.infantry_3 = DETECTED;
                            break;
                        case 4:
                        case 104:
                            ignore_cmd.infantry_4 = DETECTED;
                            break;
                        case 5:
                        case 105:
                            ignore_cmd.infantry_5 = DETECTED;
                            break;
                        case 7:
                        case 107:
                            ignore_cmd.guard = DETECTED;
                            break;
                        default:
                            break;
                    }
                    ignore_mutex.unlock();
                };
                //如果血量没有任何变化直接跳过
                if(std::memcmp(&last_struct_003,&struct_003,32)){
                    RCLCPP_INFO(this->get_logger(),"\n %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d \n %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d %d <- %d ",
                                struct_003.blue_1_robot_HP,last_struct_003.blue_1_robot_HP,
                                struct_003.blue_2_robot_HP,last_struct_003.blue_2_robot_HP,
                                struct_003.blue_3_robot_HP,last_struct_003.blue_3_robot_HP,
                                struct_003.blue_4_robot_HP,last_struct_003.blue_4_robot_HP,
                                struct_003.blue_5_robot_HP,last_struct_003.blue_5_robot_HP,
                                struct_003.blue_7_robot_HP,last_struct_003.blue_7_robot_HP,
                                struct_003.blue_base_HP,last_struct_003.blue_base_HP,
                                struct_003.blue_outpost_HP,last_struct_003.blue_outpost_HP,
                                struct_003.red_1_robot_HP,last_struct_003.red_1_robot_HP,
                                struct_003.red_2_robot_HP,last_struct_003.red_2_robot_HP,
                                struct_003.red_3_robot_HP,last_struct_003.red_3_robot_HP,
                                struct_003.red_4_robot_HP,last_struct_003.red_4_robot_HP,
                                struct_003.red_5_robot_HP,last_struct_003.red_5_robot_HP,
                                struct_003.red_7_robot_HP,last_struct_003.red_7_robot_HP,
                                struct_003.red_base_HP,last_struct_003.red_base_HP,
                                struct_003.red_outpost_HP,last_struct_003.red_outpost_HP
                                );
                    if(struct_201.robot_id == 107){
                        //机器人血量从0变化到非0 即可认为复活，添加时间戳和机器人id到，处理队列中（使用异步，等待10秒后修改bool）
                        if(last_struct_003.red_1_robot_HP==0 && struct_003.red_1_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.hero = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 1).detach();                        }
                        if(last_struct_003.red_2_robot_HP==0 && struct_003.red_2_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.miner = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 2).detach();
                        }
                        if(last_struct_003.red_3_robot_HP==0 && struct_003.red_3_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.infantry_3 = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 3).detach();
                        }
                        if(last_struct_003.red_4_robot_HP==0 && struct_003.red_4_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.infantry_4 = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 4).detach();
                        }
                        if(last_struct_003.red_5_robot_HP==0 && struct_003.red_5_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.infantry_5 = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 5).detach();
                        }
                        if(struct_003.red_outpost_HP != 0){
                            ignore_cmd.guard = IGNORED;
                        } else if(last_struct_003.red_7_robot_HP == 0 && struct_003.red_7_robot_HP != 0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.guard = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 7).detach();
                        }
                    }else if(struct_201.robot_id == 7) {
                        if(last_struct_003.blue_1_robot_HP==0 && struct_003.blue_1_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.hero = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 101).detach();
                        }
                        if(last_struct_003.blue_2_robot_HP==0 && struct_003.blue_2_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.miner = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 102).detach();
                        }
                        if(last_struct_003.blue_3_robot_HP==0 && struct_003.blue_3_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.infantry_3 = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 103).detach();
                        }
                        if(last_struct_003.blue_4_robot_HP==0 && struct_003.blue_4_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.infantry_4 = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 104).detach();
                        }
                        if(last_struct_003.blue_5_robot_HP==0 && struct_003.blue_5_robot_HP!=0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.infantry_5 = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 105).detach();
                        }
                        if(struct_003.blue_outpost_HP != 0){
                            ignore_cmd.guard = IGNORED;
                        } else if(last_struct_003.blue_7_robot_HP == 0 && struct_003.blue_7_robot_HP != 0){
                            std::lock_guard<std::mutex> lock(ignore_mutex);
                            ignore_cmd.guard = IGNORED;
                            ignore_mutex.unlock();
                            std::thread(pFunc, 107).detach();
                        }
                    } 
                }
                ignore_cmd.outpost = IGNORED;
                vision_cmd_pub->publish(ignore_cmd);
                //update data
                std::memcpy(&last_struct_003,&struct_003,sizeof(struct_003));

                my_msg_interface::msg::AerialCmd aerial_cmd;
                // if(std::memcmp(&last_struct_303,&struct_303,sizeof(RM_referee::MinimapInteractionCommsMessageStruct))){
                    if( !pBuffer->canTransform("aerial_map", "map", tf2::TimePointZero, tf2::durationFromSec(0.1)) )
                        return;
                    std::string fromFrameRel = "aerial_map";
                    std::string toFrameRel = "map"; 
                    geometry_msgs::msg::PoseStamped target_pos;
                    target_pos.header.set__frame_id("map").set__stamp(this->get_clock()->now());
                    target_pos.pose.position.set__x(struct_303.target_position_x).set__y(struct_303.target_position_y).set__z(0);
                    target_pos.pose.orientation.set__x(0).set__y(0).set__z(0).set__w(1);
                    geometry_msgs::msg::PoseStamped temp;
                    geometry_msgs::msg::TransformStamped t;
                    try{
                        t = pBuffer->lookupTransform(
                            toFrameRel, fromFrameRel,
                            tf2::TimePointZero);
                        tf2::doTransform(target_pos, temp, t);
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_INFO(
                            this->get_logger(), "Could not transform %s to %s: %s",
                            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                        return;
                    }
                    aerial_cmd.set__updated(true)
                                .set__target_pose(temp);
                // } else {
                    // aerial_cmd.set__updated(false)
                    //             .set__target_pose(geometry_msgs::msg::PoseStamped());
                // }
                // aerial_cmd_pub->publish(aerial_cmd);
                std::memcpy(&last_struct_303,&struct_303,sizeof(struct_303));
                
                if(struct_303.cmd_source != 0)
                    RCLCPP_INFO(this->get_logger(),"%d %#X %c %f %f ",
                                                    struct_303.target_robot_id,
                                                    struct_303.cmd_source,
                                                    struct_303.cmd_keyboard,
                                                    struct_303.target_position_x,
                                                    struct_303.target_position_y
                                                    );
                bool can_cancel_flag = 1 ;
                if(can_cancel_flag && struct_201.current_HP < 300) {    //最低血量->参数服务器
                    //发送cancel
                    
                    can_cancel_flag = 0;
                } else if( !can_cancel_flag && struct_201.current_HP < 300) {
                    can_cancel_flag = 1;
                }
            }
            void TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg) {
                std::lock_guard<std::mutex> behaviortopicmsg_lock(behaviortopicmsg_mutex); 
            }
            void ControlIDCallbcak(const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> behaviortopicmsg_lock(behaviortopicmsg_mutex); 
                if(msg->data == 1) {
                    behaviortopicmsg.GameForceStart = 1;
                }else {
                    behaviortopicmsg.GameForceStart = 0;
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
                if( !pBuffer->canTransform("map", "aerial_map", tf2::TimePointZero, tf2::durationFromSec(0.1)) )
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
                    .cmd_id = 0x0307,
                    .path = {
                        .intention = 0x01, //0x01:到目标点攻击 0x02:到目标点防御 0x03:到目标点
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
                geometry_msgs::msg::PoseStamped temp;
                std::vector<geometry_msgs::msg::Pose> path_poses_in_aerial_map;
                std::string fromFrameRel = "map";
                std::string toFrameRel = "aerial_map"; 
                RCLCPP_INFO(rclcpp::get_logger("FRAME"),"%s",toFrameRel.c_str());
                geometry_msgs::msg::TransformStamped t;
                if (msg->poses.size() > 49) {//降采样到49个点
                    float step = float(msg->poses.size()) / 49;
                    path_poses_in_aerial_map.erase(path_poses_in_aerial_map.begin(),path_poses_in_aerial_map.end());
                    path_poses_in_aerial_map.resize(0);
                    for (float i = 0 ; i < msg->poses.size()-1; i += step) {
                        // pBuffer->transform(msg->poses[int(i)].pose, temp, "aerial_map");
                        //     path_poses_in_aerial_map.push_back(temp.pose);
                        // RCLCPP_INFO(rclcpp::get_logger("ORIGIN"),"origin:(%lf,%lf)",msg->poses[int(i)].pose.position.x,msg->poses[int(i)].pose.position.y);
                        try{
                            t = pBuffer->lookupTransform(
                                toFrameRel, fromFrameRel,
                                tf2::TimePointZero);
                            tf2::doTransform(msg->poses[int(i)], temp, t);
                            path_poses_in_aerial_map.push_back(temp.pose);
                            // RCLCPP_INFO(rclcpp::get_logger("TEST1"),"temp.poses:(%lf,%lf)size:%zu",temp.pose.position.x,temp.pose.position.y,path_poses_in_aerial_map.size());
                        } catch (const tf2::TransformException & ex) {
                            RCLCPP_INFO(
                                this->get_logger(), "Could not transform %s to %s: %s",
                                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                            return;
                        }
                    }
                } else {
                    // pBuffer->transform(msg->poses[int(i)].pose, temp, "aerial_map");
                    // path_poses_in_aerial_map.push_back(temp);
                    path_poses_in_aerial_map.erase(path_poses_in_aerial_map.begin(),path_poses_in_aerial_map.end());
                    path_poses_in_aerial_map.resize(0);
                    for (float i = 0 ; i < msg->poses.size(); i ++) {
                        try{
                            t = pBuffer->lookupTransform(
                                toFrameRel, fromFrameRel,
                                tf2::TimePointZero);
                            tf2::doTransform(msg->poses[int(i)], temp, t);
                            path_poses_in_aerial_map.push_back(temp.pose);
                            // RCLCPP_INFO(rclcpp::get_logger("TEST1"),"temp.poses:(%lf,%lf)size:%zu",temp.pose.position.x,temp.pose.position.y,path_poses_in_aerial_map.size());
                        } catch (const tf2::TransformException & ex) {
                            RCLCPP_INFO(
                                this->get_logger(), "Could not transform %s to %s: %s",
                                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                            return;
                        }
                    }
                }
                RCLCPP_INFO(rclcpp::get_logger("TEST"), "start:(%lf, %lf)",path_poses_in_aerial_map.front().position.x,path_poses_in_aerial_map.front().position.y);
                if(!path_poses_in_aerial_map.empty()) {
                    map_data.path.start_position_x = path_poses_in_aerial_map.begin()->position.x * 10;
                    map_data.path.start_position_y = path_poses_in_aerial_map.begin()->position.y * 10;
                }
                int accumulated_x = path_poses_in_aerial_map.begin()->position.x * 10;//dm
                int accumulated_y = path_poses_in_aerial_map.begin()->position.y * 10;//dm
                for(size_t i =1 ;i<path_poses_in_aerial_map.size();i++) {
                    map_data.path.delta_x[i-1] = path_poses_in_aerial_map[i].position.x * 10 - accumulated_x;
                    accumulated_x += map_data.path.delta_x[i-1];
                    map_data.path.delta_y[i-1] = path_poses_in_aerial_map[i].position.y * 10 - accumulated_y;
                    accumulated_y += map_data.path.delta_y[i-1];
                }
                    // map_data.path.start_position_x = 55;
                    // map_data.path.start_position_y = 75;
                    // map_data.path.delta_x[0] = 0;
                    // map_data.path.delta_y[0] = 20;
                // for(int i = 0 ; i < 49 ; i++) {
                //     RCLCPP_INFO(this->get_logger(),"delta %d %d ",map_data.path.delta_x[i],map_data.path.delta_y[i]);
                // };
                map_data.header.CRC8 = Factory_.crc8.Get_CRC8_Check_Sum((uint8_t*)&map_data.header,sizeof(RM_referee::PacketHeader)-1);
                map_data.frame_tail = Factory_.crc16.Get_CRC16_Check_Sum((uint8_t*)&map_data,sizeof(map_data_t)-2);
                
                if( Factory_.crc8.Verify_CRC8_Check_Sum((uint8_t*)&map_data,sizeof(RM_referee::PacketHeader))
                    && Factory_.crc16.Verify_CRC16_Check_Sum((uint8_t*)&map_data,114)){
                    RCLCPP_INFO(this->get_logger(),"Right！");
                } else{
                    RCLCPP_INFO(this->get_logger(),"False！");
                }
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
                        // .SequenceNumber = 0x02,
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
                decision_serial_write.header.CRC8 = Factory_.crc8.Get_CRC8_Check_Sum((uint8_t*)&decision_serial_write.header,sizeof(RM_referee::PacketHeader)-1);
                decision_serial_write.frame_tail = Factory_.crc16.Get_CRC16_Check_Sum((uint8_t*)&decision_serial_write,sizeof(decision_serial_write_t)-2);
                if( Factory_.crc8.Verify_CRC8_Check_Sum((uint8_t*)&decision_serial_write,sizeof(RM_referee::PacketHeader))
                    && Factory_.crc16.Verify_CRC16_Check_Sum((uint8_t*)&decision_serial_write,19)){
                    // RCLCPP_INFO(this->get_logger(),"Right！");
                } else{
                    // RCLCPP_INFO(this->get_logger(),"False！");
                }
                std::lock_guard<std::mutex> lock(serial_write_mutex);
                RCLCPP_INFO(this->get_logger(),"sentry_cmd %d %d %d %d %d",msg->confirm_res,
                                                                        msg->confirm_insta_res,
                                                                        msg->pending_missile_exch,
                                                                        msg->remote_missile_req_count,
                                                                        msg->remote_health_req_count
                                                                        );
                // serialPort.async_write_some(boost::asio::buffer(&decision_serial_write, sizeof(decision_serial_write_t)), [](const boost::system::error_code& error, std::size_t bytes_transferred) {
                //     if (!error) {
                //         RCLCPP_INFO(rclcpp::get_logger("sentry_cmd"), "Wrote %zu bytes", bytes_transferred);
                //         //输出一些重要信息
                //     } else {
                //         RCLCPP_ERROR(rclcpp::get_logger("sentry_cmd"), "Write failed: %s", error.message().c_str());
                //     }
                // });
                serialPort.write_some(boost::asio::buffer(&decision_serial_write, sizeof(decision_serial_write_t)));
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


