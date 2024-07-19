/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#include <memory>
#include <functional>
#include <mutex>
#include <boost/asio.hpp>
#include "MappingTables.h"
//below is for test 
    #include<cstdlib>
    #include <iostream>
    #include <fstream>
    #include <sstream>
    #include <vector>
    #include <rclcpp/rclcpp.hpp>
    #include <iomanip>
//upside is for test 
namespace RM_referee{
    TypeMethodsTables::TypeMethodsTables() :exitFlag_(false){
        m_map.clear();
        //TODO：初始化数据包
        m_map.emplace(gamestatuspacket.GetID(),&gamestatuspacket);
        m_map.emplace(gameresulteventpacket.GetID(),&gameresulteventpacket);
        m_map.emplace(gamerobothppacket.GetID(),&gamerobothppacket);
        m_map.emplace(playgroundeventpacket.GetID(), &playgroundeventpacket);  
        m_map.emplace(extsupplyprojectileactionpacket.GetID(), &extsupplyprojectileactionpacket);
        m_map.emplace(dartinfopacket.GetID(),&dartinfopacket);
        m_map.emplace(robotpositionpacket.GetID(),&robotpositionpacket);
        m_map.emplace(robotstatepacket.GetID(),&robotstatepacket);
        m_map.emplace(refereewarningeventpacket.GetID(),&refereewarningeventpacket);
        m_map.emplace(powerheatdatapacket.GetID(), &powerheatdatapacket);
        m_map.emplace(robotbuffpacket.GetID(),&robotbuffpacket);
        m_map.emplace(airsupportdatapacket.GetID(),&airsupportdatapacket);
        m_map.emplace(damageeventpacket.GetID(),&damageeventpacket);
        m_map.emplace(shooteventpacket.GetID(),&shooteventpacket);
        m_map.emplace(projectileallowancepacket.GetID(),&projectileallowancepacket);
        m_map.emplace(robotrfidstatepacket.GetID(),&robotrfidstatepacket);
        m_map.emplace(dartclientcmdpacket.GetID(),&dartclientcmdpacket);
        m_map.emplace(groundrobotpositionpacket.GetID(),&groundrobotpositionpacket);
        m_map.emplace(radarmarkdatapacket.GetID(),&radarmarkdatapacket);
        m_map.emplace(sentryinfopacket.GetID(),&sentryinfopacket);
        m_map.emplace(customrobotdatapacket.GetID(), &customrobotdatapacket);  
        m_map.emplace(minimapinteractioncommsmessagepacket.GetID(),&minimapinteractioncommsmessagepacket);
        m_map.emplace(keyboardmousemessagepacket.GetID(),&keyboardmousemessagepacket);
        m_map.emplace(clientminimaprecvpacket.GetID(),&clientminimaprecvpacket);
        m_map.emplace(customclientdatapacket.GetID(),&customclientdatapacket);
        m_map.emplace(mapdatapacket.GetID(),&mapdatapacket);
        m_map.emplace(custominfopacket.GetID(),&custominfopacket);
    }

    TypeMethodsTables::~TypeMethodsTables() {}

    uint16_t TypeMethodsTables::MapSolve(const uint16_t cmd_id , uint8_t* data ,uint16_t data_size){
        std::lock_guard<std::mutex> lock(m_map_mutex);
        auto it = m_map.find(cmd_id);
        if(it!=m_map.end()) {
            return it->second->SolvePacket(cmd_id ,data ,data_size);
        } else {
            printf("\ncurrent cmd_id does not exist! error id : 0x%x\n",cmd_id);
            // throw std::out_of_range("cmd_id not found in map");
            return 0;
        }
    }

    #define SERIALIZEPACKET(PACKET) \
            if(PACKET.GetID() == cmd_id) { \
                if(PACKET.m_mutex.try_lock()) { \
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&PACKET.m_value); \
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + PACKET.GetDataLength(); \
                    data.resize(PACKET.GetDataLength()); \
                    data.erase(data.begin(),data.end()); \
                    data.insert(data.end(), dataStart, dataEnd); \
                    PACKET.m_mutex.unlock(); \
                    return data; \
                } else { \
                    data.resize(0); \
                    return data; \
                } \
            } \

    std::vector<boost::asio::detail::buffered_stream_storage::byte_type>& TypeMethodsTables::Mapserialize(const uint16_t cmd_id ) {
        try { // 可能抛出异常的代码
            /**
            SERIALIZEPACKET(powerheatdatapacket);
            * * * * * * * * * * * * * * * * * * * 
            if(powerheatdatapacket.GetID() == cmd_id) { \
                if(powerheatdatapacket.m_mutex.try_lock()) { \
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&powerheatdatapacket.m_value); \
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + powerheatdatapacket.GetDataLength(); \
                    data.resize(powerheatdatapacket.GetDataLength()); \
                    data.erase(data.begin(),data.end()); \
                    data.insert(data.end(), dataStart, dataEnd); \
                    powerheatdatapacket.m_mutex.unlock(); \
                    return data; \
                } else { \
                    data.resize(0); \
                    return data; \
                } \
            } \
            */
            SERIALIZEPACKET(gamestatuspacket);
            SERIALIZEPACKET(gameresulteventpacket);
            SERIALIZEPACKET(gamerobothppacket);
            SERIALIZEPACKET(playgroundeventpacket);
            SERIALIZEPACKET(extsupplyprojectileactionpacket);
            SERIALIZEPACKET(dartinfopacket);
            SERIALIZEPACKET(robotpositionpacket);
            SERIALIZEPACKET(robotstatepacket);
            SERIALIZEPACKET(refereewarningeventpacket);
            SERIALIZEPACKET(powerheatdatapacket);
            SERIALIZEPACKET(robotbuffpacket);
            SERIALIZEPACKET(airsupportdatapacket);
            SERIALIZEPACKET(damageeventpacket);
            SERIALIZEPACKET(shooteventpacket);
            SERIALIZEPACKET(projectileallowancepacket);
            SERIALIZEPACKET(robotrfidstatepacket);
            SERIALIZEPACKET(dartclientcmdpacket);
            SERIALIZEPACKET(groundrobotpositionpacket);
            SERIALIZEPACKET(radarmarkdatapacket);
            SERIALIZEPACKET(sentryinfopacket);
            SERIALIZEPACKET(customrobotdatapacket);
            SERIALIZEPACKET(minimapinteractioncommsmessagepacket);
            SERIALIZEPACKET(keyboardmousemessagepacket);
            SERIALIZEPACKET(clientminimaprecvpacket);
            SERIALIZEPACKET(customclientdatapacket);
            SERIALIZEPACKET(mapdatapacket);
            SERIALIZEPACKET(custominfopacket);

        } catch (const std::system_error& e) {
            std::cerr << "Caught system_error with code " << e.code()
                    << " meaning " << e.what() << '\n';
        }
        printf("current cmd_id does not exist! error id : 0x%x\n",cmd_id);
        data.resize(0);
        return data;
    }

    uint16_t TypeMethodsTables::MapSearchDataLength(const uint16_t cmd_id ) {
        std::lock_guard<std::mutex> lock(m_map_mutex);
        auto it = m_map.find(cmd_id);
        if(it!=m_map.end()) {
            return it->second->GetDataLength();
        } else {
            printf("current cmd_id does not exist! error id : 0x%x\n",cmd_id);
            return 0;
        }
    }

    uint16_t TypeMethodsTables::FilledPacketData(void* Pdest ,const uint16_t PdestSize ,void* Pdata , const uint16_t PdataSize , const uint16_t cmd_id ) { 
        (void)PdestSize;
        // std::lock_guard<std::mutex> lock(m_map_mutex);
        // auto it = m_map.find(cmd_id);
        // if(it!=m_map.end()) {
            std::memcpy(Pdest,Pdata,PdataSize);
        // } else {
        //     printf("\ncurrent cmd_id does not exist! error id : 0x%x\n",cmd_id);
        //     // throw std::out_of_range("cmd_id not found in map");
        //     return 0;
        // }
        return cmd_id;
    }

    /** 
     * 使用状态机
     * ALL:遍历每一个字节,
     * WAITING: 遇到0xA5开辟待处理队列存取字节到待处理队列进入 PROCESSINGHEAD
     * PROCESSINGHEAD: 连续存取字节到待处理队列,待处理队列长度等于头长度则进行crc8,成功则进入 PROCESSINGPACKET 失败清空待处理队列进入 WAITING
     * PROCESSINGPACKET: 连续存取字节到待处理队列,待处理队列长度等于头长度+2字节(cmd_id)+头包中解析的数据+2字节(crc16)长度则进行crc16,成功则唤醒线程进行数据包处理,并进入 WAITING,失败清空待处理队列进入 WAITING\
     * BUG: 有时候会出现crc8错误，但是却略过了正确的数据包的部分，crc8错误不代表crc校验的内容全部都是错误的
    */

    /** 
     * 使用状态机
     * ALL:通过偏移指针（此时偏移数等于零）遍历字节，无效则弹出第一个字节
     * WAITING: 遇到0xA5，开辟待处理队列存取字节到待处理队列，偏移字节数开始增加，进入 PROCESSINGHEAD
     * PROCESSINGHEAD:  连续存取字节到待处理队列,待处理队列长度等于头长度（偏移字节数等于头长度）则进行crc8,
     *                  成功则进入 PROCESSINGPACKET 
     *                  失败偏移字节归零弹出第一个字节，清空待处理队列进入 WAITING
     *                  （即避免错误的0xA5后面紧跟着下一个正确的0xA5）
     * PROCESSINGPACKET:连续存取字节到待处理队列,待处理队列长度等于头长度+2字节(cmd_id)+头包中解析的数据+2字节(crc16)长度（偏移字节数相同）则进行crc16,
     *                  成功则唤醒线程进行数据包处理,并进入 WAITING,
     *                  失败偏移字节归零弹出第一个字节，清空待处理队列进入 WAITING
     * TODO: crc16错误可否认为这只是数据包传输错误，不是部分错误字节导致的，认为整个数据包内容都是错误的，不会包含正确的数据包片段？
    */
    void TypeMethodsTables::ProcessData()
    {
        enum ProcessState { 
            WAITING,
            PROCESSINGHEAD,
            PROCESSINGPACKET
        };
        union Cmd_ID {
            uint16_t cmd_id;
            uint8_t data[2];
        };

        
        ProcessState state = WAITING;
        uint8_t current_byte ;
        uint16_t offset_bytes = 0 , start_bytes = 0;
        std::vector<uint8_t>* processingArry;
        uint16_t current_data_length = 0;
        Cmd_ID cmd_id;
        while (!exitFlag_)
        {
            std::unique_lock<std::mutex> lock(dataQueue_mutex);
            condVar_.wait(lock, [this,&offset_bytes]() { return /*!dataQueue_.empty() &&*/ dataQueue_.size() > static_cast<std::vector<uint8_t>::size_type>(offset_bytes)+1; });//BUG!
            current_byte = *(dataQueue_.begin() + offset_bytes);
            lock.unlock();
            // RCLCPP_INFO(rclcpp::get_logger("process"),"%#x,%d",current_byte,offset_bytes);
            if(state == WAITING) {
                if(current_byte == RM_referee::StartOfFrame) { 
                    //检测到帧头切换状态,记录帧头字节位置
                    start_bytes = offset_bytes ;
                    state = PROCESSINGHEAD;
                    processingArry = new std::vector<uint8_t>();
                } else { 
                    offset_bytes++;
                }
            }
            if(state == PROCESSINGHEAD) {
                if(processingArry->size() < sizeof(RM_referee::PacketHeader)) {
                    //尚未存满头部，偏移自增继续存取
                    processingArry->push_back(current_byte);
                    offset_bytes++;
                } else {
                    if(crc8.Verify_CRC8_Check_Sum(processingArry->data(),processingArry->size())) {
                        //头部校验成功，进入数据包处理状态，获取数据长度
                        state = PROCESSINGPACKET;
                        current_data_length = ((RM_referee::PacketHeader*)processingArry->data())->DataLength;
                        // RCLCPP_WARN(rclcpp::get_logger("Process"), "DataLength : %d",current_data_length);
                    } else {
                        //头部校验失败，清空待处理队列，偏移字节数归零，移除第一个字节
                        RCLCPP_WARN(rclcpp::get_logger("Process"), "CRC8 check error!");
                        state = WAITING;
                        delete processingArry;
                        processingArry = nullptr;
                        offset_bytes = 0;
                        std::lock_guard<std::mutex> lock(dataQueue_mutex);
                        dataQueue_.erase(dataQueue_.begin(),dataQueue_.begin()+start_bytes+1);
                    }
                }
            }
            if(state == PROCESSINGPACKET) {
                if(processingArry->size() < sizeof(RM_referee::PacketHeader) + 2 + current_data_length + 2) {
                    //尚未存满数据包，继续存取，并尝试获取cmd_id
                    processingArry->push_back(current_byte);
                    offset_bytes++;
                    if(processingArry->size() == sizeof(RM_referee::PacketHeader) + 1) {
                        cmd_id.data[0] = current_byte;
                    }
                    if(processingArry->size() == sizeof(RM_referee::PacketHeader) + 2) {
                        cmd_id.data[1] = current_byte;
                        // RCLCPP_WARN(rclcpp::get_logger("TEST"), "cmd_id:%#x",cmd_id.cmd_id);
                    }
                } else {
                    if(crc16.Verify_CRC16_Check_Sum(processingArry->data(),processingArry->size())) {
                        //TODO:比较这样操作和额外开辟线程的区别
                        //数据包校验成功，唤醒线程进行数据包处理,同时清空待处理队列，偏移字节数归零，移除已经处理的字节
                        uint16_t processed = MapSolve(cmd_id.cmd_id,&processingArry->at(sizeof(RM_referee::PacketHeader)+2),current_data_length);
                        if(processed != current_data_length) {
                            RCLCPP_WARN(rclcpp::get_logger("TEST"), "cmd_id:%#x current_length:%d processed_length:%d",cmd_id.cmd_id,current_data_length,processed);
                            // throw std::runtime_error("There must be a bug in the code!");
                        }
                        state = WAITING;
                        delete processingArry;
                        cmd_id.cmd_id = 0;
                        current_data_length = 0;
                        processingArry = nullptr;
                        std::lock_guard<std::mutex> lock(dataQueue_mutex);
                        dataQueue_.erase(dataQueue_.begin(), dataQueue_.begin() + offset_bytes);
                        offset_bytes = 0;
                    } else {
                        //数据包校验失败，清空待处理队列，偏移字节数归零，移除第一个字节
                        RCLCPP_WARN(rclcpp::get_logger("Process"), "CRC16 check error!");
                        state = WAITING;
                        delete processingArry;
                        cmd_id.cmd_id = 0;
                        current_data_length = 0;
                        processingArry = nullptr;
                        offset_bytes = 0;
                        std::lock_guard<std::mutex> lock(dataQueue_mutex);
                        dataQueue_.erase(dataQueue_.begin(),dataQueue_.begin()+start_bytes+1);
                    }
                }
            }
        }
    }
    /**
     * @note 做必要的日志文件记录和输出 
    */
    void TypeMethodsTables::SerialRead(boost::asio::serial_port& serialPort, std::ofstream* file) {
        boost::system::error_code ec;    
        std::vector<uint8_t> buffer(1);  // 适当调整缓冲区大小
        while (!exitFlag_) {
            std::size_t len = serialPort.read_some(boost::asio::buffer(buffer), ec);        
            // RCLCPP_INFO(rclcpp::get_logger("receive"),"%#x",buffer.front());
            if (!ec) {
                std::lock_guard<std::mutex> lock(dataQueue_mutex);
                dataQueue_.insert(dataQueue_.end(), buffer.begin(), buffer.begin() + len);
                file->write(reinterpret_cast<const char*>(buffer.data()), len);  // 将接收到的字节写入文件
                condVar_.notify_one();
            }
            else {
                // handle error
            }
        }
    };

    void TypeMethodsTables::SerialRead(boost::asio::serial_port& serialPort) {
        boost::system::error_code ec;    
        std::vector<uint8_t> buffer(1);  // 适当调整缓冲区大小
        while (!exitFlag_) {
            std::size_t len = serialPort.read_some(boost::asio::buffer(buffer), ec);        
            // RCLCPP_INFO(rclcpp::get_logger("receive"),"%#x",buffer.front());
            if (!ec) {
                std::lock_guard<std::mutex> lock(dataQueue_mutex);
                dataQueue_.insert(dataQueue_.end(), buffer.begin(), buffer.begin() + len);
                condVar_.notify_one();
            }
            else {
                // handle error
            }
        }
    };

    /**
     * @brief for replay 
    */
    void TypeMethodsTables::SerialRead(std::string replay_file_path) {
            int BAUD_RATE = 14400; // 115200 bits per second = 14400 bytes per second
            int SLEEP_DURATION = 1000; // in milliseconds
            std::ifstream file(replay_file_path, std::ios::binary);
            std::vector<char> buffer(BAUD_RATE);

            if(!file.is_open()) {
                RCLCPP_ERROR(rclcpp::get_logger("replay"), "Unable to open the file.");
                return ;
            }

            while (!file.eof()) {
                file.read(buffer.data(), BAUD_RATE);

                {
                    std::lock_guard<std::mutex> lock(dataQueue_mutex);
                    dataQueue_.insert(dataQueue_.end(), buffer.begin(), buffer.end());
                }

                condVar_.notify_one();
                std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_DURATION));
            } 
            if(file.eof()){
                RCLCPP_ERROR(rclcpp::get_logger("replay"), "file is over.");
            }
    };

    void TypeMethodsTables::AsyncSerialRead(boost::asio::serial_port& serialPort) {
        std::vector<uint8_t> buffer(16);  
        while (!exitFlag_) {
            serialPort.async_read_some(boost::asio::buffer(buffer), [this,&buffer](boost::system::error_code ec, std::size_t len) {
                if (!ec) {
                    std::lock_guard<std::mutex> lock(dataQueue_mutex);
                    dataQueue_.insert(dataQueue_.end(), buffer.begin(), buffer.begin() + len);
                    condVar_.notify_one();
                }
                else {
                    // handle error
                }
            });
        }
    }

    void TypeMethodsTables::AsyncSerialRead(boost::asio::serial_port& serialPort ,std::ofstream* file) {
        std::vector<uint8_t> buffer(16);  
        while (!exitFlag_) {
            serialPort.async_read_some(boost::asio::buffer(buffer), [this,&buffer,&file](boost::system::error_code ec, std::size_t len) {
                if (!ec) {
                    std::lock_guard<std::mutex> lock(dataQueue_mutex);
                    dataQueue_.insert(dataQueue_.end(), buffer.begin(), buffer.begin() + len);
                    file->write(reinterpret_cast<const char*>(buffer.data()), len);  // 将接收到的字节写入文件
                    condVar_.notify_one();
                }
                else {
                    // handle error
                }
            });
        }
    }

    int TypeMethodsTables::read() {
        // std::ifstream file("../../../../samples.txt");
        std::ifstream file("/home/eatwhat/ws00_sentry/src/RefereeSystem/dump/serialPortdump_20240514211200.txt");

        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                while (!iss.eof()) {
                    int byte;
                    iss >> std::hex >> byte;
                    testbuffer.push_back(static_cast<uint8_t>(byte));
                }
            }
            file.close();
        } else {
            std::cerr << "Unable to open the file." << std::endl;
        }

        testheader = RM_referee::PacketHeader();
        it = testbuffer.begin();//重复执行
        RCLCPP_INFO(rclcpp::get_logger("logger"),"read once ");

        return 0;
    };

    void TypeMethodsTables::testprocess() {
            std::cout<<testbuffer.size()<<"\n";
            if(testbuffer.size() == 0) {
                read();
                return;
            };
            if(testbuffer.size() <= sizeof(RM_referee::PacketHeader))
                return ;
            while (*it != RM_referee::StartOfFrame) {
                it++;
                if (it == testbuffer.end()) {
                    read();
                    return;
                }
            }
            testbuffer.erase(testbuffer.begin(), it);
            it = testbuffer.begin();
            std::memcpy(&testheader, &(*it), sizeof(RM_referee::PacketHeader));
            //CRC8
            if(crc8.Verify_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&testheader),sizeof(RM_referee::PacketHeader))) {
                it += sizeof(RM_referee::PacketHeader);
                uint16_t cmd_id = static_cast<uint16_t>( ((*it)<<8) | (*(it+1)) );
                //CRC16
                if(crc16.Verify_CRC16_Check_Sum(&(*testbuffer.begin()),sizeof(RM_referee::PacketHeader) + 2 + testheader.DataLength + 2)) {
                    RCLCPP_INFO(rclcpp::get_logger("logger"),"%#x",cmd_id);
                    uint16_t erased = MapSolve(cmd_id,&(*(it+2)),testheader.DataLength);
                    if(erased) it += 2+erased+2;
                }
            } else
                it ++;
    };

}