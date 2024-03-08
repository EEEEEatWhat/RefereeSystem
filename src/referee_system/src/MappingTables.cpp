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
            if(powerheatdatapacket.GetID() == cmd_id) {
                if(powerheatdatapacket.m_mutex.try_lock()) {
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&powerheatdatapacket.m_value);
                    const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + powerheatdatapacket.GetDataLength();
                    data.resize(powerheatdatapacket.GetDataLength());
                    data.insert(data.end(), dataStart, dataEnd);
                    powerheatdatapacket.m_mutex.unlock();
                    printf("Mapserialize success ! cmd_id id : 0x%x DataLength:%d\n",cmd_id,data.size());
                    return data;
                } else {
                    data.resize(0);
                    return data;
                }
            }
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
     * WAITING: 遇到0xA5开辟代处理队列存取字节到代处理队列进入 PROCESSINGHEAD
     * PROCESSINGHEAD: 连续存取字节到代处理队列,代处理队列长度等于头长度则进行crc8,成功则进入 PROCESSINGPACKET 失败清空代处理队列进入 WAITING
     * PROCESSINGPACKET: 连续存取字节到代处理队列,代处理队列长度等于头长度+2字节(cmd_id)+头包中解析的数据+2字节(crc16)长度则进行crc16,成功则唤醒线程进行数据包处理,并进入 WAITING,失败清空代处理队列进入 WAITING
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
        uint8_t current_byte;
        std::vector<uint8_t>* processingArry;
        uint16_t current_data_length;
        Cmd_ID cmd_id;
        while (!exitFlag_)
        {
            std::unique_lock<std::mutex> lock(dataQueue_mutex);
            condVar_.wait(lock, [this]() { return !dataQueue_.empty(); });
            current_byte = *dataQueue_.front().data();
            dataQueue_.pop();
            RCLCPP_WARN(rclcpp::get_logger("TEST"), "%X ",current_byte);
            lock.unlock();
            if(state == WAITING) {
                if(current_byte == RM_referee::StartOfFrame) {
                    state = PROCESSINGHEAD;
                    processingArry = new std::vector<uint8_t>();
                }
            }
            if(state == PROCESSINGHEAD) {
                if(processingArry->size() < sizeof(RM_referee::PacketHeader)) {
                    processingArry->push_back((uint8_t)current_byte);
                } else {
                    if(crc8.Verify_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&processingArry),sizeof(RM_referee::PacketHeader))) {
                        state = PROCESSINGPACKET;
                        current_data_length = ((RM_referee::PacketHeader*)processingArry)->DataLength;
                    } else {
                        state = WAITING;
                        delete processingArry;
                        processingArry = nullptr;
                    }
                }
            }
            if(state == PROCESSINGPACKET) {
                if(processingArry->size() < sizeof(RM_referee::PacketHeader) + 2 + current_data_length + 2) {
                    processingArry->push_back((uint8_t)current_byte);
                    if(processingArry->size() == processingArry->size() + 1) {
                        cmd_id.data[0] = (uint8_t)current_byte;
                    }
                    if(processingArry->size() == processingArry->size() + 2) {
                        cmd_id.data[1] = (uint8_t)current_byte;
                        RCLCPP_WARN(rclcpp::get_logger("TEST"), "cmd_id:0x%x",cmd_id.cmd_id);
                    }
                } else {
                    if(crc16.Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&processingArry),sizeof(processingArry))) {
                        //TODO:比较这样操作和额外开辟线程的区别
                        uint16_t erased = MapSolve(cmd_id.cmd_id,processingArry[sizeof(RM_referee::PacketHeader)+2].data(),current_data_length);
                        if(erased != cmd_id.cmd_id) {
                            RCLCPP_WARN(rclcpp::get_logger("TEST"), "erased:0x%x",erased);
                            throw std::runtime_error("There must be a bug in the code!");
                        }
                        state = WAITING;
                        delete processingArry;
                        cmd_id.cmd_id = 0;
                        current_data_length = 0;
                        processingArry = nullptr;
                    } else {
                        state = WAITING;
                        delete processingArry;
                        cmd_id.cmd_id = 0;
                        current_data_length = 0;
                        processingArry = nullptr;
                    }
                }
            }
        }
    }

    void TypeMethodsTables::SerialRead(boost::asio::serial_port& serialPort) {
        while (!exitFlag_) {
            boost::system::error_code ec;    
            std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
            std::size_t len = serialPort.read_some(boost::asio::buffer(buffer), ec);
            if (!ec) {
                std::lock_guard<std::mutex> lock(dataQueue_mutex);
                dataQueue_.push(std::vector<uint8_t>(buffer.begin(), buffer.begin() + len));
                condVar_.notify_one();
            }
            else {
                // handle error
            }
        }

    };




    int TypeMethodsTables::read() {
        // std::ifstream file("../../../../samples.txt");
        std::ifstream file("/home/suzuki/RefereeSystem/src/referee_system/samples2.txt");

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
        return 0;
    };

    void TypeMethodsTables::testprocess() {
            // std::cout<<testbuffer.size()<<"\n";
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
                    uint16_t erased = MapSolve(cmd_id,&(*(it+2)),testheader.DataLength);
                    if(erased) it += 2+erased+2;
                }
            } else
                it ++;
    };

}