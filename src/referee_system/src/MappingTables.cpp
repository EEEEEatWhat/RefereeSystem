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
    #include <iomanip>
//upside is for test 
namespace RM_referee{
    TypeMethodsTables::TypeMethodsTables() {
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
                    printf("Mapserialize success ! cmd_id id : 0x%x DataLength:%d\n",cmd_id,data.size()); \
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
        // std::lock_guard<std::mutex> lock(m_map_mutex);
        // auto it = m_map.find(cmd_id);
        // if(it!=m_map.end()) {
            std::memcpy(Pdest,Pdata,PdataSize);
        // } else {
        //     printf("\ncurrent cmd_id does not exist! error id : 0x%x\n",cmd_id);
        //     // throw std::out_of_range("cmd_id not found in map");
        //     return 0;
        // }
    }

    /** 
     * TODO:
     * 数据处理逻辑
     * 0.第一字节默认0xA5
     * 1.接受长度小于头长度则等待数据再处理
     * 3.CRC校验
     *  清除处理过数据包，开始数据端处理
     * 0.检查buffer剩余数据长度
     * 1.正常则处理，缺少则等待数据再处理
     * 2.CRC16校验
     * 3.调用Factory.Solve()
    */
    void TypeMethodsTables::SerialReadAsync(boost::asio::serial_port& serialPort,std::vector<uint8_t>& buffer) {
        boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(sizeof(RM_referee::PacketHeader)),
            [&, header = RM_referee::PacketHeader()](const boost::system::error_code& ec, std::size_t bytes_transferred) mutable {
                if (!ec) {
                    
                }
            }
            
        );

    };




    int TypeMethodsTables::read() {
        system("pwd");
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