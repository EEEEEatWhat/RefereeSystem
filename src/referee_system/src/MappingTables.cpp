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
        // m_map.emplace(GameStatusPacket::GetID(), std::make_shared<GameStatusPacket>(&gamestatuspacket));
        m_map.emplace(extsupplyprojectileactionpacket.GetID(), &extsupplyprojectileactionpacket);
        m_map.emplace(powerheatdatapacket.GetID(), &powerheatdatapacket);
        m_map.emplace(customrobotdatapacket.GetID(), &customrobotdatapacket);  
    }

    TypeMethodsTables::~TypeMethodsTables() {}

    uint16_t TypeMethodsTables::MapSolve(const uint16_t cmd_id , uint8_t* data ,uint16_t data_size){
        auto it = m_map.find(cmd_id);
        if(it!=m_map.end()) {
            return it->second->SolvePacket(cmd_id ,data ,data_size);
        } else {
            printf("\ncurrent cmd_id does not exist! error id : 0x%x\n",cmd_id);
            // throw std::out_of_range("cmd_id not found in map");
            return 0;
        }
    }

    uint16_t TypeMethodsTables::Mapserialize(std::vector<boost::asio::detail::buffered_stream_storage::byte_type> &Pdata ,const uint16_t cmd_id ) {
        if(powerheatdatapacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(powerheatdatapacket.m_mutex);
            RM_referee::PowerHeatDataStruct& frontPowerHeatDataElement = powerheatdatapacket.m_queue.front();
            const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&frontPowerHeatDataElement);
            const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + powerheatdatapacket.GetDataLength();
            Pdata.insert(Pdata.end(), dataStart, dataEnd);
            powerheatdatapacket.m_queue.pop();
            powerheatdatapacket.m_mutex.unlock();
            printf("Mapserialize success ! cmd_id id : 0x%x DataLength:%d\n",cmd_id,powerheatdatapacket.GetDataLength());
            return powerheatdatapacket.GetDataLength();
        }

        if(customrobotdatapacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(customrobotdatapacket.m_mutex);        
            RM_referee::CustomRobotDataStruct& frontCustomRobotDataElement = customrobotdatapacket.m_queue.front();
            const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&frontCustomRobotDataElement);
            const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + customrobotdatapacket.GetDataLength();
            Pdata.insert(Pdata.end(), dataStart, dataEnd);
            customrobotdatapacket.m_queue.pop();
            customrobotdatapacket.m_mutex.unlock();
            printf("Mapserialize success ! cmd_id id : 0x%x DataLength:%d\n",cmd_id,customrobotdatapacket.GetDataLength());
            return customrobotdatapacket.GetDataLength();
        }

        if(extsupplyprojectileactionpacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(extsupplyprojectileactionpacket.m_mutex);
            RM_referee::ExtSupplyProjectileActionStruct& frontExtSupplyProjectileActionElement = extsupplyprojectileactionpacket.m_queue.front();
            const boost::asio::detail::buffered_stream_storage::byte_type* dataStart = reinterpret_cast<const boost::asio::detail::buffered_stream_storage::byte_type*>(&frontExtSupplyProjectileActionElement);
            const boost::asio::detail::buffered_stream_storage::byte_type* dataEnd = dataStart + extsupplyprojectileactionpacket.GetDataLength();
            Pdata.insert(Pdata.end(), dataStart, dataEnd);
            extsupplyprojectileactionpacket.m_queue.pop();
            extsupplyprojectileactionpacket.m_mutex.unlock();
            printf("Mapserialize success ! cmd_id id : 0x%x DataLength:%d\n",cmd_id,extsupplyprojectileactionpacket.GetDataLength());
            return extsupplyprojectileactionpacket.GetDataLength();
        }

        printf("current cmd_id does not exist! error id : 0x%x\n",cmd_id);
        return 0;
    }

    uint16_t TypeMethodsTables::MapSearchDataLength(const uint16_t cmd_id ) {
        auto it = m_map.find(cmd_id);
        if(it!=m_map.end()) {
            return it->second->GetDataLength();
        } else {
            printf("current cmd_id does not exist! error id : 0x%x\n",cmd_id);
            return 0;
        }
    }

    std::shared_ptr<RefereePacket> TypeMethodsTables::MapGetData(const uint16_t cmd_id ) {
        if(powerheatdatapacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(powerheatdatapacket.m_mutex);
            return std::shared_ptr<RM_referee::PowerHeatDataPacket>(&powerheatdatapacket, [](RM_referee::PowerHeatDataPacket*){});
        }

        if(customrobotdatapacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(customrobotdatapacket.m_mutex);
            return std::shared_ptr<RM_referee::CustomRobotDataPacket>(&customrobotdatapacket, [](RM_referee::CustomRobotDataPacket*){});
        }

        if(extsupplyprojectileactionpacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(extsupplyprojectileactionpacket.m_mutex);
            return std::shared_ptr<RM_referee::ExtSupplyProjectileActionPacket>(&extsupplyprojectileactionpacket, [](RM_referee::ExtSupplyProjectileActionPacket*){});
        }

        printf("current cmd_id does not exist! error id : 0x%x\n",cmd_id);
        return nullptr;
    }

    uint16_t TypeMethodsTables::FilledPacketData(void* Pdest ,const uint16_t PdestSize , const uint16_t cmd_id ) {
        if(powerheatdatapacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(powerheatdatapacket.m_mutex);
            if (!powerheatdatapacket.m_queue.empty()) {
                if (PdestSize >= powerheatdatapacket.GetDataLength()) {
                    std::memcpy(Pdest,&powerheatdatapacket.m_queue.front(), powerheatdatapacket.GetDataLength());
                    return powerheatdatapacket.GetID();
                } else {
                    // Pdest没有足够的空间，处理错误...
                }
            } else {
                // 队列为空，处理错误...
            }        
        }

        if(customrobotdatapacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(customrobotdatapacket.m_mutex);
            if (!customrobotdatapacket.m_queue.empty()) {
                if (PdestSize >= customrobotdatapacket.GetDataLength()) {
                    std::memcpy(Pdest,&customrobotdatapacket.m_queue.front(), customrobotdatapacket.GetDataLength());
                    return customrobotdatapacket.GetID();
                } else {
                    // Pdest没有足够的空间，处理错误...
                }
            } else {
                // 队列为空，处理错误...
            }        
        }

        if(extsupplyprojectileactionpacket.GetID() == cmd_id) {
            std::lock_guard<std::mutex> lock(extsupplyprojectileactionpacket.m_mutex);
            if (!extsupplyprojectileactionpacket.m_queue.empty()) {
                if (PdestSize >= extsupplyprojectileactionpacket.GetDataLength()) {
                    std::memcpy(Pdest,&extsupplyprojectileactionpacket.m_queue.front(), extsupplyprojectileactionpacket.GetDataLength());
                    return extsupplyprojectileactionpacket.GetID();
                } else {
                    // Pdest没有足够的空间，处理错误...
                }
            } else {
                // 队列为空，处理错误...
            }        
        }

        printf("current cmd_id does not exist! error id : 0x%x\n",cmd_id);
        return 0x0000;
    }

    void TypeMethodsTables::SerialReadAsync(boost::asio::serial_port& serialPort,std::vector<uint8_t>& buffer) {
        boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(sizeof(RM_referee::PacketHeader)),
            [&, header = RM_referee::PacketHeader()](const boost::system::error_code& ec, std::size_t bytes_transferred) mutable {
                if (!ec) {
                    /** 
                     * TODO:
                     * 数据处理逻辑
                     * 0.第一字节默认0xA5
                     * 1.接受长度小于头长度，退出，等待下次 ？
                     * 2.直接memcpy（大小端?）  
                     * 3.CRC校验
                     *  清除处理过的头数据包，开始数据端处理
                     * 0.检查buffer剩余数据长度
                     * 1.正常则处理，缺少则等待数据再处理（接受其他的不完整数据？）
                     * 2.CRC16校验
                     * 3.调用Factory.Solve()
                    */
                    auto it = buffer.begin();
                    while (*it != RM_referee::StartOfFrame) {
                        it++;
                        if (it == buffer.end())
                            return 0;
                    }
                    if(bytes_transferred < sizeof(RM_referee::PacketHeader))
                        return 0;
                    std::memcpy(&header, &(*it), sizeof(RM_referee::PacketHeader));
                    //CRC8
                    // header.CRC8;
                    it+=sizeof(RM_referee::PacketHeader);
                    buffer.erase(buffer.begin(), it);
                    uint16_t cmd_id = static_cast<uint16_t>(*it << 8 | *(it+1));
                    //CRC16
                    //CRC16();
                    MapSolve(cmd_id,&(*(it+2)),header.DataLength);
                    /*
                    boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(header.DataLength),
                        [&, data = std::move(header)](const boost::system::error_code& ec, std::size_t bytes_transferred) mutable {
                            if (!ec) {
                                // 在这里处理完整的数据段
                                std::cout << "Received data: ";
                                for (auto byte : buffer) {
                                    std::cout << static_cast<int>(byte);
                                }
                                std::cout << std::endl;

                                // 继续下一次异步读取
                                // boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(sizeof(DataHeader)),
                                //     [&, header = DataHeader()](const boost::system::error_code& ec, std::size_t bytes_transferred) mutable {
                                //         // 处理下一次异步读取的完成事件
                                //     });
                            } else {
                                std::cerr << "Error: Incomplete data." << std::endl;
                            }
                        });
                    */
                } else {
                    std::cerr << "Error: Incomplete header." << std::endl;
                }
                return 0;
            }
            
        );

    };



    std::vector<uint8_t> buffer;
    RM_referee::PacketHeader header;
    std::vector<boost::asio::detail::buffered_stream_storage::byte_type>::iterator it;//重复执行
    int TypeMethodsTables::read() {
        system("pwd");
        std::ifstream file("/home/suzuki/RefereeSystem/src/referee_system/samples2.txt");

        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);

                while (!iss.eof()) {
                    int byte;
                    iss >> std::hex >> byte;
                    buffer.push_back(static_cast<uint8_t>(byte));
                }

                // 演示：输出读取的十六进制数据
                // std::cout << "Read Hex Data: ";
                // for (const auto& byte : buffer) {
                //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                // }
                // std::cout << std::endl;
                // std::cout<<std::dec<<buffer.size();
            }

            file.close();
        } else {
            std::cerr << "Unable to open the file." << std::endl;
        }

        header = RM_referee::PacketHeader();
        it = buffer.begin();//重复执行
        return 0;
    };
    void TypeMethodsTables::testprocess() {
            std::cout<<"\n"<<buffer.size()<<"\n";
            if(buffer.size() == 0) {
                read();
                return;
            };
            if(buffer.size() <= sizeof(RM_referee::PacketHeader))
                return ;
            while (*it != RM_referee::StartOfFrame) {
                it++;
                if (it == buffer.end()) {
                    // std::cout<<"No start of frame found! buffer.size:"<<buffer.size()<<"\n";
                    read();
                    // exit(0);
                    return;
                }
            }
            buffer.erase(buffer.begin(), it);
            it = buffer.begin();
            std::memcpy(&header, &(*it), sizeof(RM_referee::PacketHeader));
            //CRC8
            if(crc8.Verify_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&header),sizeof(RM_referee::PacketHeader))) {
                it += sizeof(RM_referee::PacketHeader);
                uint16_t cmd_id = static_cast<uint16_t>( ((*it)<<8) | (*(it+1)) );
                //CRC16
                if(crc16.Verify_CRC16_Check_Sum(&(*buffer.begin()),sizeof(RM_referee::PacketHeader) + 2 + header.DataLength + 2)) {
                    uint16_t erased = MapSolve(cmd_id,&(*(it+2)),header.DataLength);
                    if(erased) it += 2+erased+2;
                }
            } else
                it ++;

    };

}