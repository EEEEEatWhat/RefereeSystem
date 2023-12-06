/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#include <memory>
#include <functional>
#include <boost/asio.hpp>
#include"MappingTables.h"
//below is for test 
    #include <iostream>
    #include <fstream>
    #include <sstream>
    #include <vector>
    #include <iomanip>
//upside is for test 
namespace RM_referee{
    TypeMethodsTables::TypeMethodsTables(){}
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
        std::ifstream file("../samples2.txt");

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
                std::cout << "Read Hex Data: ";
                for (const auto& byte : buffer) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                }
                std::cout << std::endl;
                std::cout<<std::dec<<buffer.size();
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
            // if(bytes_transferred < sizeof(RM_referee::PacketHeader))
            //     return ;
            std::cout<<"\n"<<buffer.size()<<"\n";
            if(buffer.size()==89126)
                std::cout<<"stop here!";
            if(buffer.size() <= sizeof(RM_referee::PacketHeader))
                return ;
            while (*it != RM_referee::StartOfFrame) {
                it++;
                if (it == buffer.end()) {
                    buffer.erase(buffer.begin(), it);
                    return;
                }
            }
            std::memcpy(&header, &(*it), sizeof(RM_referee::PacketHeader));
            //CRC8
            // header.CRC8;
            it += sizeof(RM_referee::PacketHeader);
            /*****************************/
            buffer.erase(buffer.begin(), it);
            it = buffer.begin();
            /* 
            另一种写法不擦除buffer，继续处理
            ******************************/
            uint16_t cmd_id = static_cast<uint16_t>( ((*it)<<8) | (*(it+1)) );
            //CRC16
            //CRC16();
            uint16_t erased = MapSolve(cmd_id,&(*(it+2)),header.DataLength);
            if(erased)
                it += 2+erased+2;
            // buffer.erase(buffer.begin(), it);
            // it = buffer.begin();
    };

}