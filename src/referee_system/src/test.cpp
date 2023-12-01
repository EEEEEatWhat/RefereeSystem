#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include "DataType.h"
#include "MappingTables.h"


void SerialReadAsync(boost::asio::serial_port& serialPort,std::vector<uint8_t>& buffer) {
    boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(sizeof(RM_referee::PacketHeader)),
        [&, header = RM_referee::PacketHeader()](const boost::system::error_code& ec, std::size_t bytes_transferred) mutable {
            if (!ec) {
                /**
                 * 数据处理逻辑
                 * 0.第一字节默认0xA5
                 * 1.接受长度小于头长度，退出，等待下次 ？
                 * 2.直接memcpy（大小端?）  
                 * 3.CRC校验
                 *  清除处理过的头数据包，开始数据端处理
                 * 0.检查buffer剩余数据长度
                 * 1.正常则处理，缺少则等待数据再处理（接受其他的不完整数据？）
                */
                auto it = buffer.begin();
                while (*it != RM_referee::StartOfFrame) {
                    it++;
                    if (it == buffer.end())
                        return 0;
                }
                if(bytes_transferred < sizeof(RM_referee::PacketHeader))
                    return;
                std::memcpy(&header, &(*it), sizeof(RM_referee::PacketHeader));
                //CRC
                header.CRC8;
                it+=sizeof(RM_referee::PacketHeader);
                buffer.erase(buffer.begin(), it);
                uint16_t cmd_id = static_cast<uint16_t>(*it << 8 | *(++it));

                boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(header.DataLength),
                    [&, data = std::move(header)](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) mutable {
                        if (!ec) {
                            // 在这里处理完整的数据段
                            std::cout << "Received data: ";
                            for (auto byte : buffer) {
                                std::cout << static_cast<int>(byte);
                            }
                            std::cout << std::endl;

                            // 继续下一次异步读取
                            // boost::asio::async_read(serialPort, boost::asio::buffer(buffer), boost::asio::transfer_exactly(sizeof(DataHeader)),
                            //     [&, header = DataHeader()](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) mutable {
                            //         // 处理下一次异步读取的完成事件
                            //     });
                        } else {
                            std::cerr << "Error: Incomplete data." << std::endl;
                        }
                    });
            } else {
                std::cerr << "Error: Incomplete header." << std::endl;
            }
        });

};

int main(int argc, char const *argv[])
{
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");
    std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
    SerialReadAsync(serialPort,buffer);
    RM_referee::TypeMethodsTables Factory_;
    Factory_.AddTypeMethod<RM_referee::GameStatusPacket>(0x0001);
    return 0;
}






