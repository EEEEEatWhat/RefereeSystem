#include <memory>
#include <functional>
#include <boost/asio.hpp>
#include"MappingTables.h"

namespace RM_referee{
    TypeMethodsTables::TypeMethodsTables(){}
    TypeMethodsTables::~TypeMethodsTables() {}
    /**
     * @brief  向工厂添加ID和绑定的类
     * @param  Type 类型名
     * @param  cmd_id 
     * @return NONE
    */
    template <typename Type>
    void TypeMethodsTables::AddTypeMethod(uint16_t cmd_id) {
        m_map.emplace(cmd_id, std::make_shared<Type>());
    }

    /// @brief 通过map键值对解包
    /// @brief 查表效率可能没有switch高，后续看情况选择
    /// @param cmd_id 键
    /// @param data 待解包数据
    /// @param buf 解包后存放位置
    /// @warning 注意内存大小不要越界访问
    void TypeMethodsTables::MapSolve(const uint16_t cmd_id , uint8_t* data ,uint8_t* buf){
        auto it = m_map.find(cmd_id);
        it->second->SolvePacket(data ,buf );
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
                     * 
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
                    //CRC8
                    header.CRC8;
                    it+=sizeof(RM_referee::PacketHeader);
                    buffer.erase(buffer.begin(), it);
                    uint16_t cmd_id = static_cast<uint16_t>(*it << 8 | *(it+1));
                    //CRC16
                    //CRC16();
                    std::shared_ptr<uint8_t> parry(new uint8_t[header.DataLength]);//buf 只是临时设想！
                    MapSolve(cmd_id,&(*(it+2)),&(*parry));
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
            });

    };
}