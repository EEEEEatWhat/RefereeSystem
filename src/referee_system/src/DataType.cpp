#include "DataType.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
namespace RM_referee{
    /**
     * 对于每一个数据包，都需要实现SolvePacket方法
     * 但是对于不同的数据包，SolvePacket方法的实现是不同的
     * 有的数据包内存布局和接受的数据包一样，有的数据包内存布局和接受的数据包不一样 !
     * 布局相同的数据包，使用宏来实现SolvePacket方法
     * WARNING: 由于宏的特性，无法输出数据包的具体内容，测试无误后再使用宏
    */
    #define PACKETCOPY(TYPE) \
    uint16_t TYPE::SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size)  {  \
        if(cmd_id != GetID()) \
            std::cout<<"SolveMethod does not match ID !\n"; \
        std::lock_guard<std::mutex> lock(m_mutex); \
        std::memcpy(&m_value,data,data_size); \
        m_mutex.unlock(); \
        return GetDataLength(); \
    };
    /**
        uint16_t PowerHeatDataPacket::SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size)  {
            if(cmd_id != GetID())
                std::cout<<"SolveMethod does not match ID !\n";
            std::memcpy(&m_value,data,data_size);
            return DateLength();
        };
    */
    PACKETCOPY(PowerHeatDataPacket)

    PACKETCOPY(PlaygroundEventPacket)
    
    PACKETCOPY(CustomRobotDataPacket)

    PACKETCOPY(ExtSupplyProjectileActionPacket)

}