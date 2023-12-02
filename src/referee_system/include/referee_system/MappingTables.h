#pragma once 
#include "DataType.h"
#include <memory>
#include <unordered_map>

    /**
        @brief 通过映射表来解析数据
        @param 映射表接受cmd_id，对应DJI裁判系统附录
        @param 解析包函数，通过继承数据包基类并重写Solve函数提供解包方法
        @date  2023/11/27 基于DJI裁判系统协议V1.5'
        @return NULL
    */
namespace RM_referee {
    using std::hex;

    class TypeMethodsTables {
    protected:
        std::unordered_map< uint16_t, std::shared_ptr<GameStatusPacket>> m_map;
    public:
        TypeMethodsTables();
        ~TypeMethodsTables();

        template <typename Type>
        void AddTypeMethod(uint16_t);
        void SerialReadAsync(boost::asio::serial_port& ,std::vector<uint8_t>& );
    protected:
        void MapSolve(const uint16_t, uint8_t *, uint8_t *);
    };

    

}  // namespace RM_referee


