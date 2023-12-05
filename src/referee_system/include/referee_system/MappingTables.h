/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/
#ifndef MAPPING_TABLES_H
#define MAPPING_TABLES_H
#pragma once 
#include "DataType.h"
#include <memory>
#include <unordered_map>
#include <map>

namespace RM_referee {
    using std::hex;

    class TypeMethodsTables {
    protected:
        std::map< uint16_t, std::shared_ptr<RefereePacket>> m_map;
    public:
        TypeMethodsTables();
        ~TypeMethodsTables();
        void testprocess();
        int read() ;

    /**
     * @brief  向工厂添加ID和绑定的类
     * @param  Type 类型名
     * @param  cmd_id 
     * @return NONE
    */
    template <typename Type>
    void AddTypeMethod(uint16_t cmd_id) {
        m_map.emplace(cmd_id, std::make_shared<Type>());
    }

        void SerialReadAsync(boost::asio::serial_port& ,std::vector<uint8_t>& );
    protected:
    /**
     * @brief   通过map键值对解包
                查表效率可能没有switch高，后续看情况选择
                注意检验cmd_id
     * @param   cmd_id 键
     * @param   data 待解包数据
     * @param   buf 解包后存放位置
     * @return  已经处理的字节数
     * @warning 注意内存大小不要越界访问
    */
        uint16_t MapSolve(const uint16_t cmd_id, uint8_t * data ,uint16_t data_size);

    };
    

}  // namespace RM_referee


#endif // MAPPING_TABLES_H
