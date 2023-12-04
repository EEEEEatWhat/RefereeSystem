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
         * @brief 注意检验cmd_id
        */
        void MapSolve(const uint16_t, uint8_t *, uint8_t *);

    };
    

}  // namespace RM_referee


#endif // MAPPING_TABLES_H
