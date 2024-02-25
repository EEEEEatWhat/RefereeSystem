/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/
#ifndef MAPPING_TABLES_H
#define MAPPING_TABLES_H
#pragma once 
#include "DataType.h"
#include "CRCcheck.h"
#include <memory>
#include <unordered_map>
#include <map>

namespace RM_referee {
    using std::hex;

    class TypeMethodsTables {
    protected:
        std::map< uint16_t, std::shared_ptr<RefereePacket>> m_map;
        CRC8 crc8;
        CRC16 crc16;
        // GameStatusPacket gamestatuspacket;
        ExtSupplyProjectileActionPacket extsupplyprojectileactionpacket;
        PowerHeatDataPacket powerheatdatapacket;
        CustomRobotDataPacket customrobotdatapacket;
    public:
        /**
         * @brief 构造函数将所有类与id绑定
        */
        TypeMethodsTables();
        ~TypeMethodsTables();
    /**
     * @brief 测试
    */
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
        
    /**
     * @brief   通过map键值对解包
                注意检验cmd_id
     * @param   cmd_id 键
     * @param   data 待解包数据
     * @param   data_size 数据包大小
     * @return  已经处理的字节数
    */
        uint16_t MapSolve(const uint16_t cmd_id, uint8_t * data ,uint16_t data_size);
    /**
     * @brief   通过map键值对查找
     *          数据序列化接口，序列化数据后通过server/client发送
     * @param   
     * @param   
     * @param   
     * @return  None
     * @warning 
    */
        void Mapserialize(uint8_t * data ,const uint16_t cmd_id );

    };
    

}  // namespace RM_referee


#endif // MAPPING_TABLES_H
