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
#include <mutex>

namespace RM_referee {
    using std::hex;

    class TypeMethodsTables {
    protected:
        std::map< uint16_t, std::shared_ptr<RefereePacket>> m_map;
        std::mutex m_map_mutex;
        CRC8 crc8;
        CRC16 crc16;
        // GameStatusPacket gamestatuspacket;
        ExtSupplyProjectileActionPacket extsupplyprojectileactionpacket;
        PowerHeatDataPacket powerheatdatapacket;
        CustomRobotDataPacket customrobotdatapacket;
        PlaygroundEventPacket playgroundeventpacket;
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
         * @brief  通过map键值对解包并存储到类内的队列中
        * @param   cmd_id 键
        * @param   data 待解包数据
        * @param   data_size 数据包大小
        * @return  已经处理的字节数
        */
        uint16_t MapSolve(const uint16_t cmd_id, uint8_t * data ,uint16_t data_size);

        /**
         * @brief   通过map键值对查找
         *          数据序列化接口，序列化数据后通过server/client发送
         * @param   Pdata 序列化后的目标的地址
         * @param   cmd_id 键，请求序列化的数据包
         * @return  已处理的字节数
         * @warning 
        */
        uint16_t Mapserialize(std::vector<boost::asio::detail::buffered_stream_storage::byte_type>  &Pdata ,const uint16_t cmd_id );

        /**
         * @brief   通过map键值对查找获取数据包长度
         * @param   cmd_id 键，请求序列化的数据包
         * @return  数据包长度
        */
        uint16_t MapSearchDataLength(const uint16_t cmd_id );

        /**
         * @brief   通过map键值对查找获取数据包
         * @param   cmd_id 键，请求的数据包
         * @return  数据包
         * @warning 未实现
        */
        std::shared_ptr<RefereePacket> MapGetData(const uint16_t cmd_id );

        /**
         * @brief   通过map键值对查找解包方法
         * @param   Pdest 解包后存放的地址
         * @param   cmd_id 键，请求的数据包
         * @return  已处理的CMD_ID
        */
        uint16_t FilledPacketData(void* Pdest ,const uint16_t PdestSize , const uint16_t cmd_id );
    };
    

}  // namespace RM_referee


#endif // MAPPING_TABLES_H
