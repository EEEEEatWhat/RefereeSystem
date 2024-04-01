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
#include <queue>

namespace RM_referee {
    using std::hex;

    class TypeMethodsTables {
    protected:
        std::vector<boost::asio::detail::buffered_stream_storage::byte_type> data;
        std::map< uint16_t, std::shared_ptr<RefereePacket>> m_map;
        std::mutex m_map_mutex;
        std::vector<uint8_t> dataQueue_;
        std::mutex dataQueue_mutex;
        std::condition_variable condVar_;

        CRC8 crc8;
        CRC16 crc16;

        GameStatusPacket gamestatuspacket ;
        GameResultEventPacket gameresulteventpacket ; 
        GameRobotHPPacket gamerobothppacket ;
        PlaygroundEventPacket playgroundeventpacket ;
        ExtSupplyProjectileActionPacket extsupplyprojectileactionpacket ;
        DartInfoPacket dartinfopacket ;
        RobotPositionPacket robotpositionpacket ;
        RobotStatePacket robotstatepacket ;
        RefereeWarningEventPacket refereewarningeventpacket ; 
        PowerHeatDataPacket powerheatdatapacket ;
        RobotBuffPacket robotbuffpacket ;
        AirSupportDataPacket airsupportdatapacket ;
        DamageEventPacket damageeventpacket ;
        ShootEventPacket shooteventpacket ;
        ProjectileAllowancePacket projectileallowancepacket ;
        RobotRfidStatePacket robotrfidstatepacket ;
        DartClientCmdPacket dartclientcmdpacket ;
        GroundRobotPositionPacket groundrobotpositionpacket ;
        RadarMarkDataPacket radarmarkdatapacket ;
        SentryInfoPacket sentryinfopacket ;
        CustomRobotDataPacket customrobotdatapacket ;
        MinimapInteractionCommsMessagePacket minimapinteractioncommsmessagepacket ;
        KeyboardMouseMessagePacket keyboardmousemessagepacket ;
        ClientMinimapRecvPacket clientminimaprecvpacket ;
        CustomClientDataPacket customclientdatapacket ;
        MapDataPacket mapdatapacket ;
        CustomInfoPacket custominfopacket ;

        //below is test
        std::vector<uint8_t> testbuffer;
        RM_referee::PacketHeader testheader;
        std::vector<boost::asio::detail::buffered_stream_storage::byte_type>::iterator it;

    public:
        std::atomic<bool> exitFlag_;

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
         * @return void
        */
        template <typename Type>
        void AddTypeMethod(uint16_t cmd_id) {
            m_map.emplace(cmd_id, std::make_shared<Type>());
        }

        /**
         * @brief  从串口读取数据
         * @param  serial_port 端口号
         * @return void
         * @warning 该函数需要单独线程调用
        */
        void SerialRead(boost::asio::serial_port& serialPort ,std::ofstream* file) ;        

        /**
         * @brief  处理数据
         * @return void
         * @warning 该函数需要单独线程调用
        */
        void ProcessData();


        /**
         * @brief  通过map键值对解包并存储到类内的结构体中
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
        std::vector<boost::asio::detail::buffered_stream_storage::byte_type>& Mapserialize(const uint16_t cmd_id ) ;

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
         * @warning 已弃用
        */
        std::shared_ptr<RefereePacket> MapGetData(const uint16_t cmd_id ) = delete;

        /**
         * @brief   通过map键值对查找解包方法
         * @param   Pdest 解包后存放的地址
         * @param   PdestSize 
         * @param   cmd_id 键，请求的数据包
         * @return  已处理的CMD_ID
         * @warning 已弃用  
        */
        uint16_t FilledPacketData(void* Pdest ,const uint16_t PdestSize , const uint16_t cmd_id ) = delete;

        /**
         * @brief   通过map键值对使用解包方法
         * @param   Pdest 解包后存放的目标地址
         * @param   PdestSize 目标地址的大小
         * @param   Pdata   待解包的数据地址
         * @param   PdataSize 待解包的数据大小
         * @param   cmd_id 数据包的键
         * @return  
        */
        uint16_t FilledPacketData(void* Pdest ,const uint16_t PdestSize ,void* Pdata , const uint16_t PdataSize , const uint16_t cmd_id );

    };
    

}  // namespace RM_referee


#endif // MAPPING_TABLES_H
