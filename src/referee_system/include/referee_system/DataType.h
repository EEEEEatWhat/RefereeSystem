/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#pragma once 
// #include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <queue>
#include "enums.h"
namespace RM_referee{
    // Packet header structure
    #pragma pack(1)
    struct PacketHeader {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t SequenceNumber;
        uint8_t CRC8;
    } ;
    #pragma pack()
    static_assert(sizeof(PacketHeader) == 5, "PacketHeader must be 5 bytes long with packing");
    static constexpr uint8_t StartOfFrame = 0xa5;

    // Base packet
    class RefereePacket {
    public:
        RefereePacket(){};
        ~RefereePacket(){};
        /**
         * @return 处理的字节数
        */
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) {
        std::cout<<"[Warning : This should not appear !]\n";
        return 0 ;
        };
        //TODO 添加获取数据结构体的接口，再将其序列化发送
        //TODO 或者添加数据结构体序列化后的接口
        virtual void GetDateStruct() {};
    };
    

    #define GENERATEPACK(NAME,TYPE,STRUCT) \
    class NAME : public RefereePacket { \
    public:\
        STRUCT m_value;\
        std::queue<STRUCT> m_queue;\
        static uint16_t GetID(){return uint16_t(PacketType::TYPE);};\
        uint16_t DateLength(){return sizeof(STRUCT);};\
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; \
    };

    /**
    @brief  How to use GENERATEPACK(NAME,TYPE,STRUCT):
            Example：0x0102 ExtSupplyProjectile 4 ExtSupplyProjectileAction
            定义数据包结构体并断言数据包大小
            
            struct ExtSupplyProjectileStruct { 
                uint8_t reserved; 
                uint8_t supply_robot_id;  
                uint8_t supply_projectile_step; 
                uint8_t supply_projectile_num; 
            };
            static_assert(sizeof(S) == 4, "ExtSupplyProjectileStruct must be 4 bytes long with packing");
            GENERATEPACK(ExtSupplyProjectile,ExtSupplyProjectileAction,ExtSupplyProjectileStruct)
    @warning  name,type,struct不要重复；type请查阅enum.h
    @brief 等效于：
            //0x0102 ExtSupplyProjectile 4
            class ExtSupplyProjectile : public RefereePacket { 
            public:
                struct ExtSupplyProjectileStruct{ 
                    uint8_t reserved; 
                    uint8_t supply_robot_id;  
                    uint8_t supply_projectile_step; 
                    uint8_t supply_projectile_num; 
                } m_value;
                static_assert(sizeof(ExtSupplyProjectileStruct) == 4, "ExtSupplyProjectileStruct must be 4 bytes long with packing");
                std::queue<ExtSupplyProjectileStruct> m_queue;
                static uint16_t GetID(){return uint16_t(PacketType::ExtSupplyProjectileAction);};
                uint16_t DateLength(){return sizeof(ExtSupplyProjectileStruct);};
                virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; 
            };
    */


    //0x0001 GameStatusPacket 11 GameStatus
    struct GameStatusPacketStruct { 
        uint8_t game_type : 4; 
        uint8_t game_progress : 4; 
        uint16_t stage_remain_time; 
        uint64_t SyncTimeStamp; }; 
    // static_assert(sizeof(ExtSupplyProjectileStruct) == 4, "ExtSupplyProjectileStruct must be 4 bytes long with packing");
    GENERATEPACK(GameStatusPacket,GameStatus,GameStatusPacketStruct)
    //0x0002 GameResultPacket 1 GameResultEvent
    struct GameResultPacketStruct { 
        uint8_t winner; }; 
    // static_assert(sizeof(ExtSupplyProjectileStruct) == 4, "ExtSupplyProjectileStruct must be 4 bytes long with packing");
    GENERATEPACK(GameResultPacket,GameResultEvent,GameResultPacketStruct)

    //0x0003 GameRobotHPPacket 32 GameRobotHP
    struct GameRobotHPPacketStruct { 
        uint16_t red_1_robot_HP; 
        uint16_t red_2_robot_HP; 
        uint16_t red_3_robot_HP; 
        uint16_t red_4_robot_HP; 
        uint16_t red_5_robot_HP; 
        uint16_t red_7_robot_HP; 
        uint16_t red_outpost_HP; 
        uint16_t red_base_HP; 
        uint16_t blue_1_robot_HP; 
        uint16_t blue_2_robot_HP; 
        uint16_t blue_3_robot_HP; 
        uint16_t blue_4_robot_HP; 
        uint16_t blue_5_robot_HP; 
        uint16_t blue_7_robot_HP; 
        uint16_t blue_outpost_HP; 
        uint16_t blue_base_HP; };
    // static_assert(sizeof(ExtSupplyProjectileStruct) == 4, "ExtSupplyProjectileStruct must be 4 bytes long with packing");
    GENERATEPACK(GameRobotHPPacket,GameRobotHP,GameRobotHPPacketStruct)

    //0x0101 4
    //0x0102 ExtSupplyProjectile 4 ExtSupplyProjectileAction
    struct ExtSupplyProjectileStruct { 
        uint8_t reserved; 
        uint8_t supply_robot_id;  
        uint8_t supply_projectile_step; 
        uint8_t supply_projectile_num; };
    static_assert(sizeof(ExtSupplyProjectileStruct) == 4, "ExtSupplyProjectileStruct must be 4 bytes long with packing");
    GENERATEPACK(ExtSupplyProjectile,ExtSupplyProjectileAction,ExtSupplyProjectileStruct)
    //0x0104 2
    //0x0105 1
    //0x0201 27
    //0x0202 PowerHeatDataPacket 16 PowerHeatData
    struct PowerHeatDataPacketStruct { 
        uint16_t chassis_voltage; 
        uint16_t chassis_current; 
        float chassis_power; 
        uint16_t buffer_energy; 
        uint16_t shooter_17mm_1_barrel_heat; 
        uint16_t shooter_17mm_2_barrel_heat; 
        uint16_t shooter_42mm_barrel_heat; }; 
    static_assert(sizeof(PowerHeatDataPacketStruct) == 16, "PowerHeatDataPacketStruct must be 16 bytes long with packing");
    GENERATEPACK(PowerHeatDataPacket,PowerHeatData,PowerHeatDataPacketStruct)

    //0x0203 16
    //0x0204 1
    //0x0205 1
    //0x0206 1
    //0x0207 7
    //0x0208 6
    //0x0209 4
    //0x020A 6
    //0x020B 40
    //0x020C 6
    //0x020D 4
    //0x020E 1
    //0x0301 128
    //0x0302 CustomRobotDataPacket 30 CustomRobotData
    struct CustomRobotDataPacketStruct { 
        uint8_t data[30]; };
    static_assert(sizeof(CustomRobotDataPacketStruct) == 30, "CustomRobotDataPacketStruct must be 30 bytes long with packing");
    GENERATEPACK(CustomRobotDataPacket,CustomRobotData,CustomRobotDataPacketStruct)
    //0x0303 15
    //0x0304 12 
    //0x0305 10
    //0x0306 8
    //0x0307 103
    //0x0308 34

}