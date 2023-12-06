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
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size){
        std::cout<<"[Warning : This should not appear !]\n";
        return 0 ;
        };
    };
    
    #define GENERATEPACK(packname) 

    /** How To Use
     * 
     *  GENERATEPACK[GameStatusPacket];
     * 
     *      is the same with 
     * 
     *  class GameResultPacket : public RefereePacket {
     *  public:
     *  virtual uint16_t SolvePacket(uint8_t*,uint8_t*) override {};
     * 
    };
    */


    //0x0001 GameStatusPacket 11
    class GameStatusPacket : public RefereePacket { 
    public:
        static uint16_t GetID(){return uint16_t(PacketType::GameStatus);};
        // uint16_t DateLength(){return sizeof(m_value);};
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override {return 0;}; 
    };

    //0x0002 GameResultPacket 1
    class GameResultPacket : public RefereePacket { 
    public:
        static uint16_t GetID(){return uint16_t(PacketType::GameResultEvent);};
        // uint16_t DateLength(){return sizeof(m_value);};
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override {return 0;}; 
    };

    //0x0003 GameRobotHPPacket 32
    class GameRobotHPPacket : public RefereePacket { 
    public:
        static uint16_t GetID(){return uint16_t(PacketType::GameRobotHP);};
        // uint16_t DateLength(){return sizeof(m_value);};
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override {return 0;}; 
    };
    //0x0101 4
    //0x0102 4
    class ExtSupplyProjectileAction : public RefereePacket { 
    public:
        struct S{ 
            uint8_t reserved; 
            uint8_t supply_robot_id;  
            uint8_t supply_projectile_step; 
            uint8_t supply_projectile_num; 
        } m_value;
        static_assert(sizeof(m_value) == 4, "ExtSupplyProjectileAction must be 4 bytes long with packing");
        std::vector<S> queue;//解包函数push，读取数据维护队列长度。
        static uint16_t GetID(){return uint16_t(PacketType::ExtSupplyProjectileAction);};
        uint16_t DateLength(){return sizeof(m_value);};
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; 
    };
    //0x0104 2
    //0x0105 1
    //0x0201 27
    //0x0202 PowerHeatDataPacket 16
    class PowerHeatDataPacket : public RefereePacket { 
    public:
        struct S{ 
            uint16_t chassis_voltage; 
            uint16_t chassis_current; 
            float chassis_power; 
            uint16_t buffer_energy; 
            uint16_t shooter_17mm_1_barrel_heat; 
            uint16_t shooter_17mm_2_barrel_heat; 
            uint16_t shooter_42mm_barrel_heat; 
        } m_value; 
        static_assert(sizeof(m_value) == 16, "PowerHeatDataPacket must be 16 bytes long with packing");
        std::vector<S> queue;

        static uint16_t GetID(){return uint16_t(PacketType::PowerHeatData);};
        uint16_t DateLength(){return sizeof(m_value);};
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; 
    };
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
    //0x0302 CustomRobotDataPacket 30
    class CustomRobotDataPacket : public RefereePacket { 
    public:
        struct { 
            uint8_t data[30];   
        } m_value;
        static_assert(sizeof(m_value) == 30, "CustomRobotDataPacket must be 30 bytes long with packing");

        static uint16_t GetID(){return uint16_t(PacketType::CustomRobotData);};
        uint16_t DateLength(){return sizeof(m_value);};
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; 
    };
    //0x0303 15
    //0x0304 12 
    //0x0305 10
    //0x0306 8
    //0x0307 103
    //0x0308 34

}